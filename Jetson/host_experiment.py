"""Nano experiment orchestration. The G3519 protocol adapter owns 0x03/0x83.

execute(name, *args) returns only after the matching successful board ACK.
It must raise on rejection/timeout and must not retry a start. next_status(deadline)
returns the next decoded 0x81 row (Nano monotonic seconds in 'received') or raises
TimeoutError. The vision producer keeps sending current 0x01 frames concurrently.
"""
import csv
import math
import time

from servo_scan import command_packet, degrees_to_ticks


class BoardController:
    def __init__(self, execute, send_packet, confirmed_ticks=None):
        self.execute = execute
        self.send_packet = send_packet
        self.confirmed_ticks = confirmed_ticks
        self.seq = 0

    def set_mode(self, mode):
        return self.execute('SET_MODE', mode)

    def start_auto(self):
        return self.execute('START_AUTO')

    def stop(self):
        return self.execute('STOP')

    def set_target(self, x_cm):
        return self.execute('SET_TARGET', x_cm)

    def set_pd(self, kp, kd):
        return self.execute('SET_PD', kp, kd)

    def start_remote(self):
        return self.execute('START_REMOTE')

    def set_servo_angle(self, deg):
        return self.set_pwm_ticks(degrees_to_ticks(deg))

    def set_pwm_ticks(self, ticks):
        if self.confirmed_ticks is None or type(ticks) is not int:
            raise ValueError('pulse outside confirmed tick range')
        seq = self.seq
        self.send_packet(command_packet(seq, ticks, self.confirmed_ticks))
        self.seq = (seq + 1) & 65535
        return seq


def run_auto(board, next_status, log, target_cm, kp, kd, duration, clock=time.monotonic, cancelled=None):
    """Run once. Start experimental time at board-confirmed AUTO. All times in seconds."""
    if not all(math.isfinite(x) for x in (target_cm, kp, kd, duration)) or duration <= 0:
        raise ValueError('expected finite target, gains and positive duration')
    events = csv.writer(log)
    events.writerow(('nano_time', 'event', 'board_ms', 'control_state', 'vision_valid',
                     'target_cm', 'ball_cm', 'ruler_cm', 'reason'))

    def record(event, row=None, reason=''):
        row = row or {}
        x = row.get('board_ball_cm') if row.get('board_vision_valid') else None
        events.writerow((clock(), event, row.get('g3519_ms'), row.get('control_state'),
                         row.get('board_vision_valid'), row.get('target_cm'), x,
                         None if x is None else 12.0 + x, reason))
        log.flush()

    def status(after, deadline):
        while clock() < deadline:
            row = next_status(deadline)
            now = clock()
            if now > deadline:
                break
            if after <= row['received'] <= now and now-row['received'] <= .2:
                return row
        raise TimeoutError('fresh board telemetry timeout')

    def check(row):
        if row['board_fault']:
            raise RuntimeError('board fault')
        if not math.isclose(row['target_cm'], target_cm, abs_tol=.001):
            raise RuntimeError('board target differs from requested rail coordinate')

    try:
        board.set_mode('UART')
        board.set_target(target_cm)
        board.set_pd(kp, kd)
        after = clock()
        deadline = after + 1.
        while True:
            row = status(after, deadline)
            check(row)
            if row['board_auto_enabled']:
                raise RuntimeError('AUTO already active before START_AUTO')
            if cancelled and cancelled.is_set():
                raise RuntimeError('vision session closed')
            if row['board_vision_valid'] and math.isfinite(row['board_ball_cm']):
                break
        record('vision_ready', row)
        after = clock()
        if cancelled and cancelled.is_set():
            raise RuntimeError('vision session closed')
        board.start_auto()
        deadline = clock() + .5
        while True:
            row = status(after, deadline)
            check(row)
            if row['control_state'] in (3, 4):
                raise RuntimeError('board exited during startup: state=%s' % row['control_state'])
            if row['control_state'] == 2 and row['board_auto_enabled'] and row['board_vision_valid']:
                break
        record('auto_confirmed', row)
        finish = clock() + duration
        while clock() < finish:
            if cancelled and cancelled.is_set():
                raise RuntimeError('vision session closed')
            row = status(after, clock() + .3)
            record('sample', row)
            check(row)
            if row['control_state'] != 2 or not row['board_auto_enabled']:
                raise RuntimeError('AUTO exited: state=%s vision=%s' %
                                   (row['control_state'], row['board_vision_valid']))
    except BaseException as exc:
        record('abort', reason=str(exc))
        raise
    finally:
        after = clock()
        try:
            board.stop()
            deadline = clock() + .5
            while True:
                row = status(after, deadline)
                if not row['board_auto_enabled'] and row['control_state'] != 2:
                    record('stop_confirmed', row)
                    break
        except Exception as exc:
            record('stop_unconfirmed', reason=str(exc))
            raise

def run_remote(board, next_debug, log, plan, hold_ms, neutral, clock=time.monotonic, cancelled=None):
    """Scan confirmed tick bounds. next_debug(deadline) reads real 0x82 telemetry.

    Rows use command_seq, board_timestamp, target_pwm_ticks, output_pwm_ticks,
    remote_enabled, result and received (local monotonic seconds). A read deadline
    without a frame raises TimeoutError. Output can be joined with join_servo_scan.
    """
    if hold_ms < 200 or hold_ms % 50:
        raise ValueError('hold at least two 100 ms telemetry periods, in 50 ms steps')
    low, high = board.confirmed_ticks
    if not low <= neutral <= high:
        raise ValueError('neutral must be inside the confirmed stroke')
    fields = ('nano_timestamp', 'event', 'command_seq', 'board_timestamp', 'target_pwm_ticks',
              'output_pwm_ticks', 'remote_enabled', 'result', 'plateau_index', 'reason')
    writer = csv.DictWriter(log, fields)
    writer.writeheader()

    def event(kind, **values):
        writer.writerow(dict(values, nano_timestamp=clock(), event=kind))
        log.flush()

    def read(end, after):
        while clock() < end:
            row = next_debug(end)
            if after <= row['received'] <= end and clock()-row['received'] <= .2:
                event('remote', **{k: row[k] for k in fields[2:8]})
                return row
        raise TimeoutError('fresh remote telemetry timeout')

    def send(ticks, stage):
        seq = board.set_pwm_ticks(ticks)
        event('tx', command_seq=seq, target_pwm_ticks=ticks, plateau_index=stage)
        return seq

    try:
        board.set_mode('REMOTE')
        after = clock()
        first = read(after+.4, after)
        if first['remote_enabled'] or first['output_pwm_ticks'] != neutral:
            raise RuntimeError('expected stopped board at neutral before scan')
        board.seq = (first['command_seq']+1) & 65535
        after = clock()
        seq = send(neutral, -1)
        end = after + .2
        while True:
            row = read(end, after)
            if row['command_seq'] == seq and not row['remote_enabled']:
                break
        if cancelled and cancelled.is_set():
            raise RuntimeError('vision session closed')
        board.start_remote()
        last_rx = clock()
        for i, target in enumerate(plan):
            after = clock()
            end = after + hold_ms/1000.
            due = after
            sequences = set()
            reached = False
            while clock() < end:
                if cancelled and cancelled.is_set():
                    raise RuntimeError('vision session closed')
                now = clock()
                if now-last_rx > .25:
                    raise TimeoutError('remote telemetry missing')
                if now >= due:
                    sequences.add(send(target.ticks, i))
                    due = clock() + .05
                try:
                    row = read(min(due, end), after)
                except TimeoutError:
                    continue
                last_rx = clock()
                if not row['remote_enabled'] or row['result'] != 0:
                    raise RuntimeError('REMOTE stopped or refused target: result=%s' % row['result'])
                if not low <= row['output_pwm_ticks'] <= high:
                    raise RuntimeError('board output outside confirmed stroke')
                if row['command_seq'] in sequences:
                    if row['target_pwm_ticks'] != target.ticks:
                        raise RuntimeError('board target differs from command')
                    if row['output_pwm_ticks'] == target.ticks:
                        reached = True
            if not reached:
                raise RuntimeError('board did not confirm this plateau')
    except BaseException as exc:
        event('abort', reason=str(exc))
        raise
    finally:
        after = clock()
        try:
            event('stop_requested')
            board.stop()
            end = clock()+1.
            while True:
                row = read(end, after)
                if not row['remote_enabled'] and row['output_pwm_ticks'] == neutral:
                    event('stop_confirmed')
                    break
        except Exception as exc:
            event('stop_unconfirmed', reason=str(exc))
            raise
