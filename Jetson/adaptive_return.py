"""Adaptive REMOTE control using SOE-timed measurements and actual PWM readback."""
import csv
import queue
import time
from collections import deque



def clip(value, low, high):
    return min(high, max(low, value))


def sign(value):
    return (value > 0) - (value < 0)


class Motion:
    """Fit velocity in cm/s on a short window of measured rail coordinates."""
    def __init__(self):
        self.samples = deque()

    def add(self, stamp, x):
        if self.samples and not 0 < stamp - self.samples[-1][0] <= .15:
            self.samples.clear()
        self.samples.append((stamp, x))
        while stamp - self.samples[0][0] > .24:
            self.samples.popleft()
        if len(self.samples) < 3 or stamp - self.samples[0][0] < .08:
            return None
        t0 = sum(t for t, _ in self.samples) / len(self.samples)
        x0 = sum(p for _, p in self.samples) / len(self.samples)
        variance = sum((t-t0)**2 for t, _ in self.samples)
        v = sum((t-t0)*(p-x0) for t, p in self.samples) / variance
        if max(abs(p-x0-v*(t-t0)) for t, p in self.samples) > .12:
            return None
        return v


class ReturnLaw:
    def __init__(self, bounds, neutral, slew, polarity=1, probe=None):
        self.bounds = bounds
        self.neutral = neutral
        self.slew = slew
        self.polarity = polarity
        self.probe = probe
        self.target = neutral
        self.changed = None
        self.anchor = None
        self.last_applied = neutral
        self.holding = False
        self.response = .12  # one 0x82 period plus scheduling
        self.command_time = None
        self.command_origin = neutral
        self.pwm_goal = neutral
        self.start_x = None
        self.start_v = None

    def written(self, stamp, target, applied):
        if self.pwm_goal != target:
            self.pwm_goal = target
            self.command_time = stamp
            self.command_origin = applied

    def feedback(self, stamp, output, echoed_target):
        if self.command_time is not None and output == echoed_target == self.pwm_goal:
            travel = abs(output - self.command_origin) / self.slew
            lag = clip(stamp - self.command_time - travel, .05, .30)
            self.response = .75*self.response + .25*lag
            self.command_time = None
        self.last_applied = output

    def check_actuator(self, stamp):
        if self.command_time is not None:
            travel = abs(self.pwm_goal-self.command_origin) / self.slew
            if stamp-self.command_time > travel + 2*self.response + .12:
                raise TimeoutError('no corresponding PWM response within travel time')

    def reset_observation(self):
        self.changed = None
        self.target = self.neutral
        self.start_x = None

    def step(self, stamp, x, v):
        neutral = self.neutral
        distance = abs(x)
        direction = sign(-x)
        toward = direction*v
        if distance >= 5.7 or abs(x + (.12+self.response)*v) >= 5.9:
            raise ValueError('rail end guard')
        if self.changed is None:
            self.changed, self.anchor = stamp, x
        if self.start_x is None:
            self.start_x, self.start_v, self.start_stamp = x, v, stamp

        drive = self.probe if self.probe else direction*self.polarity
        room = self.bounds[1]-neutral if drive > 0 else neutral-self.bounds[0]
        elapsed = stamp-self.changed
        moved = x-self.anchor
        stalled = abs(moved) < .08 and abs(v) < .35
        # Use exposure-time evidence, the observed response lag and remaining PWM travel.
        remaining = abs(self.last_applied-self.target)
        observe = self.response + remaining/self.slew + .10 + .12/(1+distance)
        near_stop = distance <= .16 + self.response*max(0.,toward) + max(0.,toward)**2/16.
        desired, phase = self.target, 'observe'

        if self.probe:
            # An already-moving ball does not by itself prove the sign of acceleration.
            residual = x - self.start_x - self.start_v*(stamp-self.start_stamp)
            if (sign(self.last_applied-neutral) == self.probe and abs(residual) >= .14
                    and abs(v-self.start_v) >= .55 and residual*(v-self.start_v) > 0):
                return neutral, 'probe_motion', stalled
        elif distance < (.30 if self.holding else .18) and abs(v)<.45:
            self.holding = True
            desired, phase = neutral, 'hold'
        elif (near_stop and toward > .45) or (distance < .20 and abs(v) > .55):
            self.holding = False
            brake = -sign(v)*self.polarity
            available = self.bounds[1]-neutral if brake > 0 else neutral-self.bounds[0]
            fraction = clip(abs(v)*(.18+self.response)/max(.18,distance), .08, 1.)
            desired, phase = neutral + brake*max(1, round(available*fraction)), 'brake'
        elif toward > max(.8, min(3., distance*1.25)):
            self.holding = False
            desired, phase = neutral, 'coast'

        if phase == 'observe' and elapsed >= observe:
            self.holding = False
            if self.probe or stalled or toward < max(.4, min(1.4, distance)):
                # Allow at most one unobserved increase; braking is independent of arrival.
                increment = max(1, round(room*clip(.05+.09*distance, .06, .35)))
                if not stalled and not self.probe:
                    increment = max(1, round(increment*.5))
                if remaining <= max(1, increment//3):
                    desired = neutral + drive*(max(0, drive*(self.target-neutral)) + increment)
                    desired = clip(desired, *self.bounds)
                    phase = 'probe' if self.probe else ('unstick' if stalled else 'drive')
                    if (desired == self.target == self.last_applied and
                            elapsed > observe + .5 and (self.probe or stalled or toward < 0)):
                        raise ValueError('stroke limit without useful motion')
        if desired != self.target:
            self.target = int(desired)
            self.changed, self.anchor = stamp, x
        return self.target, phase, stalled



def run(board, port, stream, seconds, cancelled, profile, probe=None, polarity=1):
    now = time.monotonic
    neutral = profile['neutral_ticks']
    fields = ('event', 'nano_s', 'frame', 'capture_s', 'x_cm', 'target_cm', 'error_cm',
              'velocity_cm_s', 'valid', 'reason', 'phase', 'stalled', 'target_ticks',
              'output_ticks', 'seq', 'board_ms')
    log = csv.DictWriter(stream, fields)
    log.writeheader()

    def record(event, **values):
        log.writerow(dict(values, event=event, nano_s=now()))

    def remote_after(t, end):
        while now() < end:
            row = port.next_debug(end)
            if row['received'] >= t:
                return row
        raise TimeoutError('remote telemetry missing')

    try:
        board.set_mode('REMOTE')
        after = now()
        end = after + 1.
        while True:
            row = remote_after(after, end)
            if (not row['remote_enabled'] and
                    row['target_pwm_ticks'] == row['output_pwm_ticks'] == neutral):
                break
        board.seq = (row['command_seq']+1) & 65535
        motion = Motion()
        frame = None
        end = now()+4.
        while now() < end:
            if cancelled.is_set():
                raise RuntimeError('vision stopped')
            m = getattr(port, 'measurement', None)
            if m and m[0] != frame:
                frame, t, x, _, received = m
                if x is None or not 0 <= now()-t <= .1:
                    motion.samples.clear()
                else:
                    v = motion.add(t, x)
                    if v is not None:
                        if abs(x) >= 5.5 or abs(x+.25*v) >= 5.7:
                            raise ValueError('unsafe position or approach to rail end at startup')
                        break
            time.sleep(.003)
        else:
            raise TimeoutError('no fresh position and velocity at startup')

        seq = board.set_pwm_ticks(neutral)
        after = now()
        end = after+.25
        while remote_after(after, end)['command_seq'] != seq:
            pass
        board.start_remote()
        started = now()
        law = ReturnLaw(board.confirmed_ticks, neutral, profile['slew_ticks_per_s'], polarity, probe)
        record('armed', reason=profile['firmware'], target_ticks=neutral, output_ticks=neutral)
        sent = {seq: neutral}
        target = output = last_sent = neutral
        last_ack = due = started
        ack_seq = None
        active = False
        waiting = False
        last_valid = received
        while now()-started < seconds:
            if cancelled.is_set():
                raise RuntimeError('vision stopped')
            while True:
                try:
                    row = port.debug.get_nowait()
                except queue.Empty:
                    break
                if row['received'] < started:
                    continue
                output = row['output_pwm_ticks']
                ack = row['command_seq']
                record('rx', seq=ack, board_ms=row['board_timestamp'], target_ticks=row['target_pwm_ticks'],
                       output_ticks=output, reason='result=%d enabled=%d' % (row['result'],row['remote_enabled']))
                if not row['remote_enabled'] or row['result'] != 0:
                    raise RuntimeError('REMOTE stopped or board clamped: result=%s' % row['result'])
                if not board.confirmed_ticks[0] <= output <= board.confirmed_ticks[1]:
                    raise ValueError('output outside confirmed range')
                if ack not in sent or sent[ack] != row['target_pwm_ticks']:
                    raise RuntimeError('unexpected PWM echo')
                if ack != ack_seq:
                    last_ack, ack_seq = row['received'], ack
                active = True
                law.feedback(row['received'], output, row['target_pwm_ticks'])
            if now()-last_ack > .25:
                raise TimeoutError('no advancing PWM echo')
            if now()-last_valid > .22:
                raise TimeoutError('no reliable fresh motion for 220 ms')
            m = port.measurement
            if m[0] != frame:
                frame, t, x, reason, received = m
                fresh = x is not None and 0 <= now()-t <= .1
                if fresh:
                    if abs(x) >= 5.7:
                        raise ValueError('rail end guard')
                    v = motion.add(t, x)
                    if v is not None:
                        last_valid = received
                        waiting = False
                        if active:
                            target, phase, stalled = law.step(t, x, v)
                        else:
                            target, phase, stalled = neutral, 'await_echo', False
                        if phase == 'probe_motion':
                            reason = 'observed_pwm_to_x_sign=%+d' % (probe*sign(v-law.start_v))
                        record('vision', frame=frame, capture_s=t, x_cm=x, target_cm=0, error_cm=-x,
                               velocity_cm_s=v, phase=phase, stalled=int(stalled), valid=1,
                               target_ticks=target, output_ticks=output, reason=reason)
                        if phase == 'probe_motion':
                            break
                    else:
                        waiting = True
                        record('vision', frame=frame,capture_s=t,x_cm=x,valid=1,phase='wait_speed',
                               reason='velocity evidence incomplete')
                else:
                    waiting = True
                    record('vision', frame=frame, capture_s=t, valid=0, phase='pause',
                           reason=('aged_before_decision' if reason=='ok' else reason), output_ticks=output)
            gap = now()-last_valid
            if waiting or gap > .11:
                # Withdraw torque while keeping the session alive. Never advance a ramp on a missing frame.
                target = neutral
                law.reset_observation()
            law.check_actuator(now())
            if now() >= due:
                if target != neutral and not (m[2] is not None and 0 <= now()-m[1] <= .1):
                    target = neutral
                    law.reset_observation()
                # The field limit applies to each actual write, including withdrawal on brief loss.
                step_limit = int(15*1600/180)
                last_sent += clip(target-last_sent, -step_limit, step_limit)
                seq = board.set_pwm_ticks(last_sent)
                sent[seq] = last_sent
                law.written(now(), last_sent, output)
                record('tx', seq=seq, target_ticks=last_sent, output_ticks=output)
                due = now()+.05
            time.sleep(.003)
    except BaseException as exc:
        record('abort', reason=str(exc))
        raise
    finally:
        after = now()
        board.stop()
        end = now()+.45+(board.confirmed_ticks[1]-board.confirmed_ticks[0])/profile['slew_ticks_per_s']
        while True:
            row = remote_after(after, end)
            if not row['remote_enabled'] and row['output_pwm_ticks'] == neutral:
                record('stop_confirmed', output_ticks=neutral, board_ms=row['board_timestamp'])
                break
