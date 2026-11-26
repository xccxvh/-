"""Adaptive REMOTE control from current SOE-timed top-view measurements."""
import csv
from collections import deque
import math
import queue
import time

from host_experiment import NEUTRAL


def sign(x):
    return (x > 0) - (x < 0)


class ReturnLaw:
    def __init__(self, bounds):
        self.bounds = bounds
        self.target = NEUTRAL
        self.polarity = 1
        self.verified = False
        self.reversed = False
        self.anchor = None
        self.ready_since = None
        self.holding = False
        self.window = deque()

    def step(self, stamp, x, applied):
        self.window.append((stamp, x))
        while len(self.window) > 1 and stamp-self.window[0][0] > .28:
            self.window.popleft()
        t0, x0 = self.window[0]
        dt = stamp-t0
        v = (x-x0)/dt if dt >= .10 else 0.
        if abs(x) >= 5.7 or abs(x + .15*v) >= 5.9:
            raise ValueError('rail end guard')
        if self.anchor is None:
            self.anchor = x
        if applied != self.target:
            self.ready_since = None
        elif self.ready_since is None:
            self.ready_since = stamp
            self.anchor = x
        dwell = 0 if self.ready_since is None else stamp-self.ready_since
        err = -x
        direction = sign(err)
        toward = direction*v
        displacement = direction*(x-self.anchor)
        stalled = abs(v)<.35 and abs(x-self.anchor)<.09
        previous = self.target
        direction_pwm = direction*self.polarity
        drive = self.target-NEUTRAL
        # Changes use the measured output plateau, preventing an increment on every image.
        if abs(x) < (.30 if self.holding else .18) and abs(v)<.55:
            self.holding = True
            desired, phase = NEUTRAL, 'hold'
        elif toward > .65 and abs(x) <= .15 + .20*toward + toward*toward/16.:
            self.holding = False
            desired = NEUTRAL - direction_pwm*min(12, max(3,round(2*toward)))
            phase = 'brake'
        else:
            self.holding = False
            desired, phase = self.target, 'settle'
            if drive*direction_pwm <= 0 and dwell >= .20:
                desired, phase = NEUTRAL+3*direction_pwm, 'probe'
            elif drive*direction_pwm > 0:
                if dwell >= .20 and toward > .40 and displacement > .12:
                    self.verified = True
                if dwell >= .35 and displacement < -.18 and toward < -.45:
                    if self.verified or self.reversed:
                        raise ValueError('sustained movement away from target')
                    self.polarity *= -1
                    self.reversed = True
                    desired, phase = NEUTRAL, 'reverse_probe'
                elif toward > 1.2 and dwell >= .20:
                    desired, phase = NEUTRAL + direction_pwm*max(0, abs(drive)-3), 'ease'
                elif dwell >= .65 and (stalled or (0 <= toward < .65)):
                    desired = self.target+3*direction_pwm
                    phase = 'increase_stalled' if stalled else 'increase_slow'
                    if not self.bounds[0] <= desired <= self.bounds[1]:
                        limit = self.bounds[1] if direction_pwm > 0 else self.bounds[0]
                        if self.target == limit and stalled:
                            raise ValueError('PWM limit reached without rolling')
                        desired = limit
        desired = max(self.bounds[0],min(self.bounds[1],desired))
        self.target = previous + max(-3,min(3,desired-previous))
        if self.target != previous:
            self.ready_since = None
        return self.target, v, phase, stalled


def run(board, port, stream, seconds, cancelled):
    now = time.monotonic
    law = ReturnLaw(board.confirmed_ticks)
    fields = ('event','nano_s','frame','capture_s','x_cm','target_cm','error_cm','velocity_cm_s',
              'valid','reason','phase','stalled','target_ticks','output_ticks','seq','board_ms')
    log = csv.DictWriter(stream,fields)
    log.writeheader()
    def record(event, **values):
        log.writerow(dict(values,event=event,nano_s=now()))
    def remote_after(t, end):
        while now()<end:
            row = port.next_debug(end)
            if row['received'] >= t:
                return row
        raise TimeoutError('remote telemetry missing')

    try:
        board.set_mode('REMOTE')
        stamp=now()
        row=remote_after(stamp,stamp+.5)
        if row['remote_enabled'] or row['output_pwm_ticks'] != NEUTRAL:
            raise ValueError('stopped neutral not confirmed')
        board.seq=(row['command_seq']+1)&65535
        # Require a stationary observed start; initialization consumes no drive pulses.
        samples=deque()
        frame=None
        end=now()+4.
        while now()<end:
            if cancelled.is_set():
                raise RuntimeError('vision stopped')
            m=getattr(port,'measurement',None)
            if m and m[0]!=frame:
                frame, t, x, _, received=m
                if x is None or not 0 <= now()-t <= .1:
                    samples.clear()
                else:
                    samples.append((t,x))
                    while samples and t-samples[0][0]>.45:
                        samples.popleft()
                    if t-samples[0][0]>=.25 and max(p[1] for p in samples)-min(p[1] for p in samples)<.10:
                        if abs(x)>=5.5:
                            raise ValueError('start too close to rail end')
                        break
            time.sleep(.005)
        else:
            raise TimeoutError('no fresh stationary ball at startup')

        seq=board.set_pwm_ticks(NEUTRAL)
        stamp=now()
        end=stamp+.25
        while remote_after(stamp,end)['command_seq']!=seq:
            pass
        board.start_remote()
        started=now()
        sent={seq:NEUTRAL}
        output=NEUTRAL
        last_ack=started
        ack_seq=None
        active=False
        target=NEUTRAL
        due=started
        last_valid=received
        last_measured_target=NEUTRAL
        target_since=started
        while now()-started<seconds:
            if cancelled.is_set():
                raise RuntimeError('vision stopped')
            while True:
                try:
                    row=port.debug.get_nowait()
                except queue.Empty:
                    break
                if row['received']<started:
                    continue
                output=row['output_pwm_ticks']
                if not row['remote_enabled'] or row['result']!=0:
                    raise RuntimeError('REMOTE exited result=%s'%row['result'])
                if not board.confirmed_ticks[0]<=output<=board.confirmed_ticks[1]:
                    raise ValueError('output outside confirmed range')
                ack=row['command_seq']
                if ack not in sent or sent[ack]!=row['target_pwm_ticks']:
                    raise RuntimeError('unexpected PWM echo')
                if ack!=ack_seq:
                    last_ack=row['received']
                    ack_seq=ack
                active=True
                record('rx',seq=ack,board_ms=row['board_timestamp'],target_ticks=row['target_pwm_ticks'],output_ticks=output)
            if now()-last_ack>.25:
                raise TimeoutError('no advancing PWM echo')
            m=port.measurement
            if m[0]!=frame:
                frame,t,x,reason,received=m
                fresh=x is not None and 0<=now()-t<=.1
                if fresh:
                    last_valid=received
                    if active:
                        target,v,phase,stalled=law.step(t,x,output)
                    else:
                        v,phase,stalled=0.,'await_echo',False
                    record('vision',frame=frame,capture_s=t,x_cm=x,target_cm=0,error_cm=-x,
                           velocity_cm_s=v,phase=phase,stalled=int(stalled),valid=1,
                           target_ticks=target,output_ticks=output,reason=reason)
                else:
                    record('vision',frame=frame,capture_s=t,valid=0,reason=('aged_before_decision' if reason=='ok' else reason),output_ticks=output)
                    if reason not in ('ok', 'stale_only_drop'):
                        raise RuntimeError('invalid current image: '+reason)
            if now()-last_valid>.15:
                raise TimeoutError('vision gap over 150 ms')
            if last_measured_target != target or target==output:
                target_since=now()
                last_measured_target=target
            if now()-target_since>.55:
                raise RuntimeError('PWM output failed to reach target')
            if now()>=due and (target==NEUTRAL or (m[2] is not None and 0 <= now()-m[1] <= .1)):
                seq=board.set_pwm_ticks(target)
                sent[seq]=target
                record('tx',seq=seq,target_ticks=target,output_ticks=output)
                due=now()+.05
            time.sleep(.003)
    except BaseException as exc:
        record('abort',reason=str(exc))
        raise
    finally:
        stamp=now()
        board.stop()
        end=now()+1.
        while True:
            row=remote_after(stamp,end)
            if not row['remote_enabled'] and row['output_pwm_ticks']==NEUTRAL:
                record('stop_confirmed',output_ticks=NEUTRAL,board_ms=row['board_timestamp'])
                break
