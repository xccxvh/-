"""Nano 连续回中。曝光时钟更新估计，单调时钟调度 50 Hz REMOTE PWM。"""
from collections import deque
import csv
import math
import queue
import time

from velocity_return import AlphaBeta, VelocityPlan, FRESH_S, LOST_S

PERIOD = .020


def target_echo_known(sent, ack, applied_target):
    # Firmware a2edda1 reports the last received sequence and the target applied
    # on its 10 ms tick. At a 50 Hz host update either adjacent target can appear.
    return ack in sent and applied_target in (sent[ack], sent.get((ack - 1) & 65535))


def exposure_output(echoes, changes, stamp):
    """Last confirmed settled output, provided no newer target preceded the exposure."""
    last_change = next(((t, pwm) for t, pwm in reversed(changes) if t <= stamp), None)
    for received, target, output in reversed(echoes):
        if received <= stamp:
            if last_change and last_change[0] <= received and last_change[1] == target == output:
                return output
            break
    return None


def run(board, port, stream, seconds, cancelled, profile, probe=None, polarity=-1,
        offset=None, model=None, platforms=None, brake_ticks=None, experience_path=None, timed_route=False):
    now = time.monotonic
    neutral = profile['neutral_ticks']
    lo, hi = board.confirmed_ticks
    slew = profile['slew_ticks_per_s']
    if probe is not None and offset is None:
        offset = round((hi-neutral if probe > 0 else lo-neutral)*.5)
    if platforms:
        sides = [(t-neutral) for t in platforms]
        if (any(not lo <= t <= hi for t in platforms) or sides[0] == 0
                or any(b*a <= 0 or abs(b) <= abs(a) for a,b in zip(sides,sides[1:]))):
            raise ValueError('identification needs increasing magnitudes on one confirmed side')
        offset = sides[0]
    if brake_ticks is not None and (offset is None or (brake_ticks-neutral)*offset>=0 or not lo<=brake_ticks<=hi):
        raise ValueError('brake must be on the opposite confirmed side')
    if offset is not None and (offset == 0 or not lo <= neutral+offset <= hi):
        raise ValueError('identification pulse outside confirmed stroke')
    state = AlphaBeta()
    direct = legacy = None
    route = None
    if timed_route:
        from timed_route import TimedRoute
        route = TimedRoute(lo, neutral, hi)
    elif offset is None:
        if model is None:
            from direct_return import DirectReturn
            direct = DirectReturn((lo, hi), neutral, polarity, experience_path)
        else:
            legacy = VelocityPlan((lo, hi), neutral, polarity, model)

    fields = ('event','nano_s','frame','capture_s','received_s','last_capture_s',
              'last_received_s','x_cm','valid','estimate_cm','predicted_cm','velocity_cm_s',
              'target_cm','error_cm','v_ref','a_cmd','start_bias','balance_ticks','phase',
              'target_ticks','output_ticks','seq','board_ms','reason','breakaway_pos_ticks','breakaway_neg_ticks','exposure_age_ms','output_at_exposure','hold_s')
    writer = csv.DictWriter(stream, fields)
    writer.writeheader()
    last_received = frame = measured_x = None

    def record(event, **row):
        row.update(event=event, nano_s=now(), last_capture_s=state.stamp, last_received_s=last_received)
        writer.writerow(row)

    def observe():
        nonlocal frame, last_received, measured_x
        m = getattr(port, 'measurement', None)
        if not m or m[0] == frame:
            return None
        frame, stamp, x, reason, received = m
        decision = now()
        # Only the control loop admits a REMOTE measurement, on the SOE clock.
        accepted = False
        if x is not None and 0 <= decision-stamp <= FRESH_S:
            if abs(x) >= 5.7:
                raise ValueError('measured rail end')
            if direct and state.stamp is not None and stamp - state.stamp > FRESH_S:
                direct.reset_observation()
            if route and state.stamp is not None and stamp-state.stamp > FRESH_S:
                route.interrupt_hold()
            accepted = state.update(stamp, x)
            if not accepted:
                reason = 'duplicate or implausible innovation'
        elif x is not None:
            reason = 'aged_before_admission'
        if accepted:
            last_received = received
            measured_x = x
        elif direct:
            direct.interrupt_hold()
        elif route:
            route.interrupt_hold()
        record('vision', frame=frame,capture_s=stamp,received_s=received,
               **dict(x_cm=x,valid=int(accepted),reason=reason,estimate_cm=state.x,
                      velocity_cm_s=state.v,phase='update' if accepted else 'pause'))
        return accepted

    def debug_after(start, end):
        while True:
            row = port.next_debug(end)
            if row['received'] >= start:
                return row

    try:
        board.set_mode('REMOTE')
        start = now()
        while True:
            row = debug_after(start,start+1.)
            if not row['remote_enabled'] and row['target_pwm_ticks'] == row['output_pwm_ticks'] == neutral:
                break
        board.seq = (row['command_seq']+1) & 65535
        deadline = now()+4.
        while now() < deadline:
            if cancelled.is_set():
                raise RuntimeError('vision stopped')
            if observe() and (offset is None or (state.count >= 3 and abs(state.v) < .35)):
                break
            time.sleep(.002)
        else:
            raise TimeoutError('no reliable starting position')
        if route and abs(measured_x) > .30:
            raise ValueError("place ball at O before starting timed route")
        seq = board.set_pwm_ticks(neutral)
        start = now()
        while debug_after(start,start+.25)['command_seq'] != seq:
            pass
        route_start = now()
        board.start_remote()
        start = due = last_ack = now()
        record('armed', target_ticks=neutral, output_ticks=neutral,reason=profile['firmware'])
        sent = {seq:neutral}
        output = command = neutral
        ack_seq = None
        active = blocked = updated = False
        plan = {}
        planned_stamp = state.stamp
        pulse_origin = state.x
        brake_start = at_level = None
        stage_index = 0
        output_progress = now()
        last_output = neutral
        output_history = deque(maxlen=24)
        target_changes = deque([(start, neutral)], maxlen=24)
        trusted_output = None
        holding_confirmed = False
        while seconds is None or now()-start < seconds:
            if cancelled.is_set():
                raise RuntimeError('vision stopped')
            while True:
                try:
                    status = port.status.get_nowait()
                except queue.Empty:
                    break
                if status['received'] >= start and (status['board_fault'] or status['board_auto_enabled']):
                    raise RuntimeError('board fault or internal AUTO active in REMOTE')
            while True:
                try:
                    row = port.debug.get_nowait()
                except queue.Empty:
                    break
                if row['received'] < start:
                    continue
                ack = row['command_seq']
                output = row['output_pwm_ticks']
                echoed = row['target_pwm_ticks']
                record('rx', seq=ack, board_ms=row['board_timestamp'], target_ticks=echoed,
                       output_ticks=output,reason='enabled=%d result=%d' % (row['remote_enabled'],row['result']))
                if not row['remote_enabled'] or row['result'] or not lo <= output <= hi or not target_echo_known(sent, ack, echoed):
                    raise RuntimeError('unexpected REMOTE state, clamp or PWM echo')
                if ack != ack_seq:
                    ack_seq,last_ack = ack,row['received']
                if output == echoed or output != last_output:
                    output_progress = row['received']
                elif row['received']-output_progress > .25 + abs(output-echoed)/slew:
                    raise TimeoutError('PWM output not following commands')
                output_history.append((row['received'],echoed,output))
                last_output = output
                active = True
            if now() - state.stamp > LOST_S:
                raise TimeoutError('no trusted exposure for 300 ms')
            accepted = observe()
            if accepted is not None:
                blocked = not accepted
                updated = accepted
                if accepted:
                    trusted_output = exposure_output(output_history, target_changes, state.stamp)
            tick = now()
            if tick-last_ack > .25:
                raise TimeoutError('no advancing PWM echo')
            if route and tick-route_start >= 5.:
                # Extend only an already-observed settling interval to confirm stability.
                if route.hold_start is None or route.hold_start > route_start+5.:
                    record('route_deadline', target_cm=route.goal, reason='no settled endpoint by five seconds')
                    break
            if tick < due:
                time.sleep(min(.002,due-tick))
                continue
            x,v = state.predict(tick)
            if max(abs(x),abs(x+.15*v)) >= 5.85:
                raise ValueError('predicted rail end')
            requested = command
            phase = 'hold_command'
            if active and updated and not blocked and tick - state.stamp <= FRESH_S:
                dt = state.stamp-planned_stamp
                planned_stamp = state.stamp
                if offset is None:
                    if route:
                        plan = route.step(x, v, state.stamp, measured_x)
                        route.check_endpoint(plan['target_ticks'], output, trusted_output, state.stamp, measured_x, v)
                        if route.first_hit == state.stamp:
                            record('route_reverse', capture_s=state.stamp, x_cm=measured_x, target_cm=5.,
                                   reason='elapsed %.4f s' % (state.stamp-route_start))
                        if plan['hold_s'] >= 1. and route.hold_start <= route_start+5.:
                            record('route_settled', capture_s=state.stamp,x_cm=measured_x,target_cm=-5.,
                                   hold_s=plan['hold_s'],reason='settling began %.4f s' % (route.hold_start-route_start))
                            break
                    elif direct:
                        plan = direct.step(x, v, state.stamp, measured_x, output, trusted_output)
                        if plan['hold_complete'] and not holding_confirmed:
                            record('hold_confirmed', capture_s=state.stamp, x_cm=measured_x,
                                   target_cm=0., output_ticks=trusted_output, hold_s=plan['hold_s'])
                        holding_confirmed = plan['hold_complete']
                    else:
                        plan = legacy.step(x, v, dt, output, observed_x=state.x)
                    requested = plan['target_ticks']
                    phase = plan['phase']
                else:
                    # 辨识时使用恒定目标，回读到位后计时。观测到起滚后反向施加制动。
                    travel = (state.x-pulse_origin)*polarity*(1 if offset>0 else -1)
                    if brake_start is None and travel < -.40:
                        raise ValueError('identification moved opposite expected direction')
                    if brake_start is None and travel > .28 and v*offset*polarity > 0:
                        brake_start = tick
                        record('roll_onset',capture_s=state.stamp,estimate_cm=state.x,velocity_cm_s=v,
                               target_ticks=command,output_ticks=trusted_output,reason='signed displacement and velocity')
                    if brake_start is None:
                        requested,phase = neutral+offset,'identify_drive'
                        if output == requested:
                            if at_level is None:
                                at_level = tick
                            if tick-at_level > 2.5 and abs(v)<.3 and abs(travel)<.12:
                                last = not platforms or stage_index+1 == len(platforms)
                                record('no_motion' if last else 'plateau_no_motion', output_ticks=output,
                                       reason='stationary on confirmed plateau')
                                if last:
                                    break
                                stage_index += 1
                                requested = platforms[stage_index]
                                offset = requested-neutral
                                at_level = None
                        else:
                            at_level = None
                    else:
                        requested = brake_ticks if brake_ticks is not None else max(lo,min(hi, neutral-offset))
                        phase = 'identify_brake'
                        if v*offset*polarity <= 0 or tick-brake_start > .7:
                            record('brake_end',capture_s=state.stamp,estimate_cm=state.x,velocity_cm_s=v,
                                   output_ticks=output,reason='observed sign reversal' if v*offset*polarity<=0 else 'brake deadline')
                            break
            elif tick - state.stamp > FRESH_S:
                phase = 'hold_missing'
            # Only the board slews the actuator. A missed exposure preserves the
            # most recent command until observation resumes or the SOE timeout.
            command = int(round(max(lo, min(hi, requested))))
            if target_changes[-1][1] != command:
                target_changes.append((now(), command))
            seq = board.set_pwm_ticks(command)
            sent[seq] = command
            record('tx',seq=seq,target_ticks=command,output_ticks=output,predicted_cm=x,
                   velocity_cm_s=v,target_cm=route.goal if route else 0.,error_cm=(route.goal if route else 0.)-x,phase=phase,
                   exposure_age_ms=(tick-state.stamp)*1000,
                   output_at_exposure=trusted_output if updated else None,
                   reason=plan.get('reason') if phase not in ('hold_command','hold_missing') else phase,
                   hold_s=plan.get('hold_s') if phase in ('hold','endpoint_hold') else None,
                   **{k:plan[k] for k in ('v_ref','a_cmd','start_bias','balance_ticks') if k in plan})
            updated = False
            # 按绝对期限调度。操作系统延误时跳过已错过周期，避免补发成串旧指令。
            due += PERIOD*max(1,math.floor((now()-due)/PERIOD)+1)
    except BaseException as exc:
        record('abort',reason=str(exc))
        raise
    finally:
        start = now()
        record('stop_requested')
        board.stop()
        end = now()+.45+(hi-lo)/slew
        while True:
            row = debug_after(start,end)
            if not row['remote_enabled'] and row['output_pwm_ticks'] == neutral:
                record('stop_confirmed',output_ticks=neutral,board_ms=row['board_timestamp'])
                if direct:
                    direct.save()
                break
