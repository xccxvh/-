"""Nano 连续回中。曝光时钟更新估计，单调时钟调度 50 Hz REMOTE PWM。"""
from collections import deque
import csv
import math
import queue
import time

from velocity_return import AlphaBeta, VelocityPlan

PERIOD = .020
LOSS = .220


def run(board, port, stream, seconds, cancelled, profile, probe=None, polarity=-1,
        offset=None, model=None, platforms=None, brake_ticks=None):
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
    law = VelocityPlan((lo, hi), neutral, polarity, model) if offset is None else None
    fields = ('event','nano_s','frame','capture_s','received_s','last_capture_s',
              'last_received_s','x_cm','valid','estimate_cm','predicted_cm','velocity_cm_s',
              'target_cm','error_cm','v_ref','a_cmd','start_bias','balance_ticks','phase',
              'target_ticks','output_ticks','seq','board_ms','reason','breakaway_pos_ticks','breakaway_neg_ticks')
    writer = csv.DictWriter(stream, fields)
    writer.writeheader()
    last_received = frame = None

    def record(event, **row):
        row.update(event=event, nano_s=now(), last_capture_s=state.stamp, last_received_s=last_received)
        writer.writerow(row)

    def observe():
        nonlocal frame, last_received
        m = getattr(port, 'measurement', None)
        if not m or m[0] == frame:
            return None
        frame, stamp, x, reason, received = m
        decision = now()
        # 每个帧号仅接纳一次；即使排队延迟，摄像头 SOE 到使用时仍须在 100 ms 内。
        accepted = False
        if x is not None and 0 <= decision-stamp <= .1:
            if abs(x) >= 5.7:
                raise ValueError('measured rail end')
            accepted = state.update(stamp, x)
            if not accepted:
                reason = 'duplicate or implausible innovation'
        elif x is not None:
            reason = 'aged_before_admission'
        if accepted:
            last_received = received
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
            if (observe() and state.count >= 3 and abs(state.predict(now())[0]) < 5.5
                    and (offset is None or abs(state.v)<.35)):
                break
            time.sleep(.002)
        else:
            raise TimeoutError('no reliable starting position')
        seq = board.set_pwm_ticks(neutral)
        start = now()
        while debug_after(start,start+.25)['command_seq'] != seq:
            pass
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
        trusted_output = neutral
        while now()-start < seconds:
            if cancelled.is_set():
                raise RuntimeError('vision stopped')
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
                if not row['remote_enabled'] or row['result'] or not lo <= output <= hi or sent.get(ack) != echoed:
                    raise RuntimeError('unexpected REMOTE state, clamp or PWM echo')
                if ack != ack_seq:
                    ack_seq,last_ack = ack,row['received']
                if output == echoed or output != last_output:
                    output_progress = row['received']
                elif row['received']-output_progress > .25 + abs(output-echoed)/slew:
                    raise TimeoutError('PWM output not following commands')
                output_history.append((row['received'],output))
                last_output = output
                active = True
            accepted = observe()
            if accepted is not None:
                blocked = not accepted
                updated = accepted
                if accepted:
                    for stamp, ticks in reversed(output_history):
                        if stamp <= state.stamp:
                            trusted_output = ticks
                            break
            tick = now()
            if tick-last_ack > .25:
                raise TimeoutError('no advancing PWM echo')
            if tick-last_received > LOSS:
                # 从最后接纳帧交付时计缺测；该帧的曝光年龄已验证不超过 100 ms。
                raise TimeoutError('no accepted measurement received for 220 ms')
            if tick < due:
                time.sleep(min(.002,due-tick))
                continue
            x,v = state.predict(tick)
            if max(abs(x),abs(x+.15*v)) >= 5.85:
                raise ValueError('predicted rail end')
            requested = command
            phase = 'hold_command'
            if active and updated and not blocked:
                dt = state.stamp-planned_stamp
                planned_stamp = state.stamp
                if offset is None:
                    plan = law.step(x,v,dt,output,observed_x=state.x)
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
            elif active and not blocked and offset is None and plan and abs(v) > .35:
                # 纯预测只可降低原驱动；不把预测写入实测列，也不据此增加反向驱动。
                preview = law.step(x,v,0.,output)
                delta = preview['a_cmd'] - plan['a_cmd']
                desired = plan['target_ticks'] + polarity * delta / law.gains[1 if plan['a_cmd'] >= 0 else -1]
                if (desired-command)*(command-neutral) < 0:
                    requested = max(neutral,desired) if command > neutral else min(neutral,desired)
                    phase = 'ease_prediction'
            if blocked and tick-last_received > .11 and trusted_output != neutral:
                # 多帧缺测且刚向外加力：退至上一张可信曝光时已经回读的同向输出。
                # 只撤销未经视觉验证的增量，保留已起作用的偏置；单帧掉帧仍保持。
                if (trusted_output-neutral)*(command-neutral)>0 and abs(trusted_output-neutral)<abs(command-neutral):
                    requested, phase = trusted_output, 'last_observed_drive'
            # 有效观测之间保持已发量；新目标只在有观测时推进。单次写入不超过现场 15°。
            max_step = min(133,max(1,int(slew*PERIOD)))
            if offset is None and (requested-neutral)*(command-neutral) >= 0 and abs(requested-neutral)>abs(command-neutral):
                # 缓升避免阶跃造成纹理突然位移；制动和撤力仍可使用板端全速。
                max_step = min(max_step, max(1,round((hi-lo)*PERIOD/.7)))
            command = int(round(max(lo,min(hi,command + max(-max_step,min(max_step,requested-command))))))
            seq = board.set_pwm_ticks(command)
            sent[seq] = command
            record('tx',seq=seq,target_ticks=command,output_ticks=output,predicted_cm=x,
                   velocity_cm_s=v,target_cm=0,error_cm=-x,phase=phase,
                   breakaway_pos_ticks=law.breakaway_ticks[1] if law else None,
                   breakaway_neg_ticks=law.breakaway_ticks[-1] if law else None,
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
                break
