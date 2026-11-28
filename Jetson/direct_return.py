"""按新鲜曝光直接选择 PWM。x=0 对应纸尺 12 cm，速度正向为刻度增大。"""
from collections import deque
import json
import math
import os
from pathlib import Path
from statistics import median
from tempfile import NamedTemporaryFile


def sign(x):
    return int(x > 0) - int(x < 0)


def bounded(x, lo, hi):
    return min(hi, max(lo, x))


def region(x):
    """经验只覆盖已定义的纸尺区段，不把区外样本塞进端部区段。"""
    mark = x + 12.
    if not 7 <= mark <= 17:
        return None
    lower = 7 + 2 * min(4, int((mark - 7) // 2))
    return '%d-%d' % (lower, lower + 2)


class DirectReturn:
    """每个被接纳的曝光调用 step 一次。失效时调用 reset_observation()。

    output 为决策时最新回读，output_at_exposure 为曝光时已知的实际输出或 None。
    调用方处理命令续发、视觉超时和停机，舵机限速由板端执行。step 仅积累内存证据；
    调用方须在 STOP 回读确认后调用 save()，包括因失效或异常退出的试验。
    """

    def __init__(self, limits, neutral, polarity, experience_path=None):
        lo, hi = limits
        if (not all(isinstance(t, int) and not isinstance(t, bool) for t in (lo, neutral, hi))
                or not lo < neutral < hi or polarity not in (-1, 1)):
            raise ValueError('invalid PWM limits or polarity')
        self.low, self.high, self.neutral, self.polarity = lo, hi, neutral, polarity
        self.path = Path(experience_path) if experience_path else None
        profile = dict(limits=[lo, hi], neutral=neutral, polarity=polarity)
        self.experience = dict(schema='direct-pwm-v1', profile=profile, regions={})
        if self.path and self.path.exists():
            data = json.loads(self.path.read_text(encoding='utf-8'))
            if isinstance(data, dict) and (data.get('profile') != profile or data.get('schema') != 'direct-pwm-v1'):
                # 另一组装配参数独立存档，不覆盖其学习记录。
                self.path = self.path.with_name('%s.%d-%d-%d-%+d.json' %
                                                (self.path.stem,lo,neutral,hi,polarity))
                data = json.loads(self.path.read_text()) if self.path.exists() else self.experience
            if (not isinstance(data, dict) or data.get('schema') != 'direct-pwm-v1'
                    or data.get('profile') != profile or not isinstance(data.get('regions'), dict)):
                raise ValueError('incompatible direct PWM experience')
            for key, records in data['regions'].items():
                if key not in ('7-9','9-11','11-13','13-15','15-17') or not isinstance(records, list):
                    raise ValueError('invalid PWM experience region')
                for rec in records:
                    if (not isinstance(rec, dict) or rec.get('kind') not in ('start','brake','balance')
                            or rec.get('direction') not in (-1,0,1)
                            or not lo <= rec.get('output_ticks', lo-1) <= hi
                            or not all(isinstance(v,(int,float)) and math.isfinite(v)
                                       for k,v in rec.items() if k != 'kind')):
                        raise ValueError('invalid PWM experience sample')
                    if rec['kind'] == 'brake' and not (
                            rec.get('entry_speed_cm_s',0) > 0 and rec.get('stop_distance_cm',-1) >= 0):
                        raise ValueError('invalid brake evidence')
            self.experience = data
        self.dirty = False
        self.balance = float(neutral)
        saved = self._records(0., 'balance', 0)
        if saved:
            self.balance = float(median(r['output_ticks'] for r in saved[-3:]))
        self.target = round(self.balance)
        self.phase = None
        self.drive_dir = 0
        self.level = 10.
        self.last_stamp = None
        self.reset_observation()

    def reset_observation(self):
        """断开实测位移、平台到位计时和目标带计时；保留经验及已累积的起动级别。"""
        self.start_anchor = self.brake_anchor = self.dwell = None
        self.endpoint = self.hold_start = self.hold_anchor = None
        self.brake_dir = 0
        self.brake_command = self.pending_stop = None
        self.previous_x = self.previous_v = None
        self.recent_positions = deque()
        self.move_target = float(self.target)
        self.move_watch = None
        if self.phase != 'breakaway' and self.phase is not None:
            self.drive_dir = 0
        self.phase = None

    def interrupt_hold(self):
        """An explicitly invalid frame interrupts proof of continuous measured holding."""
        self.hold_start = self.hold_anchor = None

    def _records(self, x, kind, direction):
        return [r for r in self.experience['regions'].get(region(x), [])
                if r['kind'] == kind and r['direction'] == direction]

    def _remember(self, kind, direction, start_x, end_x, tick, **effect):
        key = region(start_x)
        if key is None:
            return
        records = self.experience['regions'].setdefault(key, [])
        records.append(dict(kind=kind, direction=direction, start_x_cm=start_x, end_x_cm=end_x,
                            output_ticks=tick, **effect))
        self.dirty = True

    def save(self):
        """只在外部确认 STOP 后调用；试验中不进行文件写入或 fsync。"""
        if not self.path or not self.dirty:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        name = None
        try:
            with NamedTemporaryFile('w', dir=self.path.parent, encoding='utf-8', delete=False) as f:
                name = f.name
                json.dump(self.experience, f, ensure_ascii=False, indent=2, allow_nan=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(name, self.path)
            name = None
            self.dirty = False
        finally:
            if name is not None:
                os.unlink(name)

    def _ticks(self, direction, level):
        return round(bounded(self.balance + self.polarity * direction * level, self.low, self.high))

    def _start(self, x, v, t, direction, output, echo):
        if self.drive_dir != direction or self.phase in ('move','hold','brake'):
            self.drive_dir = direction
            past = self._records(x, 'start', direction)
            # 从成功回读下方一档开始复探，不将一次成功值当作机械阈值。
            self.level = 10.
            if self.phase == 'move':
                self.level = max(self.level, (output-self.balance)*self.polarity*direction)
            if past:
                offsets = [self.polarity*direction*(r['output_ticks']-self.balance) for r in past[-3:]]
                self.level = max(self.level, median(offsets)-8.)
            self.start_anchor = self.dwell = None
        target = self._ticks(direction, self.level)
        if echo is None or (echo-self.balance)*direction*self.polarity <= 0:
            self.start_anchor = self.dwell = None
        else:
            # 不同实际 PWM 平台各有独立的位移零点，禁止将前一平台的进展归到新回读。
            if self.start_anchor is None or self.start_anchor[2] != echo:
                if abs(v) >= .30:
                    # 观测恢复时继承曝光回读；缺测期间的运动不能作为起滚经验。
                    self.dwell = None
                    self.move_target = float(echo)
                    return round(echo), 'move', 'rolling without start evidence'
                self.start_anchor = (t, x, echo)
            start, origin, _ = self.start_anchor
            if (t-start >= .10 and direction*(x-origin) >= .12 and direction*v > .25):
                self._remember('start', direction, origin, x, echo, elapsed_s=t-start,
                               displacement_cm=x-origin)
                self.start_anchor = self.dwell = None
                self.move_target = float(echo)
                return round(echo), 'move', 'measured directional travel'
            if direction*(x-origin) < -.08:
                self.start_anchor = (t,x,echo)
            # 只有当前回读和曝光回读均到达指令才开始平台观察，掉帧后重新计时。
            if abs(output-target) <= 1 and abs(echo-target) <= 1 and abs(v) < .3:
                if self.dwell is None or abs(x-self.dwell[1]) > .06:
                    self.dwell = (t,x)
                elif t-self.dwell[0] >= .35:
                    self.level = min(self.high-self.low, self.level+8.)
                    target = self._ticks(direction,self.level)
                    self.dwell = None
            else:
                self.dwell = None
        return target, 'breakaway', 'observe applied start level'

    def _move(self, direction, raw, v, stamp, distance, brake_distance, output, echo):
        # 从真正带动钢珠的 PWM 开始。每次只在同一实际平台的观察窗结束后比较速度与位移，
        # 修正幅度随剩余距离、速度变化及当前偏转而变。下一级由板端实施机械限速。
        if self.phase != 'move':
            self.move_target = float(output if echo is None else echo)
            self.move_watch = None
        applied = round(self.move_target)
        closing = direction*v
        if echo != applied or output != applied:
            self.move_watch = None
        elif self.move_watch is None:
            self.move_watch = (stamp,raw,closing)
        else:
            start, origin, entry_speed = self.move_watch
            duration = stamp-start
            if duration >= .25:
                travelled = direction*(raw-origin)
                mean_speed = travelled/duration
                remaining = max(.01,distance-.30)
                effort = self.polarity*direction*(applied-self.balance)
                growth = (closing-entry_speed)/max(.4,abs(entry_speed))
                proximity = max(0., (2.*brake_distance-remaining)/remaining)
                slow = max(0., (.40-min(mean_speed,closing))/.40)
                change = 0.
                if (proximity > 0 and closing > .25) or growth > .15:
                    relief = max(proximity, growth, 0.)
                    change = -max(0.,effort)*relief/(1.+relief)
                elif slow > 0 or growth < -.15:
                    # 近停时按实际剩余距离和原驱动力补推，定速且有足够进展时保留原偏转。
                    gain = max(slow,-growth)
                    change = (1.+abs(effort))*.35*gain*remaining/(remaining+brake_distance)
                self.move_target = bounded(self.move_target+self.polarity*direction*change,self.low,self.high)
                self.move_watch = None
        return round(self.move_target), 'move', 'rolling response on applied PWM plateau'

    def _hold(self, raw, v, stamp, dt, output, echo):
        # 小位移直接用 PWM 域 PD；积分偏置只在实测低速、指令已执行时缓慢更新。
        # 输出行程和同向饱和防积分累加构成边界。保持时没有额外的人为角度上限。
        pd = -self.polarity*(8.*raw+4.*v)
        trim = -self.polarity*raw*dt
        tracking = echo is not None and abs(output-self.target)<=1 and abs(echo-self.target)<=1
        proposed = self.balance+pd+trim
        winding_up = (proposed>self.high and trim>0) or (proposed<self.low and trim<0)
        if self.hold_start is None:
            self.hold_start = stamp
        if tracking and not winding_up:
            self.balance = bounded(self.balance+trim,self.low,self.high)
        target = round(bounded(self.balance+pd,self.low,self.high))
        if tracking and echo == target and abs(raw)<=.05:
            if self.hold_anchor is None or echo != self.hold_anchor[2] or abs(raw-self.hold_anchor[1])>.025:
                self.hold_anchor = (stamp,raw,echo)
            age = stamp-self.hold_anchor[0]
            if age >= 1.:
                saved = self._records(raw,'balance',0)
                if echo != self.neutral and (not saved or saved[-1]['output_ticks'] != echo):
                    self._remember('balance',0,self.hold_anchor[1],raw,echo,elapsed_s=age)
                self.hold_anchor = (stamp,raw,echo)
        else:
            self.hold_anchor = None
        return target,'hold','PWM position and velocity feedback with slow balance trim'

    def _brake(self, x, v, t, motion, output, echo):
        if self.brake_dir != motion:
            self.brake_dir = motion
            self.brake_command = (x,t,abs(v))
            self.brake_anchor = self.dwell = None
            prior = self._records(x, 'brake', motion)
            self.brake_level = 10.
            if prior:
                self.brake_level = max(4., median(-self.polarity*motion*(r['output_ticks']-self.balance)
                                                 for r in prior[-3:]))
        target = self._ticks(-motion,self.brake_level)
        applied = echo is not None and (echo-self.balance)*motion*self.polarity < 0
        if not applied:
            self.brake_anchor = self.dwell = None
        else:
            if self.brake_anchor is None and motion*v >= .30:
                self.brake_anchor = (x,t,abs(v))
            if abs(echo-target)<=1 and abs(output-target)<=1 and motion*v > .30:
                if self.dwell is None:
                    self.dwell = (t,motion*v)
                elif t-self.dwell[0] >= .35:
                    if motion*v >= self.dwell[1]-.10:
                        self.brake_level += 4.
                        target = self._ticks(-motion,self.brake_level)
                    self.dwell = None
            else:
                self.dwell = None
        return target, 'brake', 'oppose observed rolling direction'

    def _finish_brake(self, x, v, t, echo):
        if self.brake_anchor is not None and echo is not None:
            origin,start,speed = self.brake_anchor
            travel = self.brake_dir*(x-origin)
            if (travel >= .08 and t-start >= .10 and self.brake_dir*v <= .20
                    and (echo-self.balance)*self.brake_dir*self.polarity < 0):
                cmd_x, cmd_t, cmd_speed = self.brake_command
                # 撤去制动后再用下一实际曝光验证近静止或反向位移，避免仅因速度估计瞬时过零学习。
                self.pending_stop = dict(direction=self.brake_dir, start=cmd_x, stamp=t,
                                         end=x, tick=echo, elapsed=t-cmd_t, speed=cmd_speed,
                                         applied_x=origin, applied_speed=speed)
        self.brake_anchor = self.brake_command = self.dwell = None
        self.brake_dir = 0

    def _confirm_stop(self, x, v, t):
        p = self.pending_stop
        if p is None:
            return
        self.pending_stop = None
        d = p['direction']
        travel = d*(p['end']-p['start'])
        continued_speed = d*(x-p['end'])/(t-p['stamp'])
        if continued_speed <= .28 and d*v <= .28 and travel > 0:
            self._remember('brake',d,p['start'],p['end'],p['tick'],elapsed_s=p['elapsed'],
                           entry_speed_cm_s=p['speed'],stop_distance_cm=travel,
                           applied_stop_distance_cm=d*(p['end']-p['applied_x']),
                           applied_entry_speed_cm_s=p['applied_speed'])


    def step(self, x, v, stamp, observed_x, output, output_at_exposure):
        echo = output_at_exposure
        if (not all(math.isfinite(z) for z in (x,v,stamp,observed_x,output))
                or not self.low <= output <= self.high
                or (echo is not None and not self.low <= echo <= self.high)
                or (self.last_stamp is not None and stamp <= self.last_stamp)):
            raise ValueError('invalid direct PWM observation')
        # 缺测由调用方统一 reset_observation。恢复后的第一次曝光不能积入丢失的时间。
        dt = stamp-self.last_stamp if self.previous_x is not None else 0.
        self.last_stamp = stamp
        raw = observed_x
        direction = -sign(raw)
        speed = abs(v)
        motion = sign(v)
        self._confirm_stop(raw,v,stamp)
        if self.brake_dir and self.brake_dir*v <= .20:
            self._finish_brake(raw,v,stamp,echo)

        # 同向成功制动的停止距离优先，仅复用相近速度。未知速度段低速移送，
        # 延迟预判和软制动区只参与决策；预测位置从不写入学习记录。
        evidence = [r for r in self._records(raw,'brake',motion)
                    if .65*speed <= r['entry_speed_cm_s'] <= 1.5*speed]
        self.recent_positions.append((stamp,raw))
        while len(self.recent_positions)>1 and stamp-self.recent_positions[0][0] > .35:
            self.recent_positions.popleft()
        recent_travel = abs(raw-self.recent_positions[0][1])
        brake_distance = max((r['stop_distance_cm'] for r in evidence),
                             default=max(recent_travel,.35*speed))
        near = abs(x) <= .30+brake_distance or x*raw <= 0
        need_brake = speed > .30 and (near or motion != direction)
        holding = abs(raw) <= .30 and abs(x) <= .30 and speed <= .30
        if self.brake_dir or need_brake:
            target, phase, reason = self._brake(raw,v,stamp,self.brake_dir or motion,output,echo)
        elif holding:
            target,phase,reason = self._hold(raw,v,stamp,dt,output,echo)
        elif abs(raw) <= .30 and speed <= .30:
            target, phase, reason = round(self.balance), 'move', 'ease on measured target band'
        elif (self.phase == 'move' and direction*v > .06) or (self.phase != 'breakaway' and direction*v >= .30):
            target,phase,reason = self._move(direction,raw,v,stamp,abs(x),brake_distance,output,echo)
        else:
            target,phase,reason = self._start(raw,v,stamp,direction or -motion,output,echo)

        if phase != 'move' or self.phase != 'move':
            self.move_watch = None
        if phase != 'breakaway':
            self.start_anchor = None
        if phase != 'hold':
            self.hold_anchor = self.hold_start = None
        # 上层提供已确认的机械行程。限位故障要求连续真曝光、当前及曝光时双回读到端，
        # 且期间实测位置无可分辨变化。计算出的饱和指令本身不能证明机构堵转。
        if target in (self.low,self.high) and output == echo == target and speed < .25 and phase in ('breakaway','move'):
            if self.endpoint is None or self.endpoint[0] != target or abs(raw-self.endpoint[2]) >= .06:
                self.endpoint = (target,stamp,raw)
            elif stamp-self.endpoint[1] >= .35:
                raise ValueError('mechanical endpoint without measured motion')
        else:
            self.endpoint = None
        self.previous_x, self.previous_v, self.target, self.phase = raw,v,target,phase
        hold = 0. if self.hold_start is None else stamp-self.hold_start
        return dict(target_ticks=target,phase=phase,reason=reason,balance_ticks=self.balance,
                    start_bias=target-self.balance if phase=='breakaway' else 0.,
                    hold_s=hold,hold_complete=hold>=10.)
