"""以厘米、秒和 PWM tick 为单位的回零控制。所有默认参数均为工程先验。"""
import math

FRESH_S = .150
LOST_S = .300


def _clip(value, low, high):
    return min(high, max(low, value))


def _sign(value):
    return (value > 0) - (value < 0)


class AlphaBeta:
    """以曝光时间更新。长间隔后的首个样本重新初始化速度与连续计数。"""

    def __init__(self):
        self.stamp = None
        self.x = self.v = 0.
        self.count = 0
        self.pending = None

    def predict(self, now):
        if not math.isfinite(now):
            raise ValueError('invalid prediction time')
        dt = 0 if self.stamp is None else _clip(now - self.stamp, 0., LOST_S)
        return self.x + self.v * dt, self.v

    def update(self, stamp, x):
        if not all(map(math.isfinite, (stamp, x))):
            return False
        dt = 0 if self.stamp is None else stamp - self.stamp
        if self.stamp is not None and dt <= 0:
            return False
        if self.stamp is None or dt > LOST_S:
            self.x, self.v, self.count = x, 0., 1
        else:
            predicted = self.x + self.v * dt
            residual = x - predicted
            # Confirm an unusual innovation against a second real exposure.
            if abs(residual) > .28 + abs(self.v) * dt:
                if self.pending is None:
                    self.pending = stamp, x
                    return False
                t1, x1 = self.pending
                if stamp <= t1:
                    return False
                along = self.x + (x - self.x) * (t1 - self.stamp) / dt
                if abs(x1 - along) > .28:
                    self.pending = stamp, x
                    return False
                self.v = (x - self.x) / dt
                self.x = x
            else:
                self.x = predicted + .65 * residual
                self.v += .18 * residual / dt
            self.count += 1
        self.pending = None
        self.stamp = stamp
        return True


class VelocityPlan:
    """对每个接收的观测调用一次。dt=0 只预览。堵转抛出 ValueError。

    gain_pos/gain_neg 按所需的 x 加速度方向选择，每 tick 的加速度取幅值。
    polarity 指定增加 PWM 时的 x 方向。start_bias 为带 PWM 符号的 tick。
    target_ticks 保留小数，最终取整由发送层负责。
    """

    def __init__(self, bounds, neutral, polarity=-1, model=None):
        self.low, self.high = bounds
        if (not all(map(math.isfinite, (self.low, neutral, self.high))) or
                not self.low < neutral < self.high or polarity not in (-1, 1)):
            raise ValueError('invalid stroke or polarity')
        model = {} if model is None else model
        # 工程先验，尚未经实物辨识。正负方向可分别覆盖。
        self.gains = {1: abs(model.get('gain_pos', .55)),
                      -1: abs(model.get('gain_neg', .55))}
        self.brake = model.get('brake_accel', 6.)
        self.delay = model.get('delay_s', .12)
        if (not all(math.isfinite(g) and g > 0 for g in
                    (self.gains[1], self.gains[-1], self.brake))
                or not math.isfinite(self.delay) or self.delay < 0):
            raise ValueError('invalid acceleration model')
        self.neutral = self.balance = self.last_target = neutral
        self.polarity = polarity
        self.memory = {-1: 0., 1: 0.}
        # 同向起滚时实际回读的绝对 PWM，独立于控制律复用的附加补偿量。
        self.breakaway_ticks = {-1: None, 1: None}
        self.trial_dir = self.limit_side = 0
        self.trial_x = self.limit_x = self.trial_time = self.limit_time = 0.
        self.extra = 0.

    def _observe_start(self, direction, x, v, dt, output):
        # 仅从近静止状态开始试探，实际输出已到位才增加试探量。
        drive = direction * self.polarity
        applied = drive * (output - self.balance)
        if direction != self.trial_dir or not direction:
            self.trial_dir = 0
            self.trial_time = self.extra = 0.
        if direction and not self.trial_dir and abs(v) < .3:
            self.trial_dir = direction
            self.trial_x = x
        if not self.trial_dir:
            return
        if applied <= 0:
            # 制动回读会延续到下一次观测，重新计时，保存同向试探量。
            self.trial_x = x
            self.trial_time = 0.
            return
        self.trial_time += dt
        progress = direction * (x - self.trial_x)
        if progress >= .12 and direction * v > .3 and self.trial_time >= .12:
            # 仅记住已落实到执行器、且随后伴有同向位移的附加力矩。
            self.breakaway_ticks[direction] = output
            self.memory[direction] = min(applied, self.memory[direction] + self.extra)
            self.trial_dir = 0
            self.extra = 0.
        elif abs(progress) > .12:
            self.trial_dir = 0
            self.extra = 0.
        elif (abs(v) < .3 and self.trial_time >= self.delay + .12 and
              abs(output - self.last_target) <= 1.):
            self.extra = min(self.high - self.low, self.extra + 8. * dt)

    def _observe_limit(self, target, output, x, v, dt):
        side = 0
        if dt and abs(v) < .35 and abs(x) > .30:
            if target == self.high and output == self.high:
                side = 1
            elif target == self.low and output == self.low:
                side = -1
        if not side or side != self.limit_side or abs(x - self.limit_x) >= .08:
            self.limit_side, self.limit_x, self.limit_time = side, x, 0.
        else:
            self.limit_time += dt
            if self.limit_time >= self.delay + .50:
                raise ValueError('stroke limit without measured motion')

    def step(self, x_pred, v, dt, output_ticks, observed_x=None):
        # 向前预测补偿执行延迟时，另传曝光时刻实测或滤波位置作为位移证据。
        # 四参数调用默认使用本次观测的位置。
        measured = x_pred if observed_x is None else observed_x
        if (not all(map(math.isfinite, (measured, x_pred, v, dt, output_ticks))) or dt < 0 or
                not self.low <= output_ticks <= self.high):
            raise ValueError('invalid observation or PWM feedback')
        distance = abs(x_pred)
        direction = -_sign(x_pred)
        # 解 d = u * delay + u² / (2 * brake)，目标附近同时限制速度斜率。
        lag_speed = self.brake * self.delay
        speed = math.sqrt(lag_speed ** 2 + 2 * self.brake * distance) - lag_speed
        v_ref = direction * min(speed, 3. * distance, 4.)
        a_cmd = _clip(3.2 * (v_ref - v), -12., 12.)
        braking = a_cmd * v < 0
        drive_dir = direction if distance > .30 and direction * a_cmd > 0 else 0
        observed_dt = min(dt, .08) if dt <= .15 else 0.
        observed_dir = -_sign(measured) if abs(measured) > .30 else 0
        if dt > .15 or (observed_dt and observed_dir == self.trial_dir
                        and observed_dir and drive_dir != observed_dir):
            # 跨缺帧区间或暂时制动，只重建位移证据的起点，保留已累积的驱动量。
            self.trial_x, self.trial_time = measured, 0.
        elif observed_dt:
            self._observe_start(observed_dir if drive_dir == observed_dir else 0,
                                measured, v, observed_dt, output_ticks)

        # 起动补偿随速度上升、接近目标而连续退出。
        bias = 0.
        if drive_dir:
            weight = _clip((distance - .30) / .30, 0., 1.) * _clip(1. - abs(v), 0., 1.)
            trial = self.extra if direction == self.trial_dir else 0.
            bias = self.polarity * direction * weight * (self.memory[direction] + trial)
        gain = self.gains[1 if a_cmd >= 0 else -1]
        effort = self.polarity * a_cmd / gain + bias
        holding = distance <= .30 and abs(v) < .35
        if holding and not braking and observed_dt and abs(output_ticks - self.last_target) <= 2.:
            trim = -self.polarity * x_pred * 2. * observed_dt
            candidate = self.balance + trim
            if self.low < candidate + effort < self.high:
                room = min(self.neutral - self.low, self.high - self.neutral) * .25
                self.balance = _clip(candidate, self.neutral - room, self.neutral + room)
        target = _clip(self.balance + effort, self.low, self.high)
        if dt > 0:
            self._observe_limit(target, output_ticks, measured, v, observed_dt)
            self.last_target = target
        elif not (braking and direction * v >= 0):
            # 心跳只能撤回或维持已实际施加且最近被观测授权的同向驱动。
            side = _sign(target - self.neutral)
            allowed = max(0., min(side * (output_ticks - self.neutral),
                                  side * (self.last_target - self.neutral)))
            target = self.neutral + side * min(abs(target - self.neutral), allowed)
        phase = 'brake' if braking else ('hold' if holding else 'drive')
        return dict(target_ticks=target, v_ref=v_ref, a_cmd=a_cmd, start_bias=bias,
                    balance_ticks=self.balance, phase=phase)
