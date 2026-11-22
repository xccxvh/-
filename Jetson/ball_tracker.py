# -*- coding: utf-8 -*-
"""Time-based ball association; predictions are never valid measurements.

Pass monotonic seconds for live frames and video seconds for replay. Velocity
is pixels/second. Gates are image-space heuristics, not mechanical limits.
"""
import time
import math

import cv2
from collections import namedtuple
from numbers import Real
from measurement_contract import LOCAL_MEASUREMENT_VERSIONS


# 只保存已选中的单个实测圆的数值快照。depth 从当帧最近的物理边界向内为正。
_Circle = namedtuple('_Circle', 'ts x y diameter appearance edge sign depth')


class _EdgeReturn(object):
    """有界的边界证据；不保存图像，不外推球心，不向普通跟踪提供预测位置。"""
    def __init__(self):
        self.confirmed = self.previous = self.departure = self.inward = None
        self.absent = False
        self.count = self.blanks = 0

    @staticmethod
    def compatible(a, b):
        # 直径、高光以及相对于横向移动的高度变化需同时一致；限制弯道/倾斜的纵向偏差。
        return (.8 <= b.diameter / a.diameter <= 1.25 and
                abs(b.appearance-a.appearance) <= max(8., .30*a.appearance) and
                abs(b.y-a.y) <= .35*a.diameter + .15*abs(b.x-a.x))

    def observe(self, ball, valid, gate, max_speed):
        # 常规关联已选择这一个强圆；只有最近确认过同一钢珠才记录出向穿过边界的球缘。
        old = self.previous
        self.previous = ball
        anchor = self.confirmed
        if valid:
            self.confirmed = ball
        if (old and anchor and ball.ts-anchor.ts <= .45 and ball.ts-old.ts <= .10 and
                old.sign == ball.sign and abs(ball.edge-old.edge) <= .2*ball.diameter and
                old.depth > old.diameter/2 and ball.depth <= ball.diameter/2 and
                (old.x-ball.x)*ball.sign >= 4 and old.depth-ball.depth >= 4 and
                abs(ball.x-old.x) <= gate(ball.ts-old.ts) and
                abs(ball.x-anchor.x) <= gate(0) + max_speed*(ball.ts-anchor.ts) and
                self.compatible(old, ball) and self.compatible(anchor, ball)):
            self.departure = ball
            self.absent = False
            self.inward = None
            self.count = self.blanks = 0
            return False

        start = self.departure
        if start is None:
            return False
        if not self.absent:
            # 穿过边界后还可观察到继续向外的圆，之后必须出现真正的空帧。
            if (ball.sign == start.sign and ball.depth <= start.depth and
                    self.compatible(start, ball) and abs(ball.edge-start.edge) <= .2*start.diameter):
                self.departure = ball
            else:
                self.departure = None
            return False

        if (valid or ball.ts-start.ts > 1.6 or ball.sign != start.sign or
                abs(ball.edge-start.edge) > .2*start.diameter or
                not self.compatible(start, ball)):
            self.departure = None
            return False
        prev = self.inward
        if prev is None:
            if ball.depth > ball.diameter/2 or ball.ts-start.ts < .10:
                self.departure = None
                return False
        else:
            # 边界相对位移和图像位移都须指向内侧，排除纸尺注册跳动；速度仍用原门限。
            dt = ball.ts-prev.ts
            dx = (ball.x-prev.x)*ball.sign
            if (dt > .13 or not 3 <= dx <= gate(dt) or ball.depth-prev.depth < 3 or
                    ball.depth > 2*start.diameter or not self.compatible(prev, ball)):
                self.departure = None
                return False
        self.inward = ball
        self.count += 1
        consecutive = self.blanks == 0
        self.blanks = 0
        if self.count >= 3 and consecutive:
            self.departure = None
            return True
        return False

    def empty(self, ts, bounds):
        self.previous = None
        self.absent = True
        if self.inward:
            self.blanks += 1
        if self.departure:
            # 即使没有候选也检查当帧边界，避免参考先跳走又回来时误用旧出向证据。
            start = self.departure
            edge = bounds[0] if start.sign == 1 else bounds[1]
            if ts-start.ts > 1.6 or self.blanks > 1 or abs(edge-start.edge) > .2*start.diameter:
                self.departure = None


class BallTracker(object):
    def __init__(self, cfg, width, height):
        self.W = int(width)
        self.H = int(height)
        # 新身份门控默认关闭，旧检测器无需补元数据；原时序/速度门限不变。
        self.identity_check = bool(cfg.get('identity_check', False))
        self.identity_size_ratio = float(cfg.get('identity_size_ratio', 2.0))
        self.identity_contrast_ratio = float(cfg.get('identity_contrast_ratio', .5))
        self.identity_contrast_floor = float(cfg.get('identity_contrast_floor', 5.0))
        if self.identity_check and not (
                math.isfinite(self.identity_size_ratio) and self.identity_size_ratio >= 1 and
                math.isfinite(self.identity_contrast_ratio) and self.identity_contrast_ratio >= 0 and
                math.isfinite(self.identity_contrast_floor) and self.identity_contrast_floor >= 0):
            raise ValueError('invalid identity continuity thresholds')
        self.max_jump = float(cfg.get('max_jump_ratio', .12)) * width
        self.uncertain_frames = int(cfg.get('uncertain_frames', 4))
        self.conf_thr = float(cfg.get('confidence_threshold', .55))
        self.alpha = min(1., max(0., float(cfg.get('velocity_alpha', .5))))
        self.acquire_frames = int(cfg.get('acquire_frames', 3))
        self.max_gap = float(cfg.get('max_gap_s', .25))
        self.max_speed = float(cfg.get('max_speed_ratio_s', 1.2)) * width
        self.base_gate = float(cfg.get('position_noise_ratio', .015)) * width
        self.ambiguity_px = float(cfg.get('ambiguity_ratio', .005)) * width
        self.acquire_margin = float(cfg.get('acquire_confidence_margin', .05))
        self.reset()

    def reset(self):
        self._edge = _EdgeReturn()
        self._reset_track()

    def _reset_track(self):
        self.state = 'LOST'
        self.x = None
        self.v = 0.0
        self.miss = 0
        self.conf = 0.0
        self.acq = 0
        self.acq_x = None
        self.acq_ts = None
        self.acq_v = 0.0
        self.last_ts = None
        self.last_input_ts = None
        # 仅保存已接受测量的数值快照，避免候选复用/外部修改污染身份。
        self.identity = None
        self._surface = None

    def _make(self, ts, x_px, valid, confidence, state, predicted, method=None,
              reason=None):
        self.state = state
        return dict(timestamp=ts, x_px=x_px, x_cm=None, valid=bool(valid),
                    confidence=float(confidence), state=state,
                    predicted=bool(predicted), method=method, reason=reason)

    @staticmethod
    def _finite_scalar(value):
        # 不将字符串、布尔量或数组隐式转换成外观标量；异常元数据关闭有效输出。
        return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)

    @staticmethod
    def _same_surface(candidate, anchor):
        """静止球圆心拟合抖动时，在当前帧测得同一球面纹理。匹配点不作位置输出。"""
        if anchor is None:
            return False
        patch = candidate.extra.get('surface_patch')
        if patch is None:
            return False
        ax, ay, template = anchor
        x, y = round(candidate.x), round(candidate.extra['measurement_y_px'])
        if template.std() < 30:
            return False
        # 搜索窗只容许圆拟合的小幅偏差。即使两张纹理很像，配准后的球面若有
        # 明显平移也不能走此静止身份复核；速度门控、圆度和尺寸检查均单独生效。
        score = cv2.matchTemplate(patch, template, cv2.TM_CCOEFF_NORMED)
        _, peak, _, (u, v) = cv2.minMaxLoc(score)
        return (peak >= .98 and 0 < u < 24 and 0 < v < 24
                and math.hypot(x+u-12-ax, y+v-12-ay) <= 3)

    def _identity_ok(self, candidate, anchor=None):
        extra = getattr(candidate, 'extra', None)
        # 明确在搜索 ROI 外的候选始终无效；旧候选没有该字段时保持兼容。
        if isinstance(extra, dict) and 'in_search_roi' in extra and extra['in_search_roi'] is not True:
            return False
        if not self.identity_check:
            return True
        if (candidate.method not in LOCAL_MEASUREMENT_VERSIONS or not isinstance(extra, dict) or
                extra.get('identity_valid') is not True or
                extra.get('in_search_roi') is not True or
                extra.get('coordinate_version') != LOCAL_MEASUREMENT_VERSIONS[candidate.method]):
            return False
        appearance = extra.get('appearance')
        y = extra.get('measurement_y_px')
        if not all(self._finite_scalar(v) for v in
                   (candidate.width, candidate.height, appearance, y)):
            return False
        if candidate.width <= 0 or candidate.height <= 0 or not 0 <= y < self.H:
            return False
        if self.identity is None:
            return True
        width, height, contrast = self.identity
        # 分别约束宽、高，避免面积相同但形状突变的干扰；采集期同样执行。
        ratio = self.identity_size_ratio
        if not (1. / ratio <= candidate.width / width <= ratio and
                1. / ratio <= candidate.height / height <= ratio):
            return False
        # 背景相对对比度允许小幅照明波动；低对比度附近使用绝对容差避免除零。
        tolerance = max(self.identity_contrast_floor,
                        abs(contrast) * self.identity_contrast_ratio)
        return abs(appearance - contrast) <= tolerance or self._same_surface(candidate, anchor)

    def _remember_identity(self, candidate):
        if self.identity_check:
            self.identity = (candidate.width, candidate.height,
                             candidate.extra['appearance'])

    def _gate(self, dt):
        return min(self.max_jump, self.base_gate + self.max_speed * dt)

    def _filter_velocity(self, previous, measured, dt):
        # Preserve the configured weight at 30 Hz; use the equivalent
        # continuous decay for other intervals, including dropped frames.
        weight = 1.0 - (1.0 - self.alpha) ** (dt * 30.0)
        measured = max(-self.max_speed, min(self.max_speed, measured))
        return weight * measured + (1.0 - weight) * previous

    def _nearest(self, candidates, predicted, gate):
        near = sorted((c for c in candidates if abs(c.x-predicted) <= gate),
                      key=lambda c: abs(c.x-predicted))
        if not near:
            if candidates:
                self._selection_reason = 'position_rejected'
            return None
        if len(near) > 1 and (abs(near[1].x-predicted) -
                              abs(near[0].x-predicted) <= self.ambiguity_px):
            self._selection_reason = 'ambiguous'
            return None
        return near[0]

    def update(self, candidates, ts=None, search_bounds=None):
        """search_bounds 为当帧有效物理搜索区的 (左, 右) 像素坐标，缺省使用原关联流程。"""
        if ts is None:
            ts = time.monotonic()
        previous_ts = self.last_input_ts
        last_x, last_v, last_time = self.x, self.v, self.last_ts
        candidates = list(candidates)
        result = self._update(candidates, ts)
        self._surface = None
        if (result['valid'] and len(candidates) == 1 and candidates[0].method == 'topview_circle'
                and candidates[0].x == result['x_px']):
            c = candidates[0]
            patch = c.extra.get('surface_patch')
            if patch is not None:
                self._surface = (round(c.x), round(c.extra['measurement_y_px']), patch[12:43, 12:43].copy())
        if (not self.identity_check or result['reason'] == 'time' or
                not isinstance(search_bounds, (tuple, list)) or len(search_bounds) != 2 or
                not all(self._finite_scalar(b) for b in search_bounds) or
                not 0 <= search_bounds[0] < search_bounds[1] < self.W):
            self._edge = _EdgeReturn()
            return result
        if previous_ts is not None and ts-previous_ts > self.max_gap:
            self._edge = _EdgeReturn()
        anchor = self._edge.confirmed
        if anchor is not None:
            # 缺测期间仍核实锚点对应的物理边界；中间参考跳变也会取消旧身份。
            anchor_edge = search_bounds[0] if anchor.sign == 1 else search_bounds[1]
            if abs(anchor_edge-anchor.edge) > .2*anchor.diameter:
                self._edge = _EdgeReturn()
                anchor = None
        if not candidates:
            self._edge.empty(ts, search_bounds)
            return result
        c = candidates[0]
        # 弱圆、多个候选、时间/位置或身份拒绝均销毁快捷通道。普通跟踪沿用原策略。
        if (len(candidates) != 1 or result['reason'] not in ('tracked', 'acquiring') or
                c.method != 'topview_circle' or
                not search_bounds[0] <= c.x <= search_bounds[1]):
            self._edge = _EdgeReturn()
            return result
        left, right = search_bounds
        edge, sign = (left, 1) if c.x-left < right-c.x else (right, -1)
        circle = _Circle(ts, c.x, c.extra['measurement_y_px'], c.width,
                         c.extra['appearance'], edge, sign, (c.x-edge)*sign)
        if c.extra.get('proposal_source') == 'motion_blur':
            # 补搜来自检测器对当帧像素的独立核验。它永远清除穿界和入向计数；
            # 仅在远离两侧边界且运动仍连续时，允许保留原 Hough 身份及原始时间戳。
            self._edge = _EdgeReturn()
            if (result['valid'] and anchor and last_time is not None and
                    circle.sign == anchor.sign and ts-anchor.ts <= .1 and
                    min(circle.depth, anchor.depth) > 2*max(circle.diameter, anchor.diameter) and
                    (circle.x-last_x)*last_v > 0 and
                    abs(circle.x-last_x-last_v*(ts-last_time)) <= self.base_gate and
                    self._edge.compatible(anchor, circle)):
                self._edge.confirmed = anchor
            return result
        if c.confidence < .85:
            self._edge = _EdgeReturn()
            return result
        ready = self._edge.observe(circle, result['valid'], self._gate, self.max_speed)
        if ready and self.acq >= 2:
            self._edge.confirmed = circle
            return self._confirm(c, ts, 'edge_return')
        return result

    def _confirm(self, chosen, ts, reason):
        # 普通获取与边界重获提交同一份当帧测量；速度只由本轮采集的实测位置生成。
        self.x, self.last_ts = chosen.x, ts
        self.v, self.miss = self.acq_v, 0
        self.conf = chosen.confidence
        self.acq = 0
        self.acq_x = self.acq_ts = None
        return self._make(ts, self.x, True, self.conf, 'DETECTED', False, chosen.method, reason=reason)

    def _update(self, candidates, ts):
        if not math.isfinite(ts):
            return self._make(time.monotonic(), None, False, 0., 'UNCERTAIN', False, reason='time')
        if self.last_input_ts is not None and ts <= self.last_input_ts:
            return self._make(ts, None, False, 0., 'UNCERTAIN', False, reason='time')
        # Fresh input after a gap must not reuse old identity or velocity.
        reference = self.last_ts if self.x is not None else self.acq_ts
        if reference is not None and ts-reference > self.max_gap:
            self._reset_track()
        self.last_input_ts = ts
        candidates = [c for c in candidates if math.isfinite(c.x) and
                      math.isfinite(c.confidence) and 0 <= c.x < self.W and
                      self.conf_thr <= c.confidence <= 1]
        # 先保留旧置信度/坐标过滤，再做身份筛选，避免干扰候选参与歧义排序。
        # 只允许紧随唯一有效实测的下一帧使用球面模板；冷启动、漏检、快速运动
        # 或竞争目标仍需原有的完整身份确认。真实灰度来自本次相机采集。
        anchor = (self._surface if len(candidates) == 1 and self.x is not None
                  and self.last_ts is not None and 0 < ts - self.last_ts <= .075 else None)
        eligible = [c for c in candidates if self._identity_ok(c, anchor)]
        self._selection_reason = ('identity_rejected' if candidates and not eligible
                                  else 'no_candidates' if not candidates
                                  else 'position_rejected')
        candidates = eligible
        if self.x is None:
            chosen = None
            if self.acq_x is not None:
                chosen = self._nearest(candidates, self.acq_x,
                                       self._gate(ts-self.acq_ts))
            else:
                ranked = sorted(candidates, key=lambda c: c.confidence, reverse=True)
                if ranked and (len(ranked) == 1 or
                               ranked[0].confidence-ranked[1].confidence > self.acquire_margin):
                    chosen = ranked[0]
                elif ranked:
                    self._selection_reason = 'ambiguous'
            if chosen is None:
                self.acq = 0
                self.acq_x = self.acq_ts = None
                self.acq_v = 0.0
                self.identity = None
                return self._make(ts, None, False, 0.,
                                  'UNCERTAIN' if candidates else 'LOST', False,
                                  reason=self._selection_reason)
            if self.acq_ts is not None:
                dt = ts - self.acq_ts
                measured = (chosen.x - self.acq_x) / dt
                if self.acq == 1:
                    self.acq_v = max(-self.max_speed, min(self.max_speed, measured))
                else:
                    self.acq_v = self._filter_velocity(self.acq_v, measured, dt)
            self._remember_identity(chosen)
            self.acq += 1
            self.acq_x, self.acq_ts = chosen.x, ts
            if self.acq < self.acquire_frames:
                return self._make(ts, chosen.x, False, chosen.confidence,
                                  'UNCERTAIN', False, chosen.method, reason='acquiring')
            return self._confirm(chosen, ts, 'tracked')

        dt = ts-self.last_ts
        pred = self.x+self.v*dt
        plausible = [c for c in candidates
                     if abs(c.x-self.x) <= self.base_gate+self.max_speed*dt]
        chosen = self._nearest(plausible, pred, self._gate(dt))
        if chosen is not None:
            self._remember_identity(chosen)
            velocity = (chosen.x-self.x)/dt
            self.v = self._filter_velocity(self.v, velocity, dt)
            self.x, self.last_ts = chosen.x, ts
            self.conf, self.miss = chosen.confidence, 0
            return self._make(ts, self.x, True, self.conf, 'DETECTED', False, chosen.method, reason='tracked')
        self.miss += 1
        if self.miss <= self.uncertain_frames:
            pred = max(0., min(self.W-1., pred))
            return self._make(ts, pred, False, self.conf*.5, 'UNCERTAIN', True,
                              reason=self._selection_reason)
        self._reset_track()
        self.last_input_ts = ts
        return self._make(ts, None, False, 0., 'LOST', False,
                          reason=self._selection_reason)
