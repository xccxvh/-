# -*- coding: utf-8 -*-
"""Time-based ball association; predictions are never valid measurements.

Pass monotonic seconds for live frames and video seconds for replay. Velocity
is pixels/second. Gates are image-space heuristics, not mechanical limits.
"""
import time
import math
from numbers import Real


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

    def _identity_ok(self, candidate):
        extra = getattr(candidate, 'extra', None)
        # 明确在搜索 ROI 外的候选始终无效；旧候选没有该字段时保持兼容。
        if isinstance(extra, dict) and 'in_search_roi' in extra and extra['in_search_roi'] is not True:
            return False
        if not self.identity_check:
            return True
        if (candidate.method != 'relative_cap' or not isinstance(extra, dict) or
                extra.get('identity_valid') is not True or
                extra.get('in_search_roi') is not True or
                extra.get('coordinate_version') != 'relative-cap-v1'):
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
        return abs(appearance - contrast) <= tolerance

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

    def update(self, candidates, ts=None):
        if ts is None:
            ts = time.monotonic()
        if not math.isfinite(ts):
            return self._make(time.monotonic(), None, False, 0., 'UNCERTAIN', False, reason='time')
        if self.last_input_ts is not None and ts <= self.last_input_ts:
            return self._make(ts, None, False, 0., 'UNCERTAIN', False, reason='time')
        # Fresh input after a gap must not reuse old identity or velocity.
        reference = self.last_ts if self.x is not None else self.acq_ts
        if reference is not None and ts-reference > self.max_gap:
            self.reset()
        self.last_input_ts = ts
        candidates = [c for c in candidates if math.isfinite(c.x) and
                      math.isfinite(c.confidence) and 0 <= c.x < self.W and
                      self.conf_thr <= c.confidence <= 1]
        # 先保留旧置信度/坐标过滤，再做身份筛选，避免干扰候选参与歧义排序。
        eligible = [c for c in candidates if self._identity_ok(c)]
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
            self.x, self.last_ts = chosen.x, ts
            self.v, self.miss = self.acq_v, 0
            self.conf = chosen.confidence
            self.acq = 0
            self.acq_x = self.acq_ts = None
            return self._make(ts, self.x, True, self.conf, 'DETECTED', False, chosen.method, reason='tracked')

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
        self.reset()
        self.last_input_ts = ts
        return self._make(ts, None, False, 0., 'LOST', False,
                          reason=self._selection_reason)
