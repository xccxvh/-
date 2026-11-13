# -*- coding: utf-8 -*-
"""Time-based ball association; predictions are never valid measurements.

Pass monotonic seconds for live frames and video seconds for replay. Velocity
is pixels/second. Gates are image-space heuristics, not mechanical limits.
"""
import time
import math


class BallTracker(object):
    def __init__(self, cfg, width, height):
        self.W = int(width)
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

    def _make(self, ts, x_px, valid, confidence, state, predicted, method=None):
        self.state = state
        return dict(timestamp=ts, x_px=x_px, x_cm=None, valid=bool(valid),
                    confidence=float(confidence), state=state,
                    predicted=bool(predicted), method=method)

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
            return None
        if len(near) > 1 and (abs(near[1].x-predicted) -
                              abs(near[0].x-predicted) <= self.ambiguity_px):
            return None
        return near[0]

    def update(self, candidates, ts=None):
        if ts is None:
            ts = time.monotonic()
        if not math.isfinite(ts):
            return self._make(time.monotonic(), None, False, 0., 'UNCERTAIN', False)
        if self.last_input_ts is not None and ts <= self.last_input_ts:
            return self._make(ts, None, False, 0., 'UNCERTAIN', False)
        # Fresh input after a gap must not reuse old identity or velocity.
        reference = self.last_ts if self.x is not None else self.acq_ts
        if reference is not None and ts-reference > self.max_gap:
            self.reset()
        self.last_input_ts = ts
        candidates = [c for c in candidates if math.isfinite(c.x) and
                      math.isfinite(c.confidence) and 0 <= c.x < self.W and
                      self.conf_thr <= c.confidence <= 1]
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
            if chosen is None:
                self.acq = 0
                self.acq_x = self.acq_ts = None
                self.acq_v = 0.0
                return self._make(ts, None, False, 0.,
                                  'UNCERTAIN' if candidates else 'LOST', False)
            if self.acq_ts is not None:
                dt = ts - self.acq_ts
                measured = (chosen.x - self.acq_x) / dt
                if self.acq == 1:
                    self.acq_v = max(-self.max_speed, min(self.max_speed, measured))
                else:
                    self.acq_v = self._filter_velocity(self.acq_v, measured, dt)
            self.acq += 1
            self.acq_x, self.acq_ts = chosen.x, ts
            if self.acq < self.acquire_frames:
                return self._make(ts, chosen.x, False, chosen.confidence,
                                  'UNCERTAIN', False, chosen.method)
            self.x, self.last_ts = chosen.x, ts
            self.v, self.miss = self.acq_v, 0
            self.conf = chosen.confidence
            self.acq = 0
            self.acq_x = self.acq_ts = None
            return self._make(ts, self.x, True, self.conf, 'DETECTED', False, chosen.method)

        dt = ts-self.last_ts
        pred = self.x+self.v*dt
        plausible = [c for c in candidates
                     if abs(c.x-self.x) <= self.base_gate+self.max_speed*dt]
        chosen = self._nearest(plausible, pred, self._gate(dt))
        if chosen is not None:
            velocity = (chosen.x-self.x)/dt
            self.v = self._filter_velocity(self.v, velocity, dt)
            self.x, self.last_ts = chosen.x, ts
            self.conf, self.miss = chosen.confidence, 0
            return self._make(ts, self.x, True, self.conf, 'DETECTED', False, chosen.method)
        self.miss += 1
        if self.miss <= self.uncertain_frames:
            pred = max(0., min(self.W-1., pred))
            return self._make(ts, pred, False, self.conf*.5, 'UNCERTAIN', True)
        self.reset()
        self.last_input_ts = ts
        return self._make(ts, None, False, 0., 'LOST', False)
