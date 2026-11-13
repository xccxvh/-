# -*- coding: utf-8 -*-
"""钢珠检测。

钢珠只露出顶部一小部分, 不能按完整圆检测。这里融合两种线索:

1. 几何线索: 轨道上缘包络的局部凸起(宽度/高度/宽高比形状先验);
2. 颜色线索: 钢珠顶部在本场景中呈明显蓝色(B-R 大且足够亮),
   用于在复杂背景中排除轨道边缘/浅色衣物等假凸起。

两种线索位置一致时置信度更高。颜色阈值依赖现场光照, 属于可配置项,
换背景或光照后应重新标定; 几何线索始终作为主判据之一。
"""
import math
from measurement_contract import COORDINATE_VERSION

import cv2
import numpy as np


class BallCandidate(object):
    def __init__(self, x, width, height, confidence, method, extra=None):
        self.x = float(x)
        self.width = float(width)
        self.height = float(height)
        self.confidence = float(confidence)
        self.method = method
        self.extra = extra or {}

    def as_dict(self):
        return {
            "x_px": self.x,
            "width_px": self.width,
            "height_px": self.height,
            "confidence": self.confidence,
            "method": self.method,
        }


class BallDetector(object):
    def __init__(self, cfg, width, height):
        self.cfg = cfg
        self.W = int(width)
        self.H = int(height)
        self.use_color = bool(cfg.get("use_color", True))
        self.use_geom = bool(cfg.get("use_geometry", True))

        self.blue_minus_red = float(cfg.get("blue_minus_red", 30))
        self.blue_min = float(cfg.get("blue_min", 140))
        self.band_above = float(cfg.get("band_above_ratio", 0.05)) * self.H
        self.band_below = float(cfg.get("band_below_ratio", 0.012)) * self.H
        self.dev_thr = float(cfg.get("dev_threshold_ratio", 0.008)) * self.H
        self.env_thr = float(cfg.get("envelope_threshold", 100))

        self.min_w = float(cfg.get("min_width_ratio", 0.018)) * self.W
        self.max_w = float(cfg.get("max_width_ratio", 0.075)) * self.W
        self.min_h = float(cfg.get("min_height_ratio", 0.004)) * self.H
        self.max_h = float(cfg.get("max_height_ratio", 0.035)) * self.H
        self.min_aspect = float(cfg.get("min_aspect", 1.3))
        self.max_aspect = float(cfg.get("max_aspect", 8.0))
        self.min_area = float(cfg.get("min_area_ratio", 0.00015)) * self.W * self.H
        self.max_area = float(cfg.get("max_area_ratio", 0.006)) * self.W * self.H
        self.typ_w = float(cfg.get("typical_width_ratio", 0.038)) * self.W
        self.typ_h = float(cfg.get("typical_height_ratio", 0.018)) * self.H
        self.x_margin = float(cfg.get("x_margin", 0.12)) * self.W
        # 仅颜色、没有几何凸起支持的候选只作弱证据。当前机位轨道上缘
        # 有蓝色反光, 颜色分支容易把它当成钢珠; 几何分支才是主判据。
        self.color_only_scale = float(cfg.get("color_only_scale", 0.5))

        self.kernel = np.ones((3, 3), np.uint8)

    def _roi(self, rail):
        """轨道上缘附近的窄带, 只处理这一条 ROI。

        轨道可能倾斜, 取两端 y 的极值再向外扩出一个安全边距,
        保证整条轨道和钢珠顶部都落在 ROI 内。
        """
        y_left = rail.slope * 0 + rail.intercept
        y_right = rail.slope * (self.W - 1) + rail.intercept
        y_min = min(y_left, y_right)
        y_max = max(y_left, y_right)
        pad = 4.0
        y0 = int(math.floor(y_min - self.band_above - pad))
        y1 = int(math.ceil(y_max + self.band_below + pad))
        y0 = max(0, y0)
        y1 = min(self.H, y1)
        return y0, y1

    def _band(self, rail, y0, y1):
        xs = np.arange(self.W, dtype=np.float32)
        ys = rail.slope * xs + rail.intercept
        rows = np.arange(y0, y1, dtype=np.float32)[:, None]
        band = (rows >= (ys[None, :] - self.band_above)) & \
               (rows <= (ys[None, :] + self.band_below))
        return band

    def _shape_score(self, w, h, area=None):
        if w <= 0 or h <= 0:
            return None
        if not (self.min_w <= w <= self.max_w):
            return None
        if not (self.min_h <= h <= self.max_h):
            return None
        if not (self.min_aspect <= (w / h) <= self.max_aspect):
            return None
        if area is not None and not (self.min_area <= area <= self.max_area):
            return None
        sw = math.exp(-((w - self.typ_w) / (0.5 * self.typ_w)) ** 2)
        sh = math.exp(-((h - self.typ_h) / (0.6 * self.typ_h)) ** 2)
        return 0.55 + 0.25 * sw + 0.20 * sh

    def _color_candidates(self, frame, band, rail, y0, y1):
        out = []
        roi = frame[y0:y1]
        b = roi[:, :, 0]
        r = roi[:, :, 2]
        # 饱和减法: 负数截为 0, 正好只关心 B-R 大于阈值的情况
        diff = cv2.subtract(b, r)
        mask = (diff > self.blue_minus_red) & (b > self.blue_min) & band
        mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN,
                                self.kernel)
        n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
        x0, x1 = self.x_margin, self.W - self.x_margin
        # Most components are tiny ruler reflections. Apply the same necessary
        # shape bounds in bulk before Python-level clipping/shape evaluation.
        # Keep label order and the original centroid/score for surviving blobs.
        sizes = stats[1:]
        widths, heights, areas = sizes[:, 2], sizes[:, 3], sizes[:, 4]
        eligible = ((widths >= self.min_w) & (widths <= self.max_w) &
                    (heights >= self.min_h) & (heights <= self.max_h) &
                    (areas >= self.min_area) & (areas <= self.max_area))
        for i in np.flatnonzero(eligible) + 1:
            x, y, w, h, area = [int(v) for v in stats[i]]
            if x <= 0 or (x + w) >= self.W:
                continue
            # 按连通域所在列的局部搜索带判断是否被截断。被截断说明它
            # 是更大的蓝色物体(衣物/箱子)的一部分, 不是钢珠顶部。
            yl = rail.y_at(x) - y0
            yr = rail.y_at(x + w) - y0
            top = min(yl, yr) - self.band_above
            bot = max(yl, yr) + self.band_below
            if y <= top + 2 or (y + h) >= bot - 2:
                continue
            score = self._shape_score(w, h, area)
            if score is None:
                continue
            cx = float(cents[i][0])
            if not (x0 <= cx <= x1):
                continue
            out.append(BallCandidate(cx, w, h, score, "color",
                                     {"area": area,
                                      "bbox": (x, y + y0, w, h)}))
        return out

    def _geom_candidates(self, gray, band, rail, y0, y1, threshold=None, recovery_pass=False):
        """轨道上缘亮度包络的局部凸起。"""
        out = []
        roi = gray[y0:y1]
        seed_threshold = max(80.0, 0.8 * self.env_thr) if recovery_pass else self.env_thr
        bright = (roi > (seed_threshold if threshold is None else threshold)) & band
        has = bright.any(axis=0)
        if int(has.sum()) < 50:
            return out

        first = np.argmax(bright, axis=0).astype(np.float32)
        xs = np.arange(self.W, dtype=np.float32)
        y_line = rail.slope * xs + rail.intercept - y0
        dev = np.full(self.W, np.nan, dtype=np.float32)
        idx = np.where(has)[0]
        dev[idx] = y_line[idx] - first[idx]

        x0 = int(self.x_margin)
        x1 = int(self.W - self.x_margin)
        good = idx[(idx >= x0) & (idx <= x1)]
        if len(good) < 50:
            return out

        offset = float(np.nanmedian(dev[good]))
        devc = dev - offset

        seg = np.zeros(self.W, dtype=bool)
        # The low-brightness recovery contour has integer-row quantization.
        # Allow one pixel at its shoulders; retain primary segmentation and
        # the original center weights, shape/seed/background checks.
        segment_height = (self.dev_thr if threshold is None and not recovery_pass
                          else max(0.0, self.dev_thr - 1.0))
        seg[x0:x1 + 1] = devc[x0:x1 + 1] > segment_height
        d = np.diff(np.concatenate(([0], seg.astype(np.int8), [0])))
        starts = np.where(d == 1)[0]
        ends = np.where(d == -1)[0]

        for s, e in zip(starts, ends):
            w = e - s
            if w < self.min_w or w > self.max_w:
                continue
            # A bright object continuing above the local search band is
            # clipped background, not a fully observed ball cap. Test before
            # median baseline subtraction, which can disguise a clipped
            # plateau as a plausible-height bump.
            local_top = y_line[s:e] - self.band_above
            if np.any(first[s:e] <= np.ceil(local_top) + 1):
                continue
            vals = devc[s:e]
            if not np.any(np.isfinite(vals)):
                continue
            peak = float(np.nanmax(vals))
            # Preserve the calibrated primary estimator. For dim-cap recovery,
            # integrate the observed envelope instead of assuming a triangle.
            area = w * peak * 0.5 if threshold is None else float(np.nansum(np.maximum(vals, 0)))
            score = self._shape_score(w, peak, area=area)
            if score is None:
                continue
            wt = np.clip(np.nan_to_num(vals) - self.dev_thr, 0.0, None)
            denom = float(wt.sum())
            if denom <= 0:
                continue
            cx = float((np.arange(s, e, dtype=np.float32) * wt).sum() / denom)
            cy = float(((first[s:e] + y0) * wt).sum() / denom)
            extra = {"peak_px": peak, "measurement_y_px": cy}
            if threshold is not None:
                extra.update(span=(int(s), int(e)), profile=vals.copy())
            # Secondary brightness is weaker evidence: it must pass the
            # rounded low-contour checks below, never this direct path.
            if not (recovery_pass and threshold is None):
                out.append(BallCandidate(cx, w, peak, score * 0.9, "geom", extra))
        if threshold is None:
            # Hysteresis: recover a dim continuous cap only when it contains
            # one or more bright fragments. Never join disconnected glints or
            # lower the global minimum width. Existing primary coordinates win.
            lower = max(35.0, 0.6 * seed_threshold)
            if lower < seed_threshold:
                low = self._geom_candidates(gray, band, rail, y0, y1, lower)
                for candidate in low:
                    s, e = candidate.extra['span']
                    if any(s - 2 <= c.x <= e + 2 for c in out):
                        continue
                    seeds = [(int(a), int(b)) for a, b in zip(starts, ends)
                             if a >= s - 2 and b <= e + 2 and b - a >= 3]
                    if not 1 <= len(seeds) <= 4:
                        continue
                    if sum(b-a for a,b in seeds) < .35 * (e-s):
                        continue
                    if any(seeds[i+1][0]-seeds[i][1] > .45*self.typ_w
                           for i in range(len(seeds)-1)):
                        continue
                    profile = candidate.extra['profile']
                    xs_fit = np.linspace(-1.0, 1.0, len(profile))
                    q = np.polyfit(xs_fit, profile, 2)
                    # A rounded cap has a concave-down envelope and an
                    # interior apex. Flat ruler patches fail this test.
                    if q[0] >= -3.0 or abs(-q[1]/(2*q[0])) > .65:
                        continue
                    residual = np.abs(np.polyval(q, xs_fit)-profile)
                    if np.median(residual) > max(1.5, .15*candidate.height):
                        continue
                    candidate.method = 'geom_recovered'
                    candidate.extra = {'peak_px':candidate.height,
                        'measurement_y_px':candidate.extra['measurement_y_px'],
                        'coordinate_version':'geometry-hysteresis-v2',
                        'threshold':lower, 'seed_count':len(seeds)}
                    out.append(candidate)
        if threshold is None:
            if recovery_pass:
                # Every coordinate from the secondary brightness level is
                # uncalibrated, including an unsplit cap at the seed level.
                for candidate in out:
                    candidate.method = 'geom_recovered'
                    candidate.extra.update(coordinate_version='geometry-hysteresis-v2',
                                           seed_threshold=seed_threshold)
        return out

    def detect(self, frame, gray, rail):
        """返回按置信度降序排列的候选列表。"""
        cands = []
        if not rail.valid:
            return cands

        y0, y1 = self._roi(rail)
        if y1 - y0 < 8:
            return cands
        band = self._band(rail, y0, y1)
        color = (self._color_candidates(frame, band, rail, y0, y1)
                 if self.use_color else [])
        geom = (self._geom_candidates(gray, band, rail, y0, y1)
                if self.use_geom else [])

        # Dispatch here so insufficient bright columns in the primary pass
        # cannot bypass recovery. At most one secondary pass, with seed>=80.
        if self.use_geom and not geom and max(80.0, 0.8 * self.env_thr) < self.env_thr:
            geom = self._geom_candidates(gray, band, rail, y0, y1,
                                         recovery_pass=True)

        # Geometry defines position. Color only corroborates the candidate;
        # changing reflection/white balance must never move the coordinate.
        # Globally closest admissible pairs give one-to-one association.
        pairs = []
        for i, c in enumerate(color):
            for j, gc in enumerate(geom):
                distance = abs(c.x - gc.x)
                if distance < min(0.03 * self.W, 0.5 * gc.width):
                    pairs.append((distance, i, j))
        used_color, used_geom = set(), set()
        for distance, i, j in sorted(pairs):
            if i in used_color or j in used_geom:
                continue
            used_color.add(i)
            used_geom.add(j)
            c, gc = color[i], geom[j]
            extra = dict(gc.extra)
            extra.update(color_x_px=c.x, color_distance_px=distance,
                         color_bbox=c.extra.get('bbox'),
                         coordinate_version=gc.extra.get('coordinate_version', COORDINATE_VERSION))
            cands.append(BallCandidate(gc.x, gc.width, gc.height,
                         min(0.98, max(c.confidence, gc.confidence) + 0.15),
                         'hybrid_recovered' if gc.method == 'geom_recovered' else 'hybrid', extra))
        cands.extend(c for i, c in enumerate(color) if i not in used_color)
        cands.extend(gc for j, gc in enumerate(geom) if j not in used_geom)
        for c in cands:
            if c.method == "color":
                c.confidence *= self.color_only_scale
        cands.sort(key=lambda c: c.confidence, reverse=True)
        return cands
