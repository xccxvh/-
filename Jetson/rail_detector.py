# -*- coding: utf-8 -*-
"""轨道上缘检测。

思路:
1. 在画面中部搜索带内, 取每列最大的正向垂直梯度(暗->亮)作为候选边缘点;
2. 用 RANSAC 拟合一条直线, 剔除背景干扰;
3. 每隔若干帧刷新一次, 其余帧复用上一条轨道线。

轨道倾斜时, 每帧/定期重新检测即可更新轨道坐标系。
"""
import cv2
import numpy as np


class RailLine(object):
    def __init__(self, slope, intercept, valid=False, confidence=0.0):
        self.slope = float(slope)
        self.intercept = float(intercept)
        self.raw_slope = self.slope
        self.raw_intercept = self.intercept
        self.valid = bool(valid)
        self.confidence = float(confidence)

    def y_at(self, x):
        return self.slope * x + self.intercept

    def angle_deg(self):
        return float(np.degrees(np.arctan(self.slope)))


class RailDetector(object):
    def __init__(self, cfg, width, height):
        self.cfg = cfg
        self.W = int(width)
        self.H = int(height)
        self.y0_ratio, self.y1_ratio = cfg.get("y_band", [0.33, 0.68])
        self.x_margin = float(cfg.get("x_margin", 0.12))
        self.grad_thr = float(cfg.get("grad_threshold", 45))
        self.slope_limit = float(cfg.get("slope_limit", 0.35))
        self.iters = int(cfg.get("ransac_iters", 300))
        self.tol = float(cfg.get("ransac_tol", 3.0))
        self.refresh = max(1, int(cfg.get("refresh_frames", 3)))
        self.min_inlier_ratio = float(cfg.get("min_inlier_ratio", 0.25))
        self.expected_y = float(cfg.get("expected_y_ratio", 0.46)) * self.H
        self.max_dev = float(cfg.get("max_dev_ratio", 0.08)) * self.H
        self.track_window = float(cfg.get("track_window_ratio", 0.05)) * self.H
        self.last = None
        self.counter = 0
        self.rng = np.random.RandomState(12345)

    def _invalidate(self):
        self.last = None
        return self._fallback()

    def _fallback(self):
        return RailLine(0.0, self.H * 0.5, False, 0.0)

    def _search_region(self):
        """跟踪阶段只在上一条轨道线附近搜索, 避免跳到桌面/箱子边缘。"""
        if self.last is not None and self.last.valid:
            win = int(self.track_window)
            yl = self.last.y_at(0)
            yr = self.last.y_at(self.W - 1)
            y0 = max(0, int(np.floor(min(yl, yr) - win)))
            y1 = min(self.H, int(np.ceil(max(yl, yr) + win)))
            return y0, y1, win, True
        y0 = int(self.y0_ratio * self.H)
        y1 = int(self.y1_ratio * self.H)
        return y0, y1, 0, False

    def detect(self, gray, force=False):
        """返回 RailLine。gray 为单通道灰度图。"""
        self.counter += 1
        if (self.last is not None and not force and
                (self.counter % self.refresh) != 0):
            return self.last

        H, W = gray.shape[:2]
        y0, y1, win, local = self._search_region()
        x0 = int(self.x_margin * W)
        x1 = int((1.0 - self.x_margin) * W)

        # 只在搜索带内做 Sobel 和逐列 argmax, 避免整幅图的开销
        sub = gray[y0:y1, x0:x1]
        if sub.shape[0] < 3 or sub.shape[1] < 3:
            return self._invalidate()
        sob = cv2.Sobel(sub, cv2.CV_32F, 0, 1, ksize=3)
        step = max(1, W // 400)
        sob = sob[:, ::step]
        cols = x0 + np.arange(sob.shape[1]) * step

        if local and win > 0:
            rows = np.arange(y0, y1, dtype=np.float32)[:, None]
            center = self.last.slope * cols + self.last.intercept
            allow = np.abs(rows - center[None, :]) <= win
            sob = np.where(allow, sob, -1e9)

        idx = np.argmax(sob, axis=0)
        vals = sob.max(axis=0)
        ok = vals > self.grad_thr
        pts = np.column_stack((cols[ok], y0 + idx[ok])).astype(np.float32)
        n = len(pts)

        if n < 60:
            return self._invalidate()

        # 顺序 RANSAC: 若最强直线不在期望高度(例如箱子包边), 剔除其内点
        # 后重新拟合, 直到找到轨道或点集耗尽。
        fit = None
        work = pts
        for _ in range(3):
            if len(work) < 60:
                break
            cand = self._fit(work)
            if cand is None:
                break
            if cand["ratio"] >= self.min_inlier_ratio:
                if local:
                    fit = cand
                    break
                if abs(cand["median_y"] - self.expected_y) <= self.max_dev:
                    fit = cand
                    break
            work = work[~cand["inl"]]

        if fit is None:
            return self._invalidate()

        m, b, ratio = fit["m"], fit["b"], fit["ratio"]
        line = RailLine(m, b, ratio >= self.min_inlier_ratio, min(1.0, ratio))

        if self.last is not None:
            close = (abs(line.slope - self.last.slope) < 0.08 and
                     abs(line.intercept - self.last.intercept) < 25)
            if close and line.valid:
                line.slope = 0.7 * self.last.slope + 0.3 * line.slope
                line.intercept = 0.7 * self.last.intercept + 0.3 * line.intercept
            elif not line.valid:
                return self._invalidate()

        if not line.valid or abs(line.slope) > self.slope_limit:
            return self._invalidate()
        self.last = line
        return line

    def _fit(self, pts):
        """对一组边缘点做一次 RANSAC + 迭代最小二乘。"""
        best = self._ransac(pts)
        if best is None:
            return None
        m, b = best
        inl = np.abs(pts[:, 1] - (m * pts[:, 0] + b)) < self.tol
        for _ in range(3):
            if int(inl.sum()) < 10:
                return None
            m, b = np.polyfit(pts[inl, 0], pts[inl, 1], 1)
            inl = np.abs(pts[:, 1] - (m * pts[:, 0] + b)) < self.tol
        if int(inl.sum()) < 10:
            return None
        return {
            "m": float(m),
            "b": float(b),
            "ratio": float(inl.sum()) / float(len(pts)),
            "median_y": float(np.median(pts[inl, 1])),
            "inl": inl,
        }

    def _ransac(self, pts):
        """批量向量化 RANSAC, 避免 Python 循环逐次算内点。"""
        px = pts[:, 0]
        py = pts[:, 1]
        n = len(pts)
        min_dx = 0.25 * self.W
        best = None
        best_n = 0
        done = 0
        batch = 64
        while done < self.iters:
            k = min(batch, self.iters - done)
            i = self.rng.randint(0, n, k)
            j = self.rng.randint(0, n, k)
            dx = px[j] - px[i]
            ok = (i != j) & (np.abs(dx) >= min_dx)
            if not np.any(ok):
                done += k
                continue
            i = i[ok]
            j = j[ok]
            dx = dx[ok]
            m = (py[j] - py[i]) / dx
            keep = np.abs(m) <= self.slope_limit
            if not np.any(keep):
                done += k
                continue
            m = m[keep]
            i = i[keep]
            b = py[i] - m * px[i]
            resid = np.abs(py[None, :] -
                           (m[:, None] * px[None, :] + b[:, None]))
            counts = (resid < self.tol).sum(axis=1)
            q = int(np.argmax(counts))
            if counts[q] > best_n:
                best_n = int(counts[q])
                best = (float(m[q]), float(b[q]))
            done += k
        return best
