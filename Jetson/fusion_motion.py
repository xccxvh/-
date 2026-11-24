# -*- coding: utf-8 -*-
"""纸尺配准后的帧差只提议搜索位置；球心由当前灰度图重新测量。"""
from collections import namedtuple
from numbers import Real

import cv2
import numpy as np

from ball_detector import BallCandidate
from local_vision import _motion_glint_seeds, _verify_topview_circles, topview_balls


_Observation = namedtuple('_Observation', 'patch origin inverse translation time shape')
_angles = np.arange(36) * (2. * np.pi / 36)
_directions = np.column_stack((np.cos(_angles), np.sin(_angles)))


def _observation(gray, rail, bounds, timestamp):
    if (not getattr(rail, 'valid', False) or not getattr(rail, 'identity_valid', False)
            or not isinstance(gray, np.ndarray) or gray.ndim != 2 or gray.dtype != np.uint8
            or isinstance(timestamp, bool) or not isinstance(timestamp, Real)
            or not np.isfinite(timestamp)):
        return None
    height, width = gray.shape
    inverse = np.asarray(getattr(rail, 'reference_inverse', None), dtype=float)
    translation = np.asarray(getattr(rail, 'reference_translation', None), dtype=float)
    edge = np.asarray(getattr(rail, 'edge_profile', None))
    limits = np.asarray(bounds, dtype=float)
    if (inverse.shape != (2, 2) or not np.isfinite(inverse).all()
            or np.linalg.det(inverse) <= 1e-8 or translation.shape != (2,)
            or not np.isfinite(translation).all() or edge.shape != (width,)
            or np.shape(getattr(rail, 'edge_support', None)) != (width,)
            or limits.shape != (2,) or not np.isfinite(limits).all()
            or limits[0] >= limits[1]):
        return None
    # 两侧保留完整球的取证余量；允许中心的物理范围仍由原始 bounds 限制。
    left = max(0, int(np.floor(limits[0])) - 42)
    right = min(width, int(np.ceil(limits[1])) + 43)
    if right <= left or not np.isfinite(edge[left:right]).all():
        return None
    top = max(0, int(np.floor(edge[left:right].min())) - 82)
    bottom = min(height, int(np.ceil(edge[left:right].max())) + 21)
    if bottom - top < 38:
        return None
    return _Observation(gray[top:bottom, left:right].copy(), np.array([left, top]),
                        inverse.copy(), translation.copy(), float(timestamp), gray.shape)


def _arriving_rim(ball, patch, aligned, origin):
    # 对应同一纸尺位置，在当前拟合圆的内外两侧量灰度差。消失的旧球也会产生
    # 帧差，但只有当前凸圆周新出现的正向对比才能支持候选，均匀曝光变化会抵消。
    center = np.array([ball.x, ball.extra['measurement_y_px']]) - origin
    radius = ball.width * .5
    inner = np.rint(center + (radius - 4) * _directions).astype(int)
    outer = np.rint(center + (radius + 4) * _directions).astype(int)
    h, w = patch.shape
    if any((p < 0).any() or (p[:, 0] >= w).any() or (p[:, 1] >= h).any()
           for p in (inner, outer)):
        return False
    u, v = inner.T
    x, y = outer.T
    before = aligned[y, x] - aligned[v, u]
    after = patch[y, x].astype(np.int16) - patch[v, u]
    good = ((aligned[y, x] >= 0) & (aligned[v, u] >= 0) & (_directions[:, 1] < .75)
            & (after >= 16) & (after - before >= 14))
    return np.count_nonzero(good) >= 5


class MotionCandidates(object):
    """接收按采集时刻（秒）排序的灰度帧，仅缓存上一张有效纸尺窄带。"""

    def __init__(self):
        self.previous = None

    def update(self, gray, rail, bounds, timestamp, propose=True):
        previous = self.previous
        current = _observation(gray, rail, bounds, timestamp)
        self.previous = current
        if (not propose or previous is None or current is None or previous.shape != current.shape
                or current.time - previous.time > .15):
            return []

        if current.time <= previous.time:
            self.previous = None
            return []

        # 全图坐标先由旧帧逆变换回到尺坐标，再映到当帧。加上两次截取的
        # 不同原点后，warpAffine 可直接在窄带内工作。无重叠区不能形成帧差。
        matrix = np.linalg.solve(current.inverse, previous.inverse)
        offset = (matrix.dot(previous.origin - previous.translation)
                  + current.translation - current.origin)
        affine = np.column_stack((matrix, offset))
        patch = current.patch
        aligned = cv2.warpAffine(previous.patch.astype(np.float32), affine, patch.shape[::-1],
                                 borderValue=-65536.)
        left, top = current.origin
        edge = rail.edge_profile[left:left+patch.shape[1]] - top
        depth = edge - np.arange(patch.shape[0])[:, None]
        # 球是带强高光的暗色球面。取变暗的入场一侧提案，忽略轨唇和孤立噪点。
        changed = ((aligned - patch >= 22) & (depth >= 4) & (depth <= 60))
        columns = (changed.sum(axis=0) >= 4).astype(np.uint8)[None, :]
        columns = cv2.morphologyEx(columns, cv2.MORPH_CLOSE, np.ones((1, 7), np.uint8))
        ends = np.flatnonzero(np.diff(np.pad(columns[0].astype(int), (1, 1), mode='constant')))
        hits = []
        for start, stop in zip(ends[::2], ends[1::2]):
            if not 4 <= stop - start <= 95:
                continue
            lo = max(bounds[0], left + start - 26)
            hi = min(bounds[1], left + stop + 25)
            if lo >= hi:
                continue
            # 帧差条带可能偏向球的前沿。只将其用于限定当帧的局部搜索；
            # 复用原有的球周、双侧尺沿支撑和球内亮斑对比度检验以及版本字段。
            circles, _ = topview_balls(gray, rail, (lo, hi), BallCandidate, [])
            if not circles:
                a = max(left, int(np.floor(lo)) - 40)
                b = min(left + patch.shape[1], int(np.ceil(hi)) + 41)
                local = cv2.GaussianBlur(patch[:, a-left:b-left], (5, 5), 1.2)
                seeds = _motion_glint_seeds(local, rail.edge_profile[a:b] - top)
                verified = _verify_topview_circles(local, seeds, a, top, rail, (lo, hi), BallCandidate)
                circles = [ball for ball, _ in verified]
            for ball in circles:
                if _arriving_rim(ball, patch, aligned, current.origin):
                    hits.append(ball)

        # 同一物体可能触发多条变化带，合并重复测量，分开的真实候选交给跟踪器。
        result = []
        for ball in sorted(hits, key=lambda c: c.confidence, reverse=True):
            if all(np.hypot(ball.x - other.x, ball.extra['measurement_y_px']
                            - other.extra['measurement_y_px']) >= 24 for other in result):
                ball.extra['proposal_source'] = 'motion'
                result.append(ball)
        return result
