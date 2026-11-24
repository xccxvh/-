"""静态 C 同帧局部提议和完整域竞争复核。只复用模板、索引和采样几何。"""
from collections import OrderedDict

import cv2
import numpy as np

from rail_detector import RailDetector
from ruler_reference import RulerReference
from static_control_reference import DigitUpperEdgeReference


# 每个校正框独立保存最近八个精确姿态；线程池各处理不同框。
_GEOMETRY_CACHE_SIZE = 8


class _LocalMatcher(RulerReference):
    def __init__(self, source):
        # 浅拷贝已验哈希的只读模板及固定参数。姿态、候选和采样缓存属于本实例。
        self.__dict__.update(vars(source))
        self.transform = None
        self.previous = None
        self.windows = {box[:2]: OrderedDict() for box in self.patches}

    def _visible_rectified_search(self, gray, box, matrix, translation):
        key = (matrix.tobytes(), translation.tobytes())
        cache = self.windows[box[:2]]
        cached = cache.get(key)
        if cached is None:
            result = super()._visible_rectified_search(gray, box, matrix, translation)
            if result is None:
                return None
            window, origin, eligible = result
            affine = np.column_stack((matrix, matrix.dot(origin) + translation))
            cache[key] = (affine, window.shape[::-1], origin, eligible)
            if len(cache) > _GEOMETRY_CACHE_SIZE:
                cache.popitem(last=False)
            return result
        cache.move_to_end(key)
        affine, size, origin, eligible = cached
        window = cv2.warpAffine(gray, affine, size, flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        return window, origin, eligible

    def update(self, gray):
        self.transform = None
        expected = np.asarray(self.previous['current_points'])
        local = self._raw_peaks(gray, expected)
        if local is not None:
            points, scores, margins = local
            if self.previous.get('rotation_rechecked'):
                # 三个实测局部峰提议本帧姿态，复核仍在完整可见域使用原模板。
                result = self._confirm_rotation(gray, points, scores)
            elif min(scores) >= self.min_score and min(margins) >= self.min_margin:
                result = self._match(gray)
            else:
                return {'valid': False, 'reason': 'weak_local'}
            if result['valid'] and np.max(np.abs(np.asarray(result['current_points'])-expected)) <= 12:
                result['registration_mode'] = 'local_track'
                return result
        self.transform = None
        return {'valid': False, 'reason': 'relocalize'}


class _EdgeSampler:
    """对同一姿态只生成一次采样坐标；梯度、有效列和中值修正全部来自当前帧。"""

    def __init__(self, edge):
        self.profile = edge.profile
        self.radius = edge.max_adjust
        self.margin = edge.support_margin
        self.plans = OrderedDict()

    def _prepare(self, shape, inverse, shift):
        points = self.profile.dot(np.linalg.inv(inverse).T) + shift
        xs = np.arange(shape[1])
        expected = np.interp(xs, points[:, 0], points[:, 1])
        for a, b, mask in [(points[0], points[1], xs < points[0, 0]),
                           (points[-2], points[-1], xs > points[-1, 0])]:
            expected[mask] = a[1] + (xs[mask]-a[0])*(b[1]-a[1])/(b[0]-a[0])
        lo = max(1, int(expected.min()-self.radius-2))
        hi = min(shape[0]-1, int(expected.max()+self.radius+3))
        if hi-lo < 3:
            return None
        # 与原算法的逐列 ±radius 条件相同。短轴连续排列，避免在整幅宽梯度带上 argmax。
        first = np.ceil(expected-self.radius).astype(int)
        ys = first[:, None] + np.arange(int(np.ceil(2*self.radius))+1)
        valid = (np.abs(ys-expected[:, None]) <= self.radius) & (ys >= lo) & (ys < hi)
        lut = np.empty(ys.shape+(2,), dtype=np.int16)
        lut[:, :, 0] = xs[:, None]
        lut[:, :, 1] = np.where(valid, ys-lo, -1)
        domain = (xs >= points[0, 0]) & (xs <= points[-1, 0])
        support_domain = (xs >= points[0, 0]-self.margin) & (xs <= points[-1, 0]+self.margin)
        return lut, lo, hi, expected, xs, first, domain, support_domain

    def measure(self, gray, transform, registration):
        inverse, shift = transform
        key = (gray.shape, inverse.tobytes(), shift.tobytes())
        # 仅复用精确矩阵和尺寸对应的列索引；下面所有梯度和支持判定均重算。
        plan = self.plans.get(key)
        if plan is None:
            plan = self._prepare(gray.shape, inverse, shift)
            if plan is None:
                return None
            self.plans[key] = plan
            if len(self.plans) > _GEOMETRY_CACHE_SIZE:
                self.plans.popitem(last=False)
        else:
            self.plans.move_to_end(key)
        lut, lo, hi, expected, xs, first, domain, support_domain = plan
        grad = cv2.Sobel(gray[lo-1:hi+1], cv2.CV_32F, 0, 1, ksize=3)[1:-1]
        samples = cv2.remap(grad, lut, None, cv2.INTER_NEAREST, borderValue=-1e6)
        best = samples.argmax(axis=1)
        strong = samples[xs, best] > 35
        good = domain & strong
        if not good.any() or good.sum() < .55*domain.sum():
            return None
        support = support_domain & strong
        delta = np.clip(first+best-expected, -self.radius, self.radius).astype(np.float32)
        delta[~support] = 0
        levels = np.clip(128+4*delta, 0, 255).astype(np.uint8)
        # 151 列复制边界中值等价于各灰度门槛过半的总数，只计算实际存在的量化范围。
        low, high = int(levels.min()), int(levels.max())
        if low != high:
            planes = (levels >= np.arange(low+1, high+1)[:, None]).astype(np.uint8)
            counts = cv2.boxFilter(planes, -1, (151, 1), normalize=False, borderType=cv2.BORDER_REPLICATE)
            levels = low + (counts >= 76).sum(axis=0)
        edge = expected + (levels.astype(float)-128)/4.
        slope, intercept = np.polyfit(xs[good], edge[good], 1)
        return dict(slope=float(slope), intercept=float(intercept), confidence=float(good.sum()/domain.sum()),
                    edge_profile=edge, edge_support=support, reference_inverse=inverse, reference_translation=shift,
                    reference_profile=self.profile, registration=registration)


class FastUpperEdgeReference:
    def __init__(self, cfg):
        self.cold = DigitUpperEdgeReference(cfg)
        self.tracks = {}
        for name, bank in [('primary', self.cold.matcher), ('fallback', self.cold.other_matcher),
                           ('digit', self.cold._digits.matcher)]:
            if bank is not None:
                self.tracks[name] = _LocalMatcher(bank)
        self.edge = _EdgeSampler(self.cold)
        self.last = None
        self.reason = 'not_updated'

    def locate(self, gray):
        last, self.last = self.last, None
        if last is not None and gray is not None and gray.shape == self.cold.matcher.shape and gray.dtype == np.uint8:
            group = last['reference_set']
            matcher = self.tracks[group]
            matcher.previous = last
            registration = matcher.update(gray)
            evidence = self.edge.measure(gray, matcher.transform, registration) if registration['valid'] else None
            if evidence is not None:
                # 数字或备用组不能利用上帧的优先权绕过旧模板的强竞争。
                banks = ()
                if group == 'digit':
                    banks = (self.cold.matcher, self.cold.other_matcher)
                elif group == 'fallback':
                    banks = (self.cold.matcher if self.cold.reference_set=='primary' else self.cold.other_matcher,)
                for bank in banks:
                    if bank is None:
                        continue
                    _, scores, margins = bank._raw_peaks(gray)
                    if any(s >= bank.min_score and m < bank.min_margin for s,m in zip(scores,margins)):
                        self.reason = 'reference_competing_patch'
                        return None
                self.last = evidence['registration']
                self.last.update(reference_set=group, registration_mode=group+'/local_track')
                self.reason = 'ok'
                return evidence
        # 冷启动或局部失败当帧重定位，严禁复用上一帧的结果。
        self.cold.matcher.transform = None
        evidence = self.cold.locate(gray)
        self.reason = self.cold.reason
        if evidence is not None:
            self.last = evidence['registration']
        return evidence


def make_rail_detector(rail_cfg, width, height):
    cfg = rail_cfg.get('upper_reference', {})
    if not (cfg.get('fast_search') is True and cfg.get('enabled') is True and
            cfg.get('digit_boxes') and cfg.get('adaptive_search') is True and cfg.get('rotation_recheck') is True):
        raise ValueError('fast reference requires enabled, fast_search, adaptive_search, rotation_recheck and digit_boxes')
    rd = RailDetector(dict(rail_cfg, upper_reference={}), width, height)
    rd.upper_reference = FastUpperEdgeReference(cfg)
    return rd
