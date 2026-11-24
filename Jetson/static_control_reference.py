# -*- coding: utf-8 -*-
"""静态 C 数字纹理复核；沿用原图、原配准和上缘测量，独立于历史绑定源码。"""
from local_vision import UpperEdgeReference
from rail_detector import RailDetector


class DigitUpperEdgeReference(UpperEdgeReference):
    """旧主/备用模板每帧优先。仅在配置了 digit_boxes 时启用第三组同帧测量。"""

    def __init__(self, cfg):
        super().__init__(cfg)
        self._digits = None
        if 'digit_boxes' not in cfg:
            return

        # 每个数字小窗来自对应的一个原主框；避免错序、浮点截断或跨到尺外。
        boxes = cfg['digit_boxes']
        if not isinstance(boxes, (list, tuple)) or len(boxes) != 3 or len(cfg['boxes']) != 3:
            raise ValueError('digit_boxes requires three primary subboxes')
        for box, primary in zip(boxes, cfg['boxes']):
            if (not isinstance(box, (list, tuple)) or len(box) != 4 or
                    any(type(v) is not int for v in box)):
                raise ValueError('digit_boxes requires integer rectangles')
            x, y, w, h = box
            px, py, pw, ph = primary
            if not (px <= x and py <= y and x+w <= px+pw and y+h <= py+ph):
                raise ValueError('digit box outside corresponding primary patch')

        # 使用相同哈希、搜索域、纹理门限和上缘剖面。数字组不再递归尝试旧备用组。
        digit_cfg = dict(cfg, boxes=boxes)
        digit_cfg.pop('fallback_boxes', None)
        self._digits = UpperEdgeReference(digit_cfg)

    def locate(self, gray):
        evidence = super().locate(gray)
        if (evidence is not None or self._digits is None or
                not self.reason.startswith('reference_rotation_')):
            return evidence

        # 与离板试验相同，只对旧旋转配准失败做复核。丢弃上一数字姿态，
        # 让本帧在完整域内独立定位；成败均不切换原主/备用模板的优先级。
        digit_matcher = self._digits.matcher
        digit_matcher.transform = None
        digit_matcher._last_rectified = False
        evidence = self._digits.locate(gray)
        if evidence is None:
            self.reason = self._digits.reason
            return None

        # 发布数字结果前，在两套旧模板的完整搜索域检查强竞争峰。
        # 任一旧框过分数门限且峰差不足，即使三个数字相合也保持失效。
        for matcher in (self.matcher, self.other_matcher):
            if matcher is None:
                continue
            _, scores, margins = matcher._raw_peaks(gray)
            if any(s >= matcher.min_score and m < matcher.min_margin
                   for s, m in zip(scores, margins)):
                digit_matcher.transform = None
                self.reason = 'reference_competing_patch'
                return None

        registration = evidence['registration']
        registration['reference_set'] = 'digit'
        registration['registration_mode'] = 'digit/' + registration.get('registration_mode', 'registered')
        self.reason = 'ok'
        return evidence


def make_rail_detector(rail_cfg, width, height):
    """正式入口和离线回放共用，要求显式开启参考和数字复核。"""
    cfg = rail_cfg.get('upper_reference')
    if not isinstance(cfg, dict) or cfg.get('enabled') is not True or 'digit_boxes' not in cfg:
        raise ValueError('digit reference requires enabled=True and digit_boxes')
    # 基础检测器只负责轨道状态和序号，由专用类一次性创建三组纹理。
    detector = RailDetector(dict(rail_cfg, upper_reference={}), width, height)
    detector.upper_reference = DigitUpperEdgeReference(cfg)
    return detector
