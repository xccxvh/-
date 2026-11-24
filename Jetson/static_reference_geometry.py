# -*- coding: utf-8 -*-
"""数字尺备用参考的正式厘米坐标，继承方案 C 的球心定义和同帧投影。"""
import hashlib
from pathlib import Path

from rail_calibration import RailCalibration, binding_id
from static_control_geometry import StaticControlCoordinates


class StaticReferenceCoordinates(StaticControlCoordinates):
    """先计算含数字模板配置的 C 绑定，再绑定实际运行的参考实现。"""

    def __init__(self, cfg, geo, base_dir=None):
        if not cfg['rail']['upper_reference'].get('digit_boxes'):
            raise ValueError('digit reference requires digit_boxes')
        # 推迟加载：父类只建立真实 C 绑定，此时尚未纳入两个新增源文件。
        rail_cfg = cfg['rail_coordinates']
        super().__init__(dict(cfg, rail_coordinates=dict(rail_cfg, calibration_file=None)),
                         geo, base_dir)
        self.cfg = rail_cfg
        if not self.enabled:
            return
        root = Path(__file__).resolve().parent
        names = ('static_control_reference.py', 'static_reference_geometry.py')
        self.binding['static_reference'] = dict(
            base_binding_id=self.binding_id,
            sources={name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in names})
        self.binding_id = binding_id(self.binding)
        # 回放采样允许空模型；部署时由原加载器校验整份绑定并重新拟合内嵌证据。
        path = rail_cfg.get('calibration_file')
        if path:
            self.calibration = RailCalibration(str(self.root / path), self.binding)
