# -*- coding: utf-8 -*-
"""方案 C 专用轨道坐标。沿用原球心测量、同帧参考、投影与厘米验收。"""
import hashlib
from pathlib import Path

from rail_calibration import RailCalibration, binding_id
from rail_coordinates import RailCoordinates


class StaticControlCoordinates(RailCoordinates):
    """把实际候选选择路径绑定在原有八源算法及相机、参考和轴定义之上。"""

    def __init__(self, cfg, geo, base_dir=None):
        if cfg.get('static_control_motion') is not True:
            raise ValueError('static_control_motion must be true')
        # 父类建立真实基础绑定；推迟模型加载，以免用基础绑定读取方案 C 模型。
        rail_cfg = cfg['rail_coordinates']
        super().__init__(dict(cfg, rail_coordinates=dict(rail_cfg, calibration_file=None)),
                         geo, base_dir)
        self.cfg = rail_cfg
        if not self.enabled:
            return
        if self.method != 'topview_circle':
            raise ValueError('static control requires topview_circle measurements')

        # 保存实际基础 ID 和实际实现文件摘要；部署目录名不参与绑定。
        # 增量字段参与原加载器的完整 binding 比对。缺文件由 read_bytes 直接报错。
        root = Path(__file__).resolve().parent
        names = ('fusion_motion.py', 'static_control_detection.py', 'static_control_geometry.py')
        self.binding['static_control'] = dict(
            base_binding_id=self.binding_id, static_control_motion=cfg['static_control_motion'],
            sources={name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in names})
        self.binding_id = binding_id(self.binding)
        model = rail_cfg.get('calibration_file')
        if model:
            self.calibration = RailCalibration(str(self.root / model), self.binding)
