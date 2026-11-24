"""把实测快搜路径及源码绑定到独立厘米模型。"""
import hashlib
from pathlib import Path

from rail_calibration import RailCalibration, binding_id
from static_reference_geometry import StaticReferenceCoordinates


class StaticFastCoordinates(StaticReferenceCoordinates):
    def __init__(self, cfg, geo, base_dir=None):
        if cfg['rail']['upper_reference'].get('fast_search') is not True:
            raise ValueError('fast reference not enabled')
        rail_cfg = cfg['rail_coordinates']
        super().__init__(dict(cfg, rail_coordinates=dict(rail_cfg, calibration_file=None)), geo, base_dir)
        self.cfg = rail_cfg
        if not self.enabled:
            return
        root = Path(__file__).resolve().parent
        names = ('static_fast_reference.py', 'static_fast_geometry.py')
        self.binding['static_fast_reference'] = dict(
            base_binding_id=self.binding_id,
            sources={name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in names})
        self.binding_id = binding_id(self.binding)
        # 只有最终完整绑定可以加载模型；空路径保留正常离线采样行为。
        path = rail_cfg.get('calibration_file')
        if path:
            self.calibration = RailCalibration(str(self.root/path), self.binding)
