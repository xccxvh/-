# -*- coding: utf-8 -*-
"""验证 C 配置及正式模型，复核基础、新路径与独立摆放证据的边界。"""
import hashlib
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from camera_geometry import CameraGeometry
from rail_coordinates import RailCoordinates
from static_control_geometry import StaticControlCoordinates


ROOT = Path(__file__).resolve().parents[1]


class StaticControlGeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT/'config_static_control_c.json').read_text())
        self.old = json.loads((ROOT/'config_motion_cm_20261122.json').read_text())
        self.geo = CameraGeometry(self.cfg['camera_geometry'], 1280, 720, ROOT)

    def test_dedicated_model_and_inherited_base(self):
        current = StaticControlCoordinates(self.cfg, self.geo, ROOT)
        old = RailCoordinates(self.old, self.geo, ROOT)
        self.assertEqual(self.cfg['ball'], self.old['ball'])
        self.assertTrue(current.calibration.validated)
        self.assertTrue(old.calibration.validated)
        self.assertNotEqual(current.binding_id, old.binding_id)
        base = dict(current.binding)
        extension = base.pop('static_control')
        self.assertEqual(base, old.binding)
        self.assertEqual(extension['base_binding_id'], old.binding_id)
        for name in ('static_control_geometry.py','static_control_detection.py','fusion_motion.py'):
            self.assertEqual(extension['sources'][name], hashlib.sha256((ROOT/name).read_bytes()).hexdigest())
        # 旧模型无法对 C 实现提供厘米，反向误配同样拒绝。
        model_path = self.cfg['rail_coordinates']['calibration_file']
        self.cfg['rail_coordinates']['calibration_file'] = self.old['rail_coordinates']['calibration_file']
        self.assertFalse(StaticControlCoordinates(self.cfg, self.geo, ROOT).calibration.ready)
        self.old['rail_coordinates']['calibration_file'] = model_path
        self.assertFalse(RailCoordinates(self.old, self.geo, ROOT).calibration.ready)

    def test_configuration_and_actual_source_drift_close_cm(self):
        self.cfg['static_control_motion'] = False
        with self.assertRaises(ValueError):
            StaticControlCoordinates(self.cfg, self.geo, ROOT)
        self.cfg['static_control_motion'] = True
        # 文件内容漂移仅在内存中模拟，不改动任何正式源或保护清单。
        read = Path.read_bytes
        def changed(path):
            data = read(path)
            return data+b'\n' if path.name == 'static_control_detection.py' else data
        with patch.object(Path, 'read_bytes', changed):
            self.assertFalse(StaticControlCoordinates(self.cfg, self.geo, ROOT).calibration.validated)
        self.cfg['tracking']['acquire_frames'] += 1
        self.assertFalse(StaticControlCoordinates(self.cfg, self.geo, ROOT).calibration.validated)
