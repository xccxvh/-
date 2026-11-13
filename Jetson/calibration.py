# -*- coding: utf-8 -*-
"""Fail-closed pixel calibration. Model residuals are NOT independent accuracy tests."""
import json
import os
import tempfile
import numpy as np
from measurement_contract import COORDINATE_VERSION


class Calibration(object):
    def __init__(self, path):
        self.path = path
        self.measurement_version = COORDINATE_VERSION
        self.points = []
        self.model = None
        self.stats = {}
        self.resolution = None
        self.ready = False
        self.load()

    def _arrays(self):
        if self.measurement_version != COORDINATE_VERSION:
            raise ValueError("坐标算法版本变化，请从原始录像重新采集到新标定文件")
        if self.resolution is None or min(self.resolution) <= 0:
            raise ValueError('缺少有效采集分辨率')
        if any(type(p[k]) not in (int,float) for p in self.points for k in ('x_cm','x_px')):
            raise ValueError('标定点必须为 JSON 数字')
        pts = sorted(self.points, key=lambda p: p['x_cm'])
        xs = np.array([p['x_px'] for p in pts], dtype=float)
        ys = np.array([p['x_cm'] for p in pts], dtype=float)
        if len(pts) < 3 or not all(any(abs(ys-v) < 1e-6) for v in (-5,0,5)):
            raise ValueError('需要 -5、0、+5 cm 三个已知位置')
        if not np.all(np.isfinite(xs)) or not np.all(np.isfinite(ys)):
            raise ValueError('标定点含非有限数值')
        if np.any(xs < 0) or np.any(xs >= self.resolution[0]):
            raise ValueError('像素坐标超出画面')
        dx = np.diff(xs)
        if not np.all(np.diff(ys) > 0) or not (np.all(dx > 1e-6) or np.all(dx < -1e-6)):
            raise ValueError('标定点必须严格单调且不能重复')
        return xs, ys

    def load(self):
        try:
            with open(self.path, 'r') as f:
                data = json.load(f)
            if data.get('version',2) != 2 or data.get('coordinate','image_x') != 'image_x':
                return
            self.measurement_version = data.get('measurement_version')
            self.points = data.get('points', [])
            self.stats = data.get('stats', {})
            res = data.get('resolution')
            if res is not None and len(res) == 2:
                self.resolution = tuple(int(v) for v in res)
            model = data.get('model')
            if model is None:
                return
            xs, ys = self._arrays()
            if model['type'] == 'linear':
                if any(type(model[k]) not in (int,float) for k in ('a','b')):
                    return
                a, b = float(model['a']), float(model['b'])
                if not np.isfinite(a) or not np.isfinite(b) or a == 0:
                    return
                expected = np.polyfit(xs, ys, 1)
                if not np.allclose([a,b], expected, rtol=1e-7, atol=1e-7):
                    return
            elif model['type'] == 'piecewise':
                order = np.argsort(xs)
                if not np.array_equal(model['xs'], xs[order]) or not np.array_equal(model['ys'], ys[order]):
                    return
            else:
                return
            self.model = model
            self.ready = True
        except (OSError, ValueError, TypeError, KeyError, AttributeError, OverflowError):
            self.ready = False
            self.model = None
            # Malformed data must not poison a subsequent add operation.
            if not isinstance(self.points, list) or any(not isinstance(p, dict) or 'x_cm' not in p or 'x_px' not in p for p in self.points):
                self.points = []

    def save(self):
        data = {'version':2, 'coordinate':'image_x',
                'measurement_version':self.measurement_version, 'points':self.points,
                'model':self.model, 'stats':self.stats,
                'resolution':list(self.resolution) if self.resolution else None}
        parent = os.path.dirname(os.path.abspath(self.path))
        fd, tmp = tempfile.mkstemp(prefix='.calibration-', dir=parent)
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(data, f, indent=2, ensure_ascii=False, allow_nan=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
        finally:
            if os.path.exists(tmp):
                os.unlink(tmp)

    def resolution_ok(self, width, height):
        return self.resolution is not None and (width, height) == self.resolution

    def add_point(self, x_cm, x_px, width, height, evidence=None):
        if self.measurement_version != COORDINATE_VERSION:
            raise ValueError("不能混用旧坐标算法标定，请使用新文件")
        x_cm, x_px = float(x_cm), float(x_px)
        res = (int(width), int(height))
        if not np.isfinite(x_cm) or not np.isfinite(x_px) or min(res) <= 0 or not 0 <= x_px < res[0]:
            raise ValueError('无效标定点或分辨率')
        if self.resolution is not None and self.resolution != res:
            raise ValueError('分辨率与原标定不一致，请使用新标定文件')
        self.resolution = res
        self.points = [p for p in self.points if abs(p['x_cm']-x_cm) > 1e-6]
        point = {'x_cm':x_cm, 'x_px':x_px}
        if evidence is not None:
            point['evidence'] = evidence
        self.points.append(point)
        self.points.sort(key=lambda p: p['x_cm'])
        self.model, self.ready, self.stats = None, False, {}
        self.save()

    def fit(self, max_linear_error_cm=0.15):
        self.ready, self.model = False, None
        try:
            xs, ys = self._arrays()
            if not np.isfinite(max_linear_error_cm) or max_linear_error_cm < 0:
                raise ValueError('无效拟合阈值')
        except (ValueError, KeyError, TypeError) as exc:
            return False, str(exc)
        a,b = np.polyfit(xs, ys, 1)
        residuals = ys-(a*xs+b)
        max_err = float(np.max(np.abs(residuals)))
        if max_err > max_linear_error_cm:
            order = np.argsort(xs)
            self.model = {'type':'piecewise', 'xs':xs[order].tolist(), 'ys':ys[order].tolist()}
        else:
            self.model = {'type':'linear','a':float(a),'b':float(b)}
        self.stats = {'linear_residuals_cm':residuals.tolist(),
                      'max_abs_linear_residual_cm':max_err, 'n_points':len(xs)}
        self.ready = True
        self.save()
        return True, self.stats

    def px_to_cm(self, x_px, width=None, height=None):
        if not self.ready or not self.resolution_ok(width,height) or x_px is None:
            return None
        if not np.isfinite(x_px) or self.extrapolated(x_px):
            return None
        if self.model['type'] == 'linear':
            return float(self.model['a']*x_px+self.model['b'])
        return float(np.interp(x_px, self.model['xs'], self.model['ys']))

    def extrapolated(self, x_px):
        if not self.ready or x_px is None:
            return False
        xs = [p['x_px'] for p in self.points]
        return not np.isfinite(x_px) or x_px < min(xs) or x_px > max(xs)

    def pose_ok(self, rail, max_displacement_px=3.0):
        """Conservative static-pose gate, NOT dynamic coordinate correction.

        A line cannot detect along-rail camera translation or focal changes.
        """
        if not self.ready or not rail.valid:
            return False
        for p in self.points:
            ev = p.get('evidence', {})
            try:
                dm = getattr(rail,'raw_slope',rail.slope)-float(ev['rail_slope'])
                db = getattr(rail,'raw_intercept',rail.intercept)-float(ev['rail_intercept'])
                errors = [abs(dm*x+db) for x in (0,self.resolution[0]-1)]
                if not all(np.isfinite(e) and e <= max_displacement_px for e in errors):
                    return False
            except (KeyError, TypeError, ValueError):
                return False
        return True
