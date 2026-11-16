# -*- coding: utf-8 -*-
"""新轨道坐标厘米标定；完全独立于历史Calibration及其文件。

RailCalibration(path, binding)只读加载。ready表示拟合结构可用，validated表示
独立实物位置验证通过；position还要求当前binding完全相符且s在拟合区间内。
模型内嵌采样证据，加载时重新拟合/验证，不把训练残差当作验证。
Python3.6/NumPy兼容。证据哈希用于追溯，不能证明操作者确认的实物位置真实。
"""
import copy
import hashlib
import json
import math
from numbers import Real
import re

import numpy as np


# 拟合只规定每个已知位置的最低有效样本数；coverage仍完整报告。
FIT_MIN_SAMPLES = 20
MIN_SAMPLES = 20
MIN_COVERAGE = .95
MAX_ABS_CM = .3


def canonical(value):
    """与父模块统一UTF-8规范JSON；保留数值类型差异，拒绝NaN和非JSON对象。"""
    def check(v):
        if v is None or type(v) in (str, bool, int):
            return
        if type(v) is float and math.isfinite(v):
            return
        if type(v) is list:
            for item in v:
                check(item)
            return
        if type(v) is dict and all(type(k) is str for k in v):
            for item in v.values():
                check(item)
            return
        raise ValueError('binding/evidence must contain finite JSON values only')
    check(value)
    return json.dumps(value, sort_keys=True, ensure_ascii=False, separators=(',', ':'), allow_nan=False)


def is_number(value):
    return isinstance(value, Real) and not isinstance(value, (bool, np.bool_)) and math.isfinite(value)


def is_sha(value):
    return isinstance(value, str) and re.match(r'^[0-9a-f]{64}$', value) is not None


def validate_binding(binding):
    """整个dict绑定；未知附加字段也进入ID，不能局部匹配忽略几何/算法变化。"""
    canonical(binding)
    if not isinstance(binding, dict) or binding.get('schema') != 'rail-binding-v1':
        raise ValueError('expected rail-binding-v1')
    if binding.get('measurement_version') != 'relative-cap-v1' or binding.get('rail_coordinate_version') != 'rail-reference-v1':
        raise ValueError('unsupported rail measurement coordinate version')
    if not isinstance(binding.get('geometry'), dict) or not binding['geometry']:
        raise ValueError('binding requires nonempty geometry JSON object')
    for key in ('reference_image_sha256', 'reference_config_sha256', 'algorithm_sha256'):
        if not is_sha(binding.get(key)):
            raise ValueError('invalid binding SHA256: '+key)
    return binding


def binding_id(binding):
    """父任务可用同一函数生成每帧rail_binding_id。"""
    validate_binding(binding)
    return hashlib.sha256(canonical(binding).encode('utf-8')).hexdigest()


def session_values(session, binding):
    """重建有效样本；coverage分母永远是所确认完整帧区间，缺帧不能消失。"""
    bid = binding_id(binding)
    if session.get('schema') != 'rail-sampling-session-v1' or session.get('known_position') is not True:
        raise ValueError('session requires explicit known-position confirmation')
    if canonical(session.get('binding')) != canonical(binding) or session.get('rail_binding_id') != bid:
        raise ValueError('session binding mismatch')
    cm, start, end = session.get('cm'), session.get('start_frame'), session.get('end_frame')
    if not is_number(cm) or type(start) is not int or type(end) is not int or not 0 <= start <= end:
        raise ValueError('invalid known cm or inclusive frame interval')
    source = session.get('source', {})
    for key in ('video_sha256', 'csv_sha256', 'binding_sha256'):
        if not is_sha(source.get(key)):
            raise ValueError('missing evidence digest: '+key)
    for key in ('video_path', 'csv_path', 'binding_path'):
        if not isinstance(source.get(key), str) or not source[key]:
            raise ValueError('missing evidence path: '+key)
    contract = session.get('prediction_contract')
    if contract not in ('column', 'pixel_valid'):
        raise ValueError('missing prediction exclusion contract')
    rows = session.get('rows')
    if not isinstance(rows, list):
        raise ValueError('session rows must be a list')
    seen, values = set(), []
    for row in rows:
        frame = row.get('frame')
        if type(frame) is not int or not start <= frame <= end or frame in seen:
            raise ValueError('duplicate or out-of-interval evidence frame')
        seen.add(frame)
        for key in ('pixel_valid', 'reference_valid', 'calibration_valid', 'cm_valid'):
            if type(row.get(key)) is not bool:
                raise ValueError('evidence flags must be boolean')
        predicted = row.get('predicted')
        if (contract == 'column' and type(predicted) is not bool) or (contract == 'pixel_valid' and predicted is not None):
            raise ValueError('prediction evidence contradicts declared contract')
        row_id = row.get('rail_binding_id')
        if row_id not in (None, '', bid):
            raise ValueError('mixed binding in evidence')
        s = row.get('rail_s_px')
        if s is not None and not is_number(s):
            raise ValueError('rail_s_px must be finite numeric or null')
        if row['pixel_valid'] and row['reference_valid'] and predicted is not True and row_id == bid and s is not None:
            values.append(float(s))
    return dict(cm=float(cm), values=values, samples=len(values), total_frames=end-start+1,
                coverage=len(values)/float(end-start+1))


def independent(sessions):
    """所有采样区间均去重；用内容哈希识别同视频，不被重命名绕过。"""
    for i, a in enumerate(sessions):
        for b in sessions[:i]:
            sa, sb = a['source'], b['source']
            same = (sa['video_sha256'] == sb['video_sha256'] or sa['csv_sha256'] == sb['csv_sha256'] or
                    sa['video_path'] == sb['video_path'])
            if same and max(a['start_frame'], b['start_frame']) <= min(a['end_frame'], b['end_frame']):
                raise ValueError('sampling intervals overlap/reuse evidence')


def grouped(sessions, binding):
    groups = {}
    for session in sessions:
        info = session_values(session, binding)
        group = groups.setdefault(info['cm'], dict(cm=info['cm'], values=[], total_frames=0))
        group['values'].extend(info['values'])
        group['total_frames'] += info['total_frames']
    return [groups[k] for k in sorted(groups)]


def fitted(fit_sessions, binding, method):
    """每个实物位置以中位s拟合，位置等权；方向可反转但必须严格单调。"""
    groups = grouped(fit_sessions, binding)
    cms = np.asarray([g['cm'] for g in groups], dtype=float)
    if len(groups) < 3 or not all(np.any(np.abs(cms-cm) <= 1e-9) for cm in (-5., 0., 5.)):
        raise ValueError('fit requires at least -5, 0, +5 cm positions')
    if not np.all(np.diff(cms) > 1e-6):
        raise ValueError('fit needs distinct positions')
    # 位置内合并已检查互不重叠的session；无效/预测/缺参考行不能充数。
    # 95%覆盖率只属于独立验收，不删除拟合失败帧或按覆盖率筛掉位置。
    for group in groups:
        if len(group['values']) < FIT_MIN_SAMPLES:
            raise ValueError('fit position %g cm requires at least %d valid samples (got %d)' %
                             (group['cm'], FIT_MIN_SAMPLES, len(group['values'])))
    xs = np.asarray([np.median(g['values']) for g in groups], dtype=float)
    dx = np.diff(xs)
    if not np.isfinite(xs).all() or not (np.all(dx > 1e-6) or np.all(dx < -1e-6)):
        raise ValueError('fit rail positions must be strictly monotonic')
    points = [dict(cm=g['cm'], s_px=float(x), samples=len(g['values']),
                   total_frames=g['total_frames'], coverage=len(g['values'])/float(g['total_frames']))
              for g, x in zip(groups, xs)]
    domain = [float(xs.min()), float(xs.max())]
    if method == 'linear':
        a, b = np.polyfit(xs, cms, 1)
        if not np.isfinite([a, b]).all() or a == 0:
            raise ValueError('degenerate linear model')
        model = dict(type='linear', a=float(a), b=float(b), domain_s_px=domain)
    elif method == 'piecewise':
        order = np.argsort(xs)
        model = dict(type='piecewise', xs=xs[order].tolist(), cms=cms[order].tolist(), domain_s_px=domain)
    else:
        raise ValueError('method must be explicitly linear or piecewise')
    residuals = [abs(map_position(model, float(x))-float(cm)) for x, cm in zip(xs, cms)]
    return model, points, dict(max_fit_residual_cm=max(residuals), note='training residual only; not independent validation')


def map_position(model, s):
    """不外推、不钳位；np.interp只在显式区间检查之后使用。"""
    if not is_number(s) or not model['domain_s_px'][0] <= s <= model['domain_s_px'][1]:
        return None
    value = model['a']*s+model['b'] if model['type'] == 'linear' else np.interp(s, model['xs'], model['cms'])
    return float(value) if math.isfinite(value) else None


def verification(model, fit_points, sessions, binding):
    """逐验证位置计算全部样本误差与完整窗口coverage；每点都必须通过。"""
    if not sessions:
        return dict(sessions=[], metrics=[], passed=False,
                    thresholds=dict(min_samples=MIN_SAMPLES, min_coverage=MIN_COVERAGE, max_abs_cm=MAX_ABS_CM))
    groups = grouped(sessions, binding)
    if len(groups) < 2 or any(b['cm']-a['cm'] <= 1e-6 for a, b in zip(groups, groups[1:])):
        raise ValueError('verification requires at least two distinct known positions')
    if any(abs(g['cm']-p['cm']) <= 1e-6 for g in groups for p in fit_points):
        raise ValueError('verification positions must not overlap fit positions')
    fit_cms = [p['cm'] for p in fit_points]
    metrics = []
    for group in groups:
        errors, out_of_range = [], 0
        for s in group['values']:
            predicted_cm = map_position(model, s)
            if predicted_cm is None:
                out_of_range += 1
            else:
                errors.append(abs(predicted_cm-group['cm']))
        count, total = len(errors), group['total_frames']
        coverage = count/float(total)
        max_abs = max(errors) if errors else None
        # 仅容忍浮点运算舍入，不能用训练误差选择/放宽验证阈值。
        passed = (count >= MIN_SAMPLES and coverage >= MIN_COVERAGE and out_of_range == 0 and
                  min(fit_cms) <= group['cm'] <= max(fit_cms) and max_abs is not None and max_abs <= MAX_ABS_CM+1e-12)
        metrics.append(dict(cm=group['cm'], samples=count, accepted_input_samples=len(group['values']),
                            total_frames=total, coverage=coverage, max_abs_cm=max_abs,
                            mean_abs_cm=float(np.mean(errors)) if errors else None,
                            out_of_range=out_of_range, passed=bool(passed)))
    return dict(sessions=copy.deepcopy(sessions), metrics=metrics, passed=all(p['passed'] for p in metrics),
                thresholds=dict(min_samples=MIN_SAMPLES, min_coverage=MIN_COVERAGE, max_abs_cm=MAX_ABS_CM))


def build_model(binding, fit_sessions, verify_sessions=None, method='linear'):
    """构建新文档，不写盘；CLI只允许保存到尚不存在的新文件。"""
    validate_binding(binding)
    verify_sessions = [] if verify_sessions is None else verify_sessions
    for session in fit_sessions+verify_sessions:
        session_values(session, binding)
    independent(fit_sessions+verify_sessions)
    model, points, fit_stats = fitted(fit_sessions, binding, method)
    validation = verification(model, points, verify_sessions, binding)
    return dict(schema='rail-calibration-v1', binding=copy.deepcopy(binding), rail_binding_id=binding_id(binding),
                method=method, fit_sessions=copy.deepcopy(fit_sessions), model=model, points=points,
                fit_stats=fit_stats, validation=validation, ready=True, validated=validation['passed'])


def numeric_equal(actual, expected):
    """加载跨NumPy版本允许极小拟合舍入差；所有门控均使用重新计算结果。"""
    if isinstance(expected, dict):
        return isinstance(actual, dict) and actual.keys() == expected.keys() and all(numeric_equal(actual[k], v) for k, v in expected.items())
    if isinstance(expected, list):
        return isinstance(actual, list) and len(actual) == len(expected) and all(numeric_equal(a, b) for a, b in zip(actual, expected))
    if type(expected) is bool:
        return type(actual) is bool and actual == expected
    if is_number(expected):
        return is_number(actual) and math.isclose(actual, expected, rel_tol=1e-10, abs_tol=1e-10)
    return type(actual) is type(expected) and actual == expected


def checked_model(data, binding):
    """拒绝手改model/points/metrics/ready；从原始证据重建才得到运行模型。"""
    validate_binding(binding)
    if data.get('schema') != 'rail-calibration-v1' or canonical(data.get('binding')) != canonical(binding):
        raise ValueError('model schema or binding mismatch')
    if data.get('rail_binding_id') != binding_id(binding):
        raise ValueError('model binding id mismatch')
    rebuilt = build_model(binding, data['fit_sessions'], data['validation']['sessions'], data['method'])
    for key in ('model', 'points', 'fit_stats', 'ready', 'validated'):
        if not numeric_equal(data.get(key), rebuilt[key]):
            raise ValueError('stored '+key+' does not match recomputed evidence')
    for key in ('metrics', 'passed', 'thresholds'):
        if not numeric_equal(data['validation'].get(key), rebuilt['validation'][key]):
            raise ValueError('stored independent validation does not match evidence')
    return rebuilt


class RailCalibration(object):
    """运行时只读门控；外部binding若发生变化，现有对象立即停止输出厘米。"""
    def __init__(self, path, binding):
        self.path = str(path)
        self.binding = binding
        self._binding_json = None
        self._model = None
        self._validated = False
        self._status = 'not_loaded'
        try:
            validate_binding(binding)
            self._binding_json = canonical(binding)
            with open(self.path, 'r', encoding='utf-8') as stream:
                data = json.load(stream)
            rebuilt = checked_model(data, binding)
            self._model = rebuilt['model']
            self._validated = rebuilt['validated']
            self._status = ('validated' if self._validated else 'not_verified' if not rebuilt['validation']['sessions'] else 'validation_failed')
        except (OSError, ValueError, TypeError, KeyError, AttributeError, OverflowError, np.linalg.LinAlgError) as exc:
            self._model = None
            self._status = 'invalid: '+str(exc)

    def _bound(self):
        try:
            return self._binding_json is not None and canonical(self.binding) == self._binding_json
        except (TypeError, ValueError):
            return False

    @property
    def ready(self):
        return self._model is not None and self._bound()

    @property
    def validated(self):
        return self.ready and self._validated

    @property
    def status(self):
        return 'binding_mismatch' if self._binding_json is not None and not self._bound() else self._status

    def position(self, s_px):
        if not self.ready or not self.validated:
            return None
        return map_position(self._model, s_px)
