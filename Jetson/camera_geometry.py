# -*- coding: utf-8 -*-
"""可选相机几何层；无参数时仅允许raw，不推测K/D。

points：原图匹配，原像素点经correct_points后由父模块重新估计几何关系。
image：固定原尺寸、P=K映射；测量已经是校正像素，父模块不得再次correct_points。
参考图image_context及厘米binding由父模块检查，本类不解释参考图或厘米标定。
内参文件SHA用于绑定；文件内payload校验只检测内容变化，不证明采集真实性。
"""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

SCHEMA = 'camera-intrinsics-v1'
MODEL = 'opencv-pinhole-brown5-v1'
CANONICAL = 'json-sort-keys-compact-utf8-v1'
MIN_VIEWS = 12
MAX_OVERALL_RMS_PX = 1.0
MAX_VIEW_RMS_PX = 2.0


def seal_document(document):
    """校验定义：去掉顶层integrity后，排序紧凑UTF-8 JSON的SHA-256。"""
    result = dict(document)
    result.pop('integrity', None)
    encoded = json.dumps(result, sort_keys=True, separators=(',', ':'),
                         ensure_ascii=False, allow_nan=False).encode('utf-8')
    result['integrity'] = dict(algorithm='sha256', canonicalization=CANONICAL,
                               payload_sha256=hashlib.sha256(encoded).hexdigest())
    return result


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate JSON key: ' + key)
        result[key] = value
    return result


def _array(value, shape, label):
    try:
        source = np.asarray(value, dtype=object)
        if source.shape != shape or any(type(v) not in (int, float) for v in source.flat):
            raise ValueError('expected numeric shape {}'.format(shape))
        result = np.asarray(value, dtype=np.float64)
        if not np.isfinite(result).all():
            raise ValueError('nonfinite values')
        return result
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError('{} invalid: {}'.format(label, exc))


def validate_document(document):
    """验证固定模型及误差记录；拒绝fisheye、未知版本及未通过诊断的文件。"""
    if not isinstance(document, dict):
        raise ValueError('intrinsics must be a JSON object')
    if document.get('schema') != SCHEMA or document.get('model') != MODEL:
        raise ValueError('unsupported intrinsics schema/model')
    if document.get('integrity') != seal_document(document)['integrity']:
        raise ValueError('intrinsics checksum mismatch')
    resolution = document.get('resolution')
    if (not isinstance(resolution, list) or len(resolution) != 2 or
            any(type(v) is not int or v <= 0 for v in resolution)):
        raise ValueError('invalid intrinsics resolution')
    for key in ('camera_id', 'focus_id'):
        if not isinstance(document.get(key), str) or not document[key].strip():
            raise ValueError('intrinsics requires nonempty ' + key)
    K = _array(document.get('K'), (3, 3), 'K')
    D = _array(document.get('D'), (5,), 'D')
    if (K[0,0] <= 0 or K[1,1] <= 0 or K[0,1] != 0 or K[1,0] != 0 or
            not np.array_equal(K[2], [0.,0.,1.]) or
            not 0 <= K[0,2] < resolution[0] or not 0 <= K[1,2] < resolution[1]):
        raise ValueError('invalid pinhole K: focal/principal point/canonical matrix')
    validation = document.get('validation')
    if not isinstance(validation, dict) or validation.get('passed') is not True:
        raise ValueError('intrinsics validation did not pass')
    reprojection = document.get('reprojection', {})
    inputs = document.get('inputs', [])
    if not isinstance(reprojection, dict) or not isinstance(inputs, list) or len(inputs) < MIN_VIEWS:
        raise ValueError('missing calibration view evidence')
    hashes = [entry.get('sha256') if isinstance(entry, dict) else None for entry in inputs]
    if (any(not isinstance(h, str) or len(h) != 64 or any(c not in '0123456789abcdef' for c in h)
            for h in hashes) or len(set(hashes)) != len(hashes)):
        raise ValueError('invalid or duplicate input SHA evidence')
    per_view = _array(reprojection.get('per_view_rms_px'), (len(inputs),), 'per-view RMS')
    overall = _array([reprojection.get('overall_rms_px')], (1,), 'overall RMS')[0]
    if (overall < 0 or overall > MAX_OVERALL_RMS_PX or np.any(per_view < 0) or
            np.any(per_view > MAX_VIEW_RMS_PX)):
        raise ValueError('intrinsics reprojection error exceeds limits')
    return K, D


def _validate_image_maps(maps, K, D, width, height):
    """启动时独立校验P=K的Brown逆采样maps；错误时拒绝image而非降级。

    不使用projectPoints/undistort等同库计算作真值，避免共享实现错误。
    9x9覆盖边缘及内部，是兼容性抽检，不代表镜头标定或逐像素精度认证。
    """
    reason = 'image map Brown validation failed'
    if (not isinstance(maps, (tuple, list)) or len(maps) != 2 or
            any(not isinstance(m, np.ndarray) or m.shape != (height, width) or
                m.dtype != np.float32 or not np.isfinite(m).all() for m in maps)):
        raise ValueError(reason + ': expected finite CV_32FC1 maps at original resolution')
    fx, fy, cx, cy = (float(K[0,0]), float(K[1,1]), float(K[0,2]), float(K[1,2]))
    k1, k2, p1, p2, k3 = [float(v) for v in D]
    expected, actual = [], []
    # 使用加载的实际焦距、主点及全部五个系数；不采用固定fixture参数。
    for v in np.linspace(0, height-1, 9, dtype=int):
        for u in np.linspace(0, width-1, 9, dtype=int):
            x, y = (int(u)-cx)/fx, (int(v)-cy)/fy
            r2 = x*x + y*y
            radial = 1. + k1*r2 + k2*r2*r2 + k3*r2*r2*r2
            xd = x*radial + 2.*p1*x*y + p2*(r2 + 2.*x*x)
            yd = y*radial + p1*(r2 + 2.*y*y) + 2.*p2*x*y
            expected.append((fx*xd + cx, fy*yd + cy))
            actual.append((maps[0][v,u], maps[1][v,u]))
    expected = np.asarray(expected, dtype=np.float64)
    if not np.isfinite(expected).all():
        raise ValueError(reason + ': independent projection is nonfinite')
    scale = max(float(width), float(height), 1., float(np.max(np.abs(expected))))
    # CV_32FC1最终保存会舍入；允许坐标尺度的一个float32 ULP（包括
    # 浮点公式运算余量），不允许借用remap的1/32插值量化误差作几何门限。
    if scale >= np.finfo(np.float32).max:
        raise ValueError(reason + ': projection exceeds float32 coordinate range')
    limit = float(np.spacing(np.float32(scale)))
    error = float(np.max(np.abs(np.asarray(actual, dtype=np.float64)-expected)))
    if error > limit:
        raise ValueError(reason + ': max_error_px={:.9g}, limit_px={:.9g}'.format(error, limit))


def _project_image_maps(K, D, width, height):
    """Project ideal output-pixel rays into the raw image. Tile to bound OpenCV Jacobian memory."""
    map_x = np.empty((height, width), dtype=np.float32)
    map_y = np.empty_like(map_x)
    zero = np.zeros(3, dtype=np.float64)
    for top in range(0, height, 16):
        bottom = min(top + 16, height)
        v, u = np.mgrid[top:bottom, :width]
        rays = np.stack(((u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1],
                         np.ones_like(u)), axis=-1)
        pixels, _ = cv2.projectPoints(rays.reshape(-1, 3), zero, zero, K, D)
        pixels = pixels.reshape(bottom - top, width, 2)
        map_x[top:bottom] = pixels[:, :, 0]
        map_y[top:bottom] = pixels[:, :, 1]
    return map_x, map_y


class CameraGeometry(object):
    def __init__(self, cfg, width, height, base_dir=None):
        if any(type(v) is not int or v <= 0 for v in (width, height)):
            raise ValueError('width/height must be positive integers')
        self.width, self.height = width, height
        self.mode = cfg.get('mode', 'raw')
        enabled = cfg.get('undistort_enabled', False)
        if type(enabled) is not bool or self.mode not in ('raw', 'points', 'image'):
            raise ValueError('invalid mode/undistort_enabled')
        if enabled != (self.mode != 'raw'):
            raise ValueError('raw requires correction disabled; points/image require undistort_enabled=true')
        camera_id, focus_id = cfg.get('camera_id', ''), cfg.get('focus_id', '')
        if not isinstance(camera_id, str) or not isinstance(focus_id, str):
            raise ValueError('camera_id/focus_id must be strings')
        self.intrinsics_valid = False
        self.image_space = 'undistorted_image' if self.mode == 'image' else 'raw_image'
        self.context = dict(mode=self.mode, resolution=[width, height], camera_id=camera_id,
                            focus_id=focus_id, intrinsics_sha256=None,
                            correction_model='none', projection='identity')
        self._K = self._D = self._maps = None
        if self.mode == 'raw':
            return  # raw不读取文件，不通过虚构单位矩阵制造“有效内参”。
        if not camera_id.strip() or not focus_id.strip():
            raise ValueError('enabled geometry requires nonempty camera_id/focus_id')
        file_name = cfg.get('intrinsics_file')
        if not isinstance(file_name, str) or not file_name:
            raise ValueError('enabled geometry requires intrinsics_file')
        path = Path(file_name)
        if not path.is_absolute():
            path = (Path(base_dir) if base_dir is not None else Path(__file__).resolve().parent) / path
        try:
            data = path.read_bytes()
            document = json.loads(data.decode('utf-8'), object_pairs_hook=_unique_object)
            K, D = validate_document(document)
        except (OSError, UnicodeError, ValueError, TypeError, OverflowError) as exc:
            raise ValueError('cannot use intrinsics {}: {}'.format(path, exc))
        if document['resolution'] != [width, height]:
            raise ValueError('intrinsics resolution mismatch')
        if document['camera_id'] != camera_id or document['focus_id'] != focus_id:
            raise ValueError('intrinsics camera_id/focus_id mismatch')
        digest = hashlib.sha256(data).hexdigest()
        if cfg.get('intrinsics_sha256') is not None and cfg['intrinsics_sha256'] != digest:
            raise ValueError('intrinsics file SHA256 binding mismatch')
        self._K, self._D = K, D
        if self.mode == 'image':
            # 固定P=K、原分辨率；不使用getOptimalNewCameraMatrix、不裁剪或缩放FOV。
            # 校正造成的边界空洞保留为黑边，不能保证原图边缘内容全部保留。
            builder = cfg.get('map_builder', 'native')
            if builder == 'project_points':
                self._maps = _project_image_maps(K, D, width, height)
            elif builder == 'native':
                self._maps = cv2.initUndistortRectifyMap(K, D, None, K, (width, height), cv2.CV_32FC1)
            else:
                raise ValueError('unknown image map_builder: {}'.format(builder))
            # 必须在intrinsics_valid/context成功赋值之前检查；已知错误不能启用。
            _validate_image_maps(self._maps, K, D, width, height)
        self.context.update(intrinsics_sha256=digest, correction_model=MODEL, projection='P=K')
        self.intrinsics_valid = True

    def prepare_frame(self, frame):
        if (not isinstance(frame, np.ndarray) or frame.ndim not in (2, 3) or
                frame.shape[:2] != (self.height, self.width)):
            raise ValueError('frame resolution/shape mismatch')
        if self.mode != 'image':
            return frame
        return cv2.remap(frame, self._maps[0], self._maps[1], cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    def correct_points(self, points):
        try:
            values = np.asarray(points, dtype=np.float64)
        except (ValueError, TypeError) as exc:
            raise ValueError('points must be numeric Nx2: {}'.format(exc))
        if values.ndim != 2 or values.shape[1] != 2 or not np.isfinite(values).all():
            raise ValueError('points must be finite Nx2')
        if self.mode == 'raw':
            return points
        if len(values) == 0:
            return values.copy()
        # 始终接收原图像素；image模式调用方只可用于参考原始点，不能重复校正测量。
        result = cv2.undistortPoints(values.reshape(-1,1,2), self._K, self._D, P=self._K).reshape(-1,2)
        if not np.isfinite(result).all():
            raise ValueError('point correction produced nonfinite coordinates')
        return result
