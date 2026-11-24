#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线棋盘标定，不访问摄像头。纯色A4不能标定，不生成默认/猜测内参。

--cols/--rows均指内角点数；square-mm是相邻角点物理间距。
至少12张不同位置/大小/双轴倾角的真实棋盘照片；保持同一相机、分辨率和焦点。
所有输入必须通过检查，不自动剔除高误差图；失败时不创建输出文件。
JSON契约见build结果：schema/model/resolution/camera_id/focus_id/K/D/board/inputs/
reprojection/validation/integrity。校验不能证明照片来源，低残差不是实机精度认证。
"""
import argparse
import glob
import hashlib
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

from camera_geometry import (SCHEMA, MODEL, MIN_VIEWS, MAX_OVERALL_RMS_PX,
                             MAX_VIEW_RMS_PX, seal_document, validate_document)

LIMITS = dict(min_views=MIN_VIEWS, duplicate_corner_rms_px=1.0,
              min_board_area_fraction=.02, min_centroid_span_x=.20,
              min_centroid_span_y=.20, min_corner_hull_fraction=.25,
              min_scale_ratio=1.25, min_tilt_span_x_deg=10., min_tilt_span_y_deg=10.,
              max_overall_rms_px=MAX_OVERALL_RMS_PX, max_view_rms_px=MAX_VIEW_RMS_PX)


def _board_shape(cols, rows):
    if any(type(v) is not int or v < 3 for v in (cols, rows)):
        raise ValueError('棋盘 cols/rows must be integer inner-corner counts >= 3')


def collect_images(pattern, cols, rows):
    """从同一文件字节解码及计算SHA；重复文件/不可读/无棋盘/分辨率混用立即拒绝。"""
    _board_shape(cols, rows)
    root = Path(pattern)
    if root.is_dir():
        paths = sorted(p for p in root.iterdir() if p.is_file() and
                       p.suffix.lower() in ('.png','.jpg','.jpeg','.bmp','.tif','.tiff'))
    else:
        paths = sorted(Path(p) for p in glob.glob(str(pattern), recursive=True) if Path(p).is_file())
    if not paths:
        raise ValueError('no local calibration images matched')
    observations, seen = [], set()
    resolution = None
    for path in paths:
        raw = path.read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        if digest in seen:
            raise ValueError('duplicate image SHA: ' + str(path))
        seen.add(digest)
        gray = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise ValueError('unreadable image: ' + str(path))
        current = (gray.shape[1], gray.shape[0])
        if resolution is not None and current != resolution:
            raise ValueError('mixed image resolution: ' + str(path))
        resolution = current
        found, corners = cv2.findChessboardCorners(gray, (cols, rows),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found or corners is None or len(corners) != cols * rows:
            raise ValueError('chessboard not found: {}; plain A4 is not calibration data'.format(path))
        refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                  (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, .001))
        observations.append(dict(path=str(path.resolve()), sha256=digest,
                                 corners=refined.reshape(-1,2)))
    return observations, resolution


def calibrate_observations(observations, resolution, cols, rows, square_mm, camera_id, focus_id):
    """纯计算入口供离线CLI和synthetic测试使用；不读设备、不写文件。"""
    _board_shape(cols, rows)
    if not math.isfinite(square_mm) or square_mm <= 0:
        raise ValueError('square-mm must be finite and positive')
    if any(not isinstance(v, str) or not v.strip() for v in (camera_id, focus_id)):
        raise ValueError('camera-id/focus-id must be explicit nonempty strings')
    if (len(resolution) != 2 or any(type(v) is not int or v <= 0 for v in resolution)):
        raise ValueError('invalid resolution')
    if len(observations) < MIN_VIEWS:
        raise ValueError('need at least {} different chessboard views; got {}'.format(MIN_VIEWS,len(observations)))
    width, height = resolution
    images, hashes, centers, areas = [], set(), [], []
    for view in observations:
        digest = view['sha256']
        if digest in hashes:
            raise ValueError('duplicate image SHA: ' + view['path'])
        hashes.add(digest)
        points = np.asarray(view['corners'], np.float32)
        if (points.shape != (cols*rows,2) or not np.isfinite(points).all() or
                np.any(points < 0) or np.any(points[:,0] >= width) or np.any(points[:,1] >= height)):
            raise ValueError('invalid/out-of-frame corners: ' + view['path'])
        # 角点顺序可能180度反转；两种顺序都检查，拒绝改文件名重复采样。
        for previous in images:
            distance = min(np.sqrt(np.mean(np.sum((points-previous)**2,axis=1))),
                           np.sqrt(np.mean(np.sum((points-previous[::-1])**2,axis=1))))
            if distance < LIMITS['duplicate_corner_rms_px']:
                raise ValueError('duplicate corner pose: ' + view['path'])
        images.append(points)
        centers.append(points.mean(axis=0))
        areas.append(cv2.contourArea(cv2.convexHull(points)) / float(width*height))
    centers = np.asarray(centers)
    hull_fraction = cv2.contourArea(cv2.convexHull(np.concatenate(images))) / float(width*height)
    coverage = dict(centroid_span_x=float(np.ptp(centers[:,0])/width),
                    centroid_span_y=float(np.ptp(centers[:,1])/height),
                    corner_hull_fraction=float(hull_fraction),
                    min_board_area_fraction=float(min(areas)), max_board_area_fraction=float(max(areas)),
                    scale_ratio=float(math.sqrt(max(areas)/min(areas))) if min(areas)>0 else 0.)
    checks = [('centroid_span_x','min_centroid_span_x'),('centroid_span_y','min_centroid_span_y'),
              ('corner_hull_fraction','min_corner_hull_fraction'),
              ('min_board_area_fraction','min_board_area_fraction'),('scale_ratio','min_scale_ratio')]
    if any(coverage[key] < LIMITS[limit] for key,limit in checks):
        raise ValueError('degenerate coverage: ' + json.dumps(coverage, sort_keys=True))
    objects = np.zeros((cols*rows,3), np.float32)
    objects[:,:2] = np.mgrid[0:cols,0:rows].T.reshape(-1,2)*square_mm
    object_views = [objects.copy() for _ in images]
    # 固定OpenCV常规五参数模型：k1,k2,p1,p2,k3；不启用rational/fisheye等额外模型。
    rms, K, D, rotations, translations = cv2.calibrateCamera(object_views, images, (width,height), None, None)
    if not np.isfinite(rms) or not np.isfinite(K).all() or not np.isfinite(D).all():
        raise ValueError('calibrateCamera produced nonfinite parameters')
    per_view, squared_sum, point_count, tilts = [], 0., 0, []
    for points, rotation, translation in zip(images, rotations, translations):
        projected = cv2.projectPoints(objects, rotation, translation, K, D)[0].reshape(-1,2)
        error = projected.astype(np.float64)-points
        sum_sq = float(np.sum(error*error))
        per_view.append(math.sqrt(sum_sq/len(points)))
        squared_sum += sum_sq
        point_count += len(points)
        matrix = cv2.Rodrigues(rotation)[0]
        normal = matrix[:,2]
        tilts.append([math.degrees(math.atan2(normal[0],normal[2])),
                      math.degrees(math.atan2(normal[1],normal[2]))])
        depths = objects.dot(matrix.T)[:,2] + float(translation[2,0])
        if not np.isfinite(depths).all() or np.any(depths <= 0):
            raise ValueError('degenerate pose: chessboard behind camera')
    overall = math.sqrt(squared_sum/point_count)
    if overall > MAX_OVERALL_RMS_PX or max(per_view) > MAX_VIEW_RMS_PX:
        raise ValueError('reprojection rejected; no views dropped: overall={} per_view={}'.format(overall,per_view))
    tilts = np.asarray(tilts)
    pose = dict(tilt_span_x_deg=float(np.ptp(tilts[:,0])), tilt_span_y_deg=float(np.ptp(tilts[:,1])),
                per_view_tilt_deg=tilts.tolist())
    if pose['tilt_span_x_deg'] < LIMITS['min_tilt_span_x_deg'] or pose['tilt_span_y_deg'] < LIMITS['min_tilt_span_y_deg']:
        raise ValueError('degenerate pose diversity: ' + json.dumps(pose))
    document = dict(schema=SCHEMA, model=MODEL, resolution=list(resolution), camera_id=camera_id,
        focus_id=focus_id, K=K.tolist(), D=D.reshape(-1).tolist(),
        board=dict(inner_cols=cols, inner_rows=rows, square_mm=float(square_mm)),
        inputs=[dict(path=view['path'], sha256=view['sha256']) for view in observations],
        reprojection=dict(overall_rms_px=overall, opencv_rms_px=float(rms), per_view_rms_px=per_view),
        validation=dict(passed=True, limits=LIMITS.copy(), coverage=coverage, pose=pose,
                        input_views=len(images), used_views=len(images), dropped_views=0),
        limitations='Fit residuals and diversity checks are not independent camera accuracy or physical-source certification.',
        runtime=dict(opencv=cv2.__version__, numpy=np.__version__))
    document = seal_document(document)
    validate_document(document)
    return document


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', required=True, help='local image glob or directory')
    parser.add_argument('--cols', type=int, required=True, help='inner corner columns')
    parser.add_argument('--rows', type=int, required=True, help='inner corner rows')
    parser.add_argument('--square-mm', type=float, required=True)
    parser.add_argument('--camera-id', required=True)
    parser.add_argument('--focus-id', required=True)
    parser.add_argument('--output', required=True, help='new JSON file; must not exist')
    args = parser.parse_args(argv)
    output = Path(args.output)
    try:
        if output.exists():
            raise ValueError('output already exists; refusing overwrite')
        # 先完整验证，再用排他创建写入；无真实图像输入时绝不写默认内参。
        observations, resolution = collect_images(args.images, args.cols, args.rows)
        document = calibrate_observations(observations, resolution, args.cols, args.rows,
                                           args.square_mm, args.camera_id, args.focus_id)
        encoded = json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
        with output.open('x', encoding='utf-8') as stream:
            stream.write(encoded)
        print('calibration saved: {} ({} views; RMS {:.6f} px)'.format(
            output, len(observations), document['reprojection']['overall_rms_px']))
        return 0
    except (OSError, ValueError, TypeError, cv2.error) as exc:
        print('calibration rejected: {}'.format(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
