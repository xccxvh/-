#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""不同姿态补充旧分支回归；不是原始四点替代，不认证 relative-cap。"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
OLD = Path('/home/user/ball_vision_src')
NEW = Path('/home/user/h_contest_vision_repository/Jetson')
CAL_SHA = '5241e1b5a607ef072f62d0bd23e079f1efbb8776d788e8c2a3938ec34e5a59f1'
REPORTS = ['verify_minus4', 'verify_minus2', 'retest_plus2', 'verify_plus4']


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def normalize(value):
    # 保留所有候选属性及 extra；仅将 NumPy/元组转换为可保存的 JSON 类型。
    if isinstance(value, dict):
        return {str(k): normalize(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [normalize(v) for v in value]
    if hasattr(value, 'tolist'):
        return normalize(value.tolist())
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError('nonfinite measurement')
    return value


def worker(args):
    # 两个全新进程分别导入源树，避免同名模块缓存造成“旧/新”实际同源。
    root = Path(args.source).resolve()
    sys.path.insert(0, str(root))
    import cv2
    import numpy as np
    from rail_detector import RailDetector
    from ball_detector import BallDetector
    from ball_tracker import BallTracker
    cfg = read(args.config)
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise RuntimeError('cannot open video: ' + args.video)
    try:
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(cap.get(cv2.CAP_PROP_FPS))
        header_fps = fps
        sidecar = Path(args.video).with_suffix('.json')
        # 与原 calibrate.collect_window / VideoFile 的时间戳规则一致。
        if sidecar.exists():
            try:
                actual = float(read(sidecar).get('actual_fps', 0))
                if actual > 1.0:
                    fps = actual
            except (ValueError, TypeError, OSError):
                pass
        if not math.isfinite(fps):
            raise ValueError('nonfinite fps')
        rail_detector = RailDetector(cfg['rail'], width, height)
        ball_detector = BallDetector(cfg['ball'], width, height)
        tracker = BallTracker(cfg['tracking'], width, height)
        frames = []
        # 从零顺序 read 到包含的终止帧；不 seek，不在帧4重置跟踪器。
        for index in range(args.end + 1):
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('early EOF at frame {}'.format(index))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            rail = rail_detector.detect(gray, force=True)
            candidates = ball_detector.detect(frame, gray, rail)
            output = tracker.update(candidates, index / max(fps, 1.0))
            frames.append(dict(frame=index,
                decoded_sha256=hashlib.sha256(frame.tobytes()).hexdigest(),
                candidates=normalize([vars(c) for c in candidates]),
                tracker=normalize({k: v for k, v in output.items() if k != 'reason'})))
        extra_ok, _ = cap.read()
        if extra_ok:
            raise RuntimeError('video exceeds expected full 149 frames')
        modules = {}
        for name, module in sorted(sys.modules.items()):
            file = getattr(module, '__file__', None)
            if file and Path(file).resolve().parent == root:
                modules[name] = str(Path(file).resolve())
        return dict(frames=frames, fps=fps, header_fps=header_fps,
                    sidecar_sha256=sha(sidecar) if sidecar.exists() else None,
                    width=width, height=height, imported_modules=modules,
                    python=sys.version, opencv=cv2.__version__, numpy=np.__version__)
    finally:
        cap.release()


def compare(old, new):
    # 精确比较，无四舍五入或数值容差；候选顺序及嵌套 extra 也参与比较。
    mismatches = []
    if len(old['frames']) != len(new['frames']):
        mismatches.append(dict(field='frame_count'))
    for a, b in zip(old['frames'], new['frames']):
        for field in ('decoded_sha256', 'candidates', 'tracker'):
            if a[field] != b[field]:
                mismatches.append(dict(frame=a['frame'], field=field))
    for field in ('fps', 'header_fps', 'sidecar_sha256', 'width', 'height'):
        if old[field] != new[field]:
            mismatches.append(dict(field=field))
    return mismatches


def fingerprint(root):
    paths = list(root.glob('*.py')) + [root / 'config_fusion_v2.json',
                                       root / 'calibration_fusion_v2.json']
    return {str(p): sha(p) for p in sorted(paths) if p.is_file()}


CASES = ['minus5_static_20261113', 'zero_static_20261113',
         'plus5_static_20261113', 'plus2_independent_20261113',
         'minus2_independent_20261113', 'zero_motion_20261113']


def run(args):
    old, new = Path(args.old).resolve(), Path(args.new).resolve()
    config = old / 'config_fusion_v2.json'
    cfg = read(config)
    if (cfg['ball'].get('mode', 'legacy') == 'relative_cap' or
            cfg['tracking'].get('identity_check', False)):
        raise ValueError('original config unexpectedly enables new branch')
    before = {'old': fingerprint(old), 'new': fingerprint(new)}
    calibration_before = {label: sha(root / 'calibration_fusion_v2.json')
                          for label, root in [('old', old), ('new', new)]}
    cases = []
    for name in CASES:
        case = dict(name=name, expected_frames=149, inclusive_window=[0, 148])
        cases.append(case)
        videos = sorted((old / 'test_results' / name).glob('raw*.avi'))
        if len(videos) != 1:
            case.update(status='missing_or_ambiguous_raw', found=[str(p) for p in videos])
            continue
        video = videos[0]
        sidecar = video.with_suffix('.json')
        case.update(video=str(video), video_sha256_before=sha(video),
                    sidecar_sha256_before=sha(sidecar) if sidecar.exists() else None, runs={})
        print('Comparing {}: frames 0-148'.format(name), flush=True)
        try:
            # 旧、新进程按顺序执行，使用同一配置路径和原始文件；禁用pyc。
            for label, root in [('old', old), ('new', new)]:
                command = [sys.executable, '-B', str(Path(__file__).resolve()), '--worker',
                           '--source', str(root), '--config', str(config),
                           '--video', str(video), '--end', '148']
                result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    universal_newlines=True, timeout=300,
                    env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
                if result.returncode:
                    raise RuntimeError(result.stderr)
                case['runs'][label] = json.loads(result.stdout)
                if result.stderr:
                    case[label + '_stderr'] = result.stderr
            case['mismatches'] = compare(case['runs']['old'], case['runs']['new'])
            case['summary'] = {}
            for label, data in case['runs'].items():
                frames = data['frames']
                case['summary'][label] = dict(
                    frames=len(frames), valid_frames=sum(f['tracker']['valid'] for f in frames),
                    frames_with_candidates=sum(bool(f['candidates']) for f in frames),
                    total_candidates=sum(len(f['candidates']) for f in frames))
            case['status'] = 'pass' if not case['mismatches'] else 'mismatch'
        except Exception as exc:
            case.update(status='execution_error', error=str(exc))
        case['video_sha256_after'] = sha(video)
        case['sidecar_sha256_after'] = sha(sidecar) if sidecar.exists() else None
        if (case['video_sha256_before'] != case['video_sha256_after'] or
                case['sidecar_sha256_before'] != case['sidecar_sha256_after']):
            case['status'] = 'input_changed_during_run'
    after = {'old': fingerprint(old), 'new': fingerprint(new)}
    calibration_after = {label: sha(root / 'calibration_fusion_v2.json')
                         for label, root in [('old', old), ('new', new)]}
    preserved = before == after
    calibration_ok = calibration_before == calibration_after and all(
        digest == CAL_SHA for digest in calibration_after.values())
    passed = preserved and calibration_ok and all(c['status'] == 'pass' for c in cases)
    report = dict(
        status='pass' if passed else 'failed',
        scope='Supplemental different-pose legacy branch preservation regression ONLY.',
        limitations=['NOT a substitute for the missing original four static videos (205/189 frames).',
                     'NOT certification of relative-cap, centimeter accuracy, or hardware performance.',
                     'Validity counts describe algorithm outputs, not ground-truth correctness.'],
        method=dict(decode='Sequential read from 0 through 148, then verify EOF; no seeking.',
                    isolation='Old/new in separate sequential subprocesses; original config for both.',
                    candidates='Exact ordered comparison of all candidate attributes including nested extra.',
                    tracker='Exact comparison of every output field except additive reason.',
                    input_check='Raw SHA256 before/after and per-decoded-frame SHA256 equality.',
                    rail='force=True each frame, matching original collect_window.',
                    time='index/max(fps,1), sidecar actual_fps > 1 overrides container FPS.'),
        old_source=str(old), new_source=str(new), config_path=str(config),
        config_sha256=sha(config), original_config=cfg,
        calibration_sha256_before=calibration_before, calibration_sha256_after=calibration_after,
        calibration_matches_history=calibration_ok, sources_unchanged=preserved,
        source_hashes_before=before, source_hashes_after=after,
        expected_total_frames=894,
        compared_frames=sum(c.get('summary', {}).get('new', {}).get('frames', 0) for c in cases),
        mismatch_count=sum(len(c.get('mismatches', [])) for c in cases), cases=cases,
        script_sha256=sha(Path(__file__)))
    destination = HERE / 'supplemental_report.json'
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('status', 'compared_frames', 'mismatch_count',
          'calibration_matches_history', 'sources_unchanged')}, ensure_ascii=False))
    return 0 if passed else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--old', default=str(OLD))
    parser.add_argument('--new', default=str(NEW))
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    for flag in ('source', 'config', 'video'):
        parser.add_argument('--' + flag, help=argparse.SUPPRESS)
    parser.add_argument('--end', type=int, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        print(json.dumps(worker(args), allow_nan=False))
        return 0
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
