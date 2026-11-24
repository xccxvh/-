#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读旧分支回归；所有输出留在本脚本目录，不连接设备或改写源文件。"""
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
                tracker=normalize({k: output[k] for k in ('x_px', 'method', 'state', 'valid')})))
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


def run(args):
    old, new = Path(args.old).resolve(), Path(args.new).resolve()
    config = old / 'config_fusion_v2.json'
    evidence_dir = old / 'test_results/fusion_v2'
    cfg = read(config)
    if (cfg['ball'].get('mode', 'legacy') == 'relative_cap' or
            cfg['tracking'].get('identity_check', False)):
        raise ValueError('original config unexpectedly enables new branch')
    before = {'old': fingerprint(old), 'new': fingerprint(new)}
    cases = []
    for name in REPORTS:
        tracked_path = evidence_dir / (name + '_tracked.json')
        full_path = evidence_dir / (name + '_full.json')
        tracked, full = read(tracked_path), read(full_path)
        te, fe = tracked['evidence'], full['evidence']
        assert te['video'] == fe['video'] and te['sha256'] == fe['sha256']
        assert fe['start_frame'] == 0 and te['start_frame'] == 4
        assert te['end_frame'] == fe['end_frame']
        assert tracked['calibration_sha256'] == full['calibration_sha256'] == CAL_SHA
        cases.append(dict(truth_cm=tracked['truth_cm'], original_video=te['video'],
            basename=Path(te['video']).name, expected_video_sha256=te['sha256'],
            full_window=[0, fe['end_frame']], steady_window=[4, te['end_frame']],
            evidence={str(p): sha(p) for p in (tracked_path, full_path)},
            historical_valid_samples=tracked['samples']))
    assert sum(c['full_window'][1] + 1 for c in cases) == 205
    assert sum(c['steady_window'][1] - 3 for c in cases) == 189
    # 只按原文件名寻找候选，再校验整文件SHA；不拿其他视频替代原始证据。
    roots = args.video_root or ['/home/user', '/mnt', '/media']
    found = {c['basename']: [] for c in cases}
    search_errors = []
    for root in roots:
        if not Path(root).exists():
            continue
        for directory, dirs, files in os.walk(root, onerror=lambda e: search_errors.append(str(e))):
            for name in set(files).intersection(found):
                found[name].append(str(Path(directory) / name))
    for case in cases:
        original = Path(case['original_video'])
        paths = found[case['basename']] + ([str(original)] if original.is_file() else [])
        hashes = {p: sha(p) for p in sorted(set(paths))}
        matching = [p for p, digest in hashes.items() if digest == case['expected_video_sha256']]
        case['discovered_raw_hashes'] = hashes
        if not matching:
            case['status'] = 'missing_raw' if not hashes else 'raw_sha_mismatch'
            continue
        case['local_video'] = matching[0]
        case['runs'] = {}
        try:
            for label, source in [('old', old), ('new', new)]:
                command = [sys.executable, '-B', str(Path(__file__).resolve()), '--worker',
                    '--source', str(source), '--config', str(config), '--video', matching[0],
                    '--end', str(case['full_window'][1])]
                env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
                result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                        env=env, universal_newlines=True, timeout=300)
                if result.returncode:
                    raise RuntimeError(result.stderr)
                case['runs'][label] = json.loads(result.stdout)
            case['mismatches'] = compare(case['runs']['old'], case['runs']['new'])
            for label in ('old', 'new'):
                frames = case['runs'][label]['frames']
                case[label + '_valid_full'] = sum(f['tracker']['valid'] for f in frames)
                case[label + '_valid_steady'] = sum(f['tracker']['valid'] for f in frames[4:])
            case['status'] = 'pass' if not case['mismatches'] else 'mismatch'
            if sha(matching[0]) != case['expected_video_sha256']:
                case['status'] = 'raw_changed_during_run'
        except Exception as exc:
            case['status'] = 'execution_error'
            case['error'] = str(exc)
    after = {'old': fingerprint(old), 'new': fingerprint(new)}
    preserved = before == after
    calibration_ok = all(sha(r / 'calibration_fusion_v2.json') == CAL_SHA for r in (old, new))
    passed = preserved and calibration_ok and all(c['status'] == 'pass' for c in cases)
    status = 'pass' if passed else 'blocked_missing_raw' if any(c['status'] == 'missing_raw' for c in cases) else 'failed'
    report = dict(status=status, scope='Legacy branch preservation only; NOT relative-cap certification.',
        old_source=str(old), new_source=str(new), config_path=str(config), config_sha256=sha(config),
        original_config=cfg, calibration_sha256=CAL_SHA, calibration_matches_history=calibration_ok,
        source_hashes_before=before, source_hashes_after=after, sources_unchanged=preserved,
        static_report_sha256=sha(old / 'docs/STATIC_VALIDATION_REPORT.md'),
        search_roots=roots, search_errors=search_errors, expected_full_frames=205,
        expected_steady_frames=189, compared_frames=sum(len(c.get('runs', {}).get('new', {}).get('frames', []))
            for c in cases if 'mismatches' in c), cases=cases)
    (HERE / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    lines = ['# 四点旧分支保持性回归', '', '状态：`{}`。已比较帧数：{} / 205。'.format(status, report['compared_frames']), '',
        '仅验证旧配置下新旧源码行为一致，不认证 relative-cap 算法，不重做厘米精度标定。', '',
        '| 位置cm | 原始视频 | 完整窗口（零基、含端点） | 稳态窗口 | 状态 |',
        '|---|---|---|---|---|']
    for c in cases:
        lines.append('| {:+g} | {} | {}–{} | {}–{} | {} |'.format(c['truth_cm'], c['basename'],
            *c['full_window'], *c['steady_window'], c['status']))
    lines += ['', '原历史记录：205帧，扣除每段最初4帧后189稳态帧；此计数不是本次重放结果。', '',
        '方法：两个独立子进程分别导入源树；从帧0顺序解码，不seek。使用旧 config_fusion_v2.json，'
        '按原 collect_window 每帧强制检测轨道，采用视频FPS（如有有效actual_fps旁文件则优先）。'
        '先比解码帧SHA，再精确比较全部候选属性（含extra）、tracker的x_px/method/state/valid，忽略新增reason。', '',
        '旧标定SHA-256：`{}`；新旧均匹配历史：{}；源码/配置/标定运行前后未变：{}。'.format(CAL_SHA, calibration_ok, preserved), '',
        '搜索目录：{}。搜索权限错误数：{}。缺失原始视频时不使用历史JSON充当重新运行结果。'.format(', '.join(roots), len(search_errors)), '',
        '复跑：`python3 -B verify_static_regression.py`；也可追加 `--video-root /path/to/raw`（可重复）。',
        '退出码：0全部一致；2缺失原始视频；1不一致/执行失败。详细路径、SHA、每帧输出及差异见 report.json。', '',
        '未调用main、摄像头、UART或Nano；只在本目录保存脚本与报告。']
    (HERE / 'REPORT.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('status', 'compared_frames', 'calibration_matches_history', 'sources_unchanged')}, ensure_ascii=False))
    return 0 if passed else 2 if status == 'blocked_missing_raw' else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--old', default=str(OLD))
    parser.add_argument('--new', default=str(NEW))
    parser.add_argument('--video-root', action='append')
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
