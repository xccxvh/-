# -*- coding: utf-8 -*-
"""独立的新轨道厘米标定CLI，不导入/覆盖历史Calibration。

采样（确认所选完整区间的实物在-5cm；未验证时cm列为空是正常的）：
  python3 calibrate_rail.py collect --csv geometry.csv --video raw.avi \
    --known-position --cm -5 --start-frame 100 --end-frame 159 --output fit-m5.json
默认sidecar为geometry.binding.json，也可显式--binding-file（别名--binding）。
拟合默认linear；piecewise必须明确指定，不通过训练残差自动切换：
  python3 calibrate_rail.py fit --binding-file geometry.binding.json \
    --sessions fit-m5.json fit-0.json fit-p5.json --output model-v1.json
独立验证至少两个不同实物位置，且不与拟合位置/视频区间重叠：
  python3 calibrate_rail.py verify --binding-file geometry.binding.json \
    --model model-v1.json --sessions verify-m3.json verify-p3.json --output model-v2.json
所有output只能新建，旧模型不被覆盖；验证失败也保存validated=false供审阅。
"""
import argparse
import copy
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path

from rail_calibration import (binding_id, canonical, validate_binding, session_values,
                              build_model, checked_model)


REQUIRED_COLUMNS = ('frame', 'rail_s_px', 'pixel_valid', 'reference_valid',
                    'calibration_valid', 'cm_valid', 'rail_binding_id')


def sha256(path):
    """只读计算证据哈希，不解码视频、不访问摄像头。"""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def read_binding(path):
    """兼容父模块sidecar包装和直接binding；包装ID必须完全正确。"""
    data = read_json(path)
    wrapped = isinstance(data, dict) and 'binding' in data
    binding = data['binding'] if wrapped else data
    validate_binding(binding)
    if wrapped and data.get('rail_binding_id') != binding_id(binding):
        raise ValueError('binding sidecar id mismatch')
    return binding


def save_new(path, data):
    """排他创建，先序列化再打开，任何既有路径（含模型）都不覆盖。"""
    canonical(data)
    text = json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2)+'\n'
    with Path(path).open('x', encoding='utf-8') as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())


def bool_flag(value):
    if value is None:
        raise ValueError('missing CSV validity flag')
    value = value.strip().lower()
    if value not in ('0', '1', 'false', 'true'):
        raise ValueError('CSV validity flag must be 0/1 or true/false')
    return value in ('1', 'true')


def collect(args, binding, binding_path):
    """保留选区原始flag与缺帧信息；不使用已有厘米列作自我标定。"""
    if not args.known_position or not math.isfinite(args.cm) or not 0 <= args.start_frame <= args.end_frame:
        raise ValueError('confirm known physical position and finite cm/inclusive frame range')
    video, csv_path = Path(args.video).resolve(), Path(args.csv).resolve()
    if not video.is_file() or not csv_path.is_file():
        raise ValueError('video and CSV must be existing local files')
    video_sha = sha256(video)
    raw_csv = csv_path.read_bytes()
    csv_sha = hashlib.sha256(raw_csv).hexdigest()
    reader = csv.DictReader(io.StringIO(raw_csv.decode('utf-8-sig')))
    columns = reader.fieldnames or []
    if len(set(columns)) != len(columns) or any(key not in columns for key in REQUIRED_COLUMNS):
        raise ValueError('CSV missing required columns or has duplicate headers')
    prediction_contract = 'column' if 'predicted' in columns else 'pixel_valid'
    bid = binding_id(binding)
    seen, rows = set(), []
    for raw in reader:
        frame = int(raw['frame'])
        if frame < 0 or frame in seen:
            raise ValueError('duplicate or negative CSV frame')
        seen.add(frame)
        if not args.start_frame <= frame <= args.end_frame:
            continue
        raw_s = raw.get('rail_s_px', '').strip()
        s = float(raw_s) if raw_s else None
        # 无效帧可以为空/NaN，不可作为有效样本；保留null让模型加载可重算。
        if s is not None and not math.isfinite(s):
            s = None
        row_id = (raw.get('rail_binding_id') or '').strip()
        if row_id not in ('', bid):
            raise ValueError('CSV frame binding mismatch')
        row = dict(frame=frame, rail_s_px=s, rail_binding_id=row_id,
                   predicted=bool_flag(raw['predicted']) if prediction_contract == 'column' else None)
        for key in ('pixel_valid', 'reference_valid', 'calibration_valid', 'cm_valid'):
            row[key] = bool_flag(raw[key])
        rows.append(row)
    data = dict(schema='rail-sampling-session-v1', binding=copy.deepcopy(binding), rail_binding_id=bid,
                known_position=True, cm=args.cm, start_frame=args.start_frame, end_frame=args.end_frame,
                prediction_contract=prediction_contract, rows=sorted(rows, key=lambda r: r['frame']),
                source=dict(video_path=str(video), video_sha256=video_sha, csv_path=str(csv_path), csv_sha256=csv_sha,
                            binding_path=str(binding_path.resolve()), binding_sha256=sha256(binding_path)),
                confirmation='operator --known-position: physical object held at the stated cm for the full inclusive interval')
    info = session_values(data, binding)
    data['sampling_summary'] = {k: v for k, v in info.items() if k != 'values'}
    if sha256(csv_path) != csv_sha or sha256(video) != video_sha:
        raise ValueError('source evidence changed during collection')
    return data


def load_sessions(paths, binding):
    sessions, evidence = [], []
    for path in paths:
        path = Path(path).resolve()
        raw = path.read_bytes()
        session = json.loads(raw.decode('utf-8'))
        session_values(session, binding)
        sessions.append(session)
        evidence.append(dict(path=str(path), sha256=hashlib.sha256(raw).hexdigest()))
    return sessions, evidence


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command')
    collect_parser = sub.add_parser('collect', help='确认实物位置后从geometry CSV采样')
    collect_parser.add_argument('--csv', required=True)
    collect_parser.add_argument('--video', required=True)
    collect_parser.add_argument('--known-position', action='store_true', required=True)
    collect_parser.add_argument('--cm', required=True, type=float)
    collect_parser.add_argument('--start-frame', required=True, type=int)
    collect_parser.add_argument('--end-frame', required=True, type=int)
    fit_parser = sub.add_parser('fit', help='只拟合；尚不可输出厘米')
    fit_parser.add_argument('--sessions', nargs='+', required=True)
    fit_parser.add_argument('--method', choices=('linear', 'piecewise'), default='linear')
    verify_parser = sub.add_parser('verify', help='用独立位置/独立区间验证并创建新模型版本')
    verify_parser.add_argument('--model', required=True)
    verify_parser.add_argument('--sessions', nargs='+', required=True)
    for cmd in (collect_parser, fit_parser, verify_parser):
        cmd.add_argument('--binding-file', '--binding', dest='binding_file', required=cmd is not collect_parser)
        cmd.add_argument('--output', required=True, help='必须不存在的新JSON文件路径')
    args = parser.parse_args(argv)
    if args.command is None:
        parser.error('choose collect, fit or verify')
    binding_path = Path(args.binding_file) if args.binding_file else Path(args.csv).with_suffix('.binding.json')
    binding_hash = sha256(binding_path)
    binding = read_binding(binding_path)
    if args.command == 'collect':
        result = collect(args, binding, binding_path)
    else:
        sessions, sources = load_sessions(args.sessions, binding)
        if args.command == 'fit':
            result = build_model(binding, sessions, method=args.method)
            result['session_files'] = sources
        else:
            parent = Path(args.model).resolve()
            parent_raw = parent.read_bytes()
            previous = checked_model(json.loads(parent_raw.decode('utf-8')), binding)
            # 本次显式提供完整独立验证集合；历史验证保留在不可变parent文件中。
            result = build_model(binding, previous['fit_sessions'], sessions, previous['method'])
            result['parent'] = dict(path=str(parent), sha256=hashlib.sha256(parent_raw).hexdigest())
            result['verification_session_files'] = sources
    if sha256(binding_path) != binding_hash:
        raise ValueError('binding sidecar changed during operation')
    result['created_utc'] = datetime.now(timezone.utc).isoformat()
    save_new(args.output, result)
    if args.command == 'collect':
        summary = result['sampling_summary']
        print('session: %d/%d samples, coverage=%.4f -> %s' %
              (summary['samples'], summary['total_frames'], summary['coverage'], args.output))
    else:
        print('ready=%s validated=%s -> %s' % (result['ready'], result['validated'], args.output))
    return result


if __name__ == '__main__':
    main()
