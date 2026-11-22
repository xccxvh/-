"""在本机重新解码原片，复用旧评估器，保留每次独立摆放和当帧姿态。"""
import argparse
import csv
import json
import os
import sys
import tempfile
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
CAL = ROOT / 'docs/motion_vision_20261121/calibration'
sys.path.insert(0, str(CAL))
import replay as old


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--config', type=Path, default=old.JET / 'config_motion_left_print_20261121.json')
    args = parser.parse_args()
    out = args.output
    out.mkdir()
    cv2.setNumThreads(1)
    cfg = json.loads(args.config.read_text())
    assert cfg['rail_coordinates']['calibration_file'] is None
    ref = cfg['rail']['upper_reference']
    ref['image'] = str((args.config.resolve().parent / ref['image']).resolve())
    points = json.loads((old.STAGE / 'config_board_points.json').read_text())['camera_geometry']
    assert all(points[k] == cfg['camera_geometry'][k] for k in ('camera_id', 'focus_id'))
    runs = old.HOST / 'motion_20261121/runs'
    # 错放两段保留现场角色、原始哈希和全部帧，原评估函数按角色排除。
    rows = old.read_csv(CAL / 'motion_left_print_20261121_v1/statistics.csv')
    clips = [dict(run=r['run'], role=r['role'], cm=int(r['cm']) if r['cm'] else None,
                  sha256=r['video_sha256'], field=True, count=int(r['frames']),
                  nominal_30hz=r['run'] == 'fit_m6') for r in rows if r['mode'] == 'raw']
    tilt = json.loads((ROOT / 'docs/motion_vision_20261121/fixed_O_tilt/capture.json').read_text())
    clips.append(dict(run='fixed_O_tilt', role='unlabelled', cm=None, count=895,
                      field=True, sha256=tilt['files']['raw.mjpg']['sha256']))
    sessions = {m: [] for m in ('raw', 'points')}
    summaries = []
    # 对实际源码、配置、图像及采集时钟取摘要，检测共享工作区在回放期间的变动。
    paths = [*old.JET.glob('*.py'), args.config, Path(ref['image']), old.JET / points['intrinsics_file'],
             old.STAGE / 'config_board_points.json', CAL / 'replay.py']
    paths += [runs/c['run']/'frames.csv' for c in clips if (runs/c['run']/'frames.csv').exists()]
    hashes = {str(p): old.sha256(p) for p in paths}
    with tempfile.TemporaryDirectory(dir=out) as scratch:
        for c in clips:
            c['video'] = runs / c['run'] / 'raw.mjpg'
            assert old.sha256(c['video']) == c['sha256'], c['run']
            reports, pair, _ = old.replay(c, cfg, points, Path(scratch))
            summaries.extend(reports)
            for mode, session in pair.items():
                sessions[mode].append((c['role'], c['run'], session))
            # 保留可重算的姿态、坐标、拒绝原因；实物角度只有独立测量才可填写。
            for mode in sessions:
                log = Path(scratch)/c['run']/(mode+'.csv')
                data = old.read_csv(log)
                assert all(r['cm_valid']=='False' and r['feedback_valid']=='False' for r in data)
                if mode == 'points' and c['run'] in ('fixed_O_tilt','fit_zero','verify_zero','verify_m5_02'):
                    (out/(c['run']+'.csv')).write_bytes(log.read_bytes())
            print(c['run'], [(r['mode'], r['samples']) for r in reports], flush=True)
        for mode in sessions:
            assert len({r['replay_binding'] for r in summaries if r['mode']==mode}) == 1
        assert all(old.sha256(p) == digest for p,digest in hashes.items())
        result = [old.evaluate(mode, ss, out, False) for mode, ss in sessions.items()]
    old.json_out(out/'evaluation.json', result)
    old.json_out(out/'provenance.json', dict(config=str(args.config), files=hashes,
                                           opencv=cv2.__version__, numpy=old.np.__version__))
    old.write_rows(out/'placements.csv', summaries)
    old.write_rows(out/'heldouts.csv', [dict(mode=m['mode'], **r) for m in result
                   for r in [*m['placements'], *[dict(z,round='zero') for z in m['zero_checks']]]])


if __name__ == '__main__':
    main()
