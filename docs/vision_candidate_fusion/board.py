"""用采集时钟统计实板速度；离线回放按同一帧号、同一版本与 PC 核对。"""
import argparse
import csv
import math
from pathlib import Path

import numpy as np


def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def summarize(samples):
    result = {}
    for field in ('total_ms', 'extra_ms', 'read_to_output_ms'):
        values = [float(r[field]) for r in samples if r[field]]
        if values:
            result[field+'_mean'] = round(float(np.mean(values)), 3)
            result[field+'_p95'] = round(float(np.percentile(values, 95)), 3)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('data', type=Path, help='本次 fusion_20261122 实验目录')
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    board = args.data/'nano'
    scene = 'moving_final'
    truth = {r['frame']: r for r in rows(Path(__file__).resolve().parents[1]/
             'continuity_20261122/truth'/ (scene+'.csv'))}
    pc = rows(args.data/'logs'/(scene+'.csv'))
    result = []
    for mode in 'AC':
        samples = [r for r in rows(board/(scene+'_'+mode+'.csv')) if r['processed']=='1']
        comparison = {r['frame']:r for r in pc if r['mode']==mode and r['processed']=='1'}
        assert {r['frame'] for r in samples} == set(comparison)
        inside = correct = outside_valid = different = 0
        delta = []
        for r in samples:
            original = comparison[r['frame']]
            assert r['timestamp']==original['timestamp']
            different += r['pixel_valid'] != original['pixel_valid']
            t = truth[r['frame']]
            inside += t['label']=='inside'
            if r['pixel_valid']=='1':
                x, y = float(r['x_px']), float(r['y_px'])
                if original['pixel_valid']=='1':
                    delta.append(math.hypot(x-float(original['x_px']),y-float(original['y_px'])))
                outside_valid += t['label']=='outside'
                correct += t['label']=='inside' and math.hypot(
                    x-float(t['center_x_px']), y-float(t['center_y_px'])) <= 12
        result.append(dict(scene=scene, mode=mode, processed=len(samples), captured=len(truth),
                           inside=inside, correct=correct, outside_valid=outside_valid,
                           pc_validity_differences=different, pc_max_xy_delta_px=max(delta, default=None),
                           **summarize(samples)))
    for run in sorted(board.glob('live_*')):
        if not run.is_dir():
            continue
        samples = rows(run/'measurements.csv')
        clocks = rows(run/'frames.csv')
        duration = float(clocks[-1]['read_s'])-float(clocks[0]['read_s'])
        measured = [float(r['timestamp']) for r in samples if r['pixel_valid']=='1']
        result.append(dict(scene=run.name, mode=samples[0]['mode'], captured=len(clocks),
                           processed=len(samples), valid=len(measured), duration_s=round(duration,3),
                           capture_fps=round((len(clocks)-1)/duration,3),
                           processed_hz=round(len(samples)/duration,3),
                           valid_hz=round(len(measured)/duration,3),
                           rail_valid=sum(r['rail_valid']=='1' for r in samples),
                           max_valid_interval_ms=round(max(np.diff(measured))*1000,3) if len(measured)>1 else None,
                           **summarize(samples)))
    with args.output.open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for r in result for k in r)))
        writer.writeheader()
        writer.writerows(result)


if __name__ == '__main__':
    main()
