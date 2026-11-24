"""汇总附加快球片段、跨场景留出，以及无球挡具的阴性对照。"""
import argparse
from pathlib import Path
from statistics import median

from score import MODES, TRUTH, frame_rows, raw_inputs, reentries, rows, save


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace', type=Path, help='含 7/ 与 8/ 的实拍录像根目录')
    parser.add_argument('out', type=Path)
    args = parser.parse_args()
    logs = args.workspace/'8/fusion_20261122/controls'
    records = []
    for scene, base in [('moving_fast_confirmed', '7/motion_20261121/runs'),
                        ('moving_free_new', '8/continuity_20261122/runs')]:
        _, clocks, scheduled, truth, logs_by_mode = raw_inputs(
            scene, args.workspace/base, TRUTH, logs/(scene+'.csv'), False, MODES)
        data = frame_rows(scene, clocks, scheduled, truth, logs_by_mode)
        for mode in MODES:
            group = [r for r in data if r['mode'] == mode]
            chosen = [r for r in group if r['scheduled']]
            inside = [r for r in chosen if r['label'] == 'inside']
            events = reentries(scene, mode, group)
            latency = [e['latency_ms'] for e in events if e['status'] == 'observed']
            records.append(dict(scene=scene, mode=mode, captured=len(clocks), processed=len(chosen),
                                inside=len(inside), correct=sum(r['correct'] for r in inside),
                                offset=sum(r['valid'] and not r['correct'] for r in inside),
                                outside_valid=sum(r['label']=='outside' and r['valid'] for r in chosen),
                                returns=len(events), recovered=len(latency),
                                median_ms=median(latency) if latency else None))
    args.out.mkdir(parents=True, exist_ok=True)
    save(args.out/'challenge.csv', records)
    empty = rows(logs/'tool_only.csv')
    save(args.out/'tool_only.csv', [
        dict(mode=m, **{k:sum(int(r[k] or 0) for r in empty if r['mode']==m)
                        for k in ('processed','rail_valid','hough_count','contour_count','motion_count','pixel_valid')})
        for m in MODES])


if __name__ == '__main__':
    main()
