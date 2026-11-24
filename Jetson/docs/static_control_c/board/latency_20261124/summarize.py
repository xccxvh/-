"""从一次正式入口的 track/link CSV 统计所有处理帧、真实 TX 及板端状态。"""
import csv
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np


def summarize(track):
    with track.open() as f:
        frames = list(csv.DictReader(f))
    with track.with_name(track.name.replace('track_', 'link_', 1)).open() as f:
        link = list(csv.DictReader(f))
    by_frame = {r['frame']: r for r in frames}
    tx = [r for r in link if r['event'] == 'tx' and r['frame'] in by_frame]
    valid = [r for r in tx if int(r['position_flags']) & 1]
    rx = [r for r in link if r['event'] == 'rx']
    def pct(values, q):
        return round(float(np.percentile(values, q)), 3) if len(values) else None
    ages = [float(r['uart_age_ms']) for r in frames]
    wire = [float(r['local_monotonic_ms']) - float(by_frame[r['frame']]['uart_frame_time_ms'])
            for r in valid]
    times = [float(r['local_monotonic_ms']) for r in valid]
    gaps = np.diff(times)
    elapsed = float(frames[-1]['video_time_s']) - float(frames[0]['video_time_s'])
    return dict(processed=len(frames), fps=round((len(frames)-1)/elapsed,2) if elapsed else None,
                reference=sum(r['reference_valid']=='1' for r in frames),
                cm=sum(r['cm_valid']=='1' for r in frames), valid_tx=len(valid),
                valid_pct=round(100.*len(valid)/len(frames),2),
                reasons=Counter(r['feedback_reason'] for r in frames),
                ref_p50_ms=pct([float(r['reference_match_ms']) for r in frames],50),
                algo_p50_ms=pct([float(r['algo_ms']) for r in frames],50),
                age_all_p50_ms=pct(ages,50), age_all_p95_ms=pct(ages,95),
                valid_tx_complete_p95_ms=pct(wire,95), valid_tx_complete_max_ms=pct(wire,100),
                valid_tx_gap_max_ms=pct(gaps,100),
                unpaired_tx=sum(r['event']=='tx' and bool(r['frame']) and r['frame'] not in by_frame for r in link),
                telemetry=len(rx), board_states=Counter(r['control_state'] for r in rx))


if __name__ == '__main__':
    for arg in sys.argv[1:]:
        print(Path(arg).name, json.dumps(summarize(Path(arg)), sort_keys=True))
