"""实机统计以相机原始读帧时钟为分母，区分处理、定位和经验证厘米输出。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def analyze(path):
    clock = rows(path/'frames.csv')
    processed = rows(path/'measurements.csv')
    ids = [int(r['frame']) for r in processed]
    assert list(range(len(clock))) == [int(r['frame']) for r in clock]
    assert sorted(set(ids)) == ids and ids[-1] < len(clock)
    assert all(abs(float(clock[i]['read_s'])-float(r['timestamp'])) < 1e-6
               for i,r in zip(ids,processed))
    times = [float(r['read_s']) for r in clock]
    span = times[-1]-times[0]
    good = [r for r in processed if r['pixel_valid']=='True']
    cm = [r for r in processed if r['cm_valid']=='True']
    assert all(r['pixel_valid']=='True' and r['x_cm'] != '' for r in cm)
    assert all(r['cm_valid']=='True' or r['x_cm']=='' for r in processed)
    # 读帧时间给出覆盖整个视频的空档；输出时间给出真实相邻有效输出间隔。
    out = [float(r['result_s']) for r in good]
    cm_out = [float(r['result_s']) for r in cm]
    ts = [float(r['timestamp']) for r in good]
    latency = [float(r['read_to_output_ms']) for r in processed if r['read_to_output_ms']]
    return dict(scene=path.name,captured=len(clock),processed=len(processed),seconds=span,
                skipped=len(clock)-len(processed),reference=sum(r['reference_valid']=='True' for r in processed),
                pixel=len(good),cm=len(cm),captured_hz=(len(clock)-1)/span,
                processed_hz=len(processed)/span,pixel_hz=len(good)/span, cm_hz=len(cm)/span,
                longest_cm_output_gap_s=float(max(np.diff(cm_out))) if len(cm_out)>1 else None,
                longest_raw_gap_s=max(np.diff([times[0]]+ts+[times[-1]])),
                longest_valid_output_gap_s=float(max(np.diff(out))) if len(out)>1 else None,
                read_to_output_p95_ms=float(np.percentile(latency,95)) if latency else None,
                mean_and_p95_ms={k:[float(np.mean(v)),float(np.percentile(v,95))]
                                 for k in ('decode_ms','reference_and_edge_ms','detector_ms','tracker_ms','coordinates_ms')
                                 for v in [[float(r[k]) for r in processed if r[k]]] if v},
                raw_sha256=hashlib.sha256((path/'raw.mjpg').read_bytes()).hexdigest())


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path,nargs='+')
    a = p.parse_args()
    print(json.dumps([analyze(run) for run in a.capture],indent=2,allow_nan=False))
