"""用相机原片帧号连接实时和完整离线输出。未处理、参考失效、无候选、跟踪拒绝分别计数。"""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path
import numpy as np


def read(path):
    with path.open() as f:
        return {int(r['frame']):r for r in csv.DictReader(f)}


def reason(row):
    if row is None:
        return 'skipped'
    if row['reference_valid'] != 'True':
        return row.get('rail_reason') or 'reference_invalid'
    if row.get('x_px') or row.get('measurement_x_px'):
        return 'pixel'
    if row['detector_reason'] != 'candidates':
        return row['detector_reason']
    status = row.get('coordinate_status') or row.get('rail_coordinate_status')
    if status and status != 'pixel_invalid':
        return status
    return row.get('tracker_reason') or 'tracker_or_contract'


def compare(args):
    captures = read(args.clock)
    logs = {key:read(Path(path)) for key,path in (spec.split('=',1) for spec in args.log)}
    # 统计分母包含完整采集时段；空档包含首帧到首次定位和最后定位到录像结束。
    times = [float(r['read_s']) for r in captures.values()]
    span = max(times)-min(times)
    metrics = {}
    fields = ['frame','read_s']
    for key, rows in logs.items():
        if set(rows)-set(captures):
            raise ValueError('measurement frame outside capture')
        reasons = Counter(reason(rows.get(i)) for i in captures)
        valid = [i for i in sorted(rows) if reason(rows[i])=='pixel']
        def gaps(stamps,start,end):
            edges = [start]+stamps+[end]
            return (max(np.diff(edges))*1000 if len(edges)>1 else 0)
        stamps = [float(captures[i]['read_s']) for i in valid]
        latency = {k:[float(r[k]) for r in rows.values() if r.get(k)] for k in (
            'decode_ms','reference_and_edge_ms','detector_ms','tracker_ms','coordinates_ms',
            'total_ms','read_to_output_ms')}
        m = dict(frames=len(captures),span_s=span,processed=len(rows),pixels=len(valid),
                 coverage_processed=len(valid)/max(1,len(rows)),processed_hz=len(rows)/span,
                 pixel_hz=len(valid)/span,longest_capture_gap_ms=gaps(stamps,min(times),max(times)),
                 counts=dict(reasons),stage_mean_p95_ms={k:[np.mean(v),np.percentile(v,95)]
                 for k,v in latency.items() if v})
        actual = [float(rows[i]['result_s']) for i in valid if rows[i].get('result_s')]
        if len(actual)>1:
            m['output_interval_p95_ms'] = np.percentile(np.diff(actual),95)*1000
            # 有效输出之间的最长间隔，不含启动和结束。
            m['max_output_interval_ms'] = max(np.diff(actual))*1000
        metrics[key] = m
        fields.extend(key+'_'+k for k in ('reason','x_px','y_px','x_cm','cm_valid'))
    args.dest.mkdir(parents=True,exist_ok=False)
    with (args.dest/'paired.csv').open('x') as f:
        writer = csv.DictWriter(f,fields)
        writer.writeheader()
        for i in sorted(captures):
            row = dict(frame=i,read_s=captures[i]['read_s'])
            for key,rows in logs.items():
                r = rows.get(i,{})
                vals = (reason(rows.get(i)),r.get('x_px',r.get('measurement_x_px')),
                        r.get('y_px',r.get('measurement_y_px')),r.get('x_cm',r.get('rail_x_cm')),
                        r.get('cm_valid','False'))
                row.update({key+'_'+k:v for k,v in zip(('reason','x_px','y_px','x_cm','cm_valid'),vals)})
            writer.writerow(row)
    (args.dest/'summary.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('clock',type=Path)
    p.add_argument('dest',type=Path)
    p.add_argument('log',nargs='+',help='名称=逐帧 CSV 文件')
    compare(p.parse_args())
