"""按 Nano 单调时钟关联上一条遥测；保留两端原始时间与接收年龄，首帧前无遥测则留空。"""
import argparse
from bisect import bisect_right
import csv
from pathlib import Path


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def join(root, runs, output):
    rx_keys=('local_monotonic_ms','g3519_ms','telemetry_seq','beam_cmd_deg','servo_pwm_us','beam_estimated_deg',
             'control_state','telemetry_flags')
    coord_keys=('reference_valid','measurement_x_px','measurement_y_px','rail_s_px','rail_x_cm','rail_coordinate_status')
    fields=['run','frame','nano_read_monotonic_ms','rx_age_ms']+list(rx_keys)
    for mode in ('raw','points'):
        fields.extend(mode+'_'+k for k in coord_keys)
    with output.open('x',newline='') as stream:
        w=csv.DictWriter(stream,fields,lineterminator='\n')
        w.writeheader()
        for name in runs:
            d=root/name
            frames={int(r['frame']):r['nano_read_monotonic_ms'] for r in read(d/'frames.csv')} if (d/'frames.csv').exists() else {}
            rx=read(d/'telemetry.csv') if (d/'telemetry.csv').exists() else []
            stamps=[float(r['local_monotonic_ms']) for r in rx]
            a,b=read(d/'raw.csv'),read(d/'points.csv')
            assert len(a)==len(b)
            for i,(left,right) in enumerate(zip(a,b)):
                assert i==int(left['frame'])==int(right['frame'])
                clock=frames.get(i)
                out=dict(run=name,frame=i,nano_read_monotonic_ms=clock)
                if clock is not None:
                    j=bisect_right(stamps,float(clock))-1
                    if j>=0:
                        out.update({k:rx[j][k] for k in rx_keys})
                        out['rx_age_ms']=float(clock)-stamps[j]
                for mode,r in [('raw',left),('points',right)]:
                    out.update({mode+'_'+k:r[k] for k in coord_keys})
                w.writerow(out)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path)
    p.add_argument('manifest',type=Path)
    p.add_argument('output',type=Path)
    a=p.parse_args()
    join(a.root,[r['run'] for r in read(a.manifest)],a.output)
