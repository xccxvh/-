"""读取经现场确认的独立摆放清单，使用 Nano 逐帧记录比较两套绑定的线性标定。"""
import argparse
import csv
from collections import Counter
from itertools import groupby
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'Jetson'))
from calibrate_rail import collect, read_binding
from rail_calibration import build_model, map_position


def metrics(cm, values):
    # 缺测保留在分母和时间序列中；每次重新摆放分别验收，避免重复组相互稀释。
    errors = [abs(x-cm) for x in values if x is not None]
    missing = [sum(1 for _ in g) for is_missing,g in groupby(values,lambda x:x is None) if is_missing]
    cov = len(errors)/float(len(values))
    return dict(frames=len(values),valid=len(errors),coverage=cov,mae_cm=np.mean(errors) if errors else None,
                max_abs_cm=max(errors) if errors else None,longest_missing_frames=max(missing or [0]),
                passed=bool(errors and cov>=.95 and max(errors)<=.3+1e-12))


def analyze(root, manifest, dest):
    with manifest.open() as f:
        placements=list(csv.DictReader(f))
    # 只按操作者确认的位置和用途入组。缺少一轮时不给出整批验收结论。
    if any(p['role'] not in ('fit','verify') for p in placements):
        raise ValueError('unknown placement role')
    repeats=Counter(float(p['cm']) for p in placements if p['role']=='verify')
    if any(repeats[cm]<2 for cm in (-5,-3,3,5)):
        raise ValueError('two independent placements required at each held-out position')
    dest.mkdir(parents=True,exist_ok=False)
    summary=[]
    for mode in ('raw','points'):
        fit, check, observations=[],[],[]
        for p in placements:
            clip=root/p['run']
            truth=float(p['cm'])
            video=next(clip.glob('raw.m*'))
            log=clip/(mode+'.csv')
            side=clip/(mode+'.binding.json')
            binding=read_binding(side)
            with log.open() as f:
                data=list(csv.DictReader(f))
            assert [int(r['frame']) for r in data]==list(range(150))
            evidence=collect(SimpleNamespace(cm=truth,known_position=True,video=video,csv=log,
                start_frame=0,end_frame=149),binding,side)
            (fit if p['role']=='fit' else check).append(evidence)
            summary.append(dict(run=p['run'],role=p['role'],cm=truth,mode=mode,model='existing',
                **metrics(truth,[float(r['rail_x_cm']) if r['rail_x_cm'] else None for r in data])))
            observations.append((p,data))
        new=build_model(binding,fit,check)
        (dest/(mode+'_model.json')).write_text(json.dumps(new,separators=(',',':'))+'\n')
        for p,data in observations:
            cm=float(p['cm'])
            # 未通过整体验证的模型只能离线诊断，不能把候选模型值送往控制。
            values=[map_position(new['model'],float(r['rail_s_px'])) if r['rail_s_px'] else None for r in data]
            summary.append(dict(run=p['run'],role=p['role'],cm=cm,mode=mode,model='expanded_offline',
                                **metrics(cm,values)))
        print(mode,'model validation:',new['validated'],'each independent placement:',
              all(r['passed'] for r in summary if r['mode']==mode and r['role']=='verify' and r['model']=='expanded_offline'))
    with (dest/'placements.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,list(summary[0]),lineterminator='\n')
        w.writeheader()
        w.writerows(summary)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('root','manifest','dest'):
        p.add_argument(name,type=Path)
    args=p.parse_args()
    analyze(args.root,args.manifest,args.dest)
