"""同一 JPEG 解码一次，交替执行新旧算法。输出 CSV 和 JSON 仅在本目录。"""
from pathlib import Path
import csv, json, sys, time, argparse, hashlib, importlib.util
from collections import Counter
import cv2
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
JET=ROOT/'frozen_v1/Jetson'
sys.path.insert(0,str(JET))
from motion_observe import decoded,replay
from rail_detector import RailDetector
from ball_detector import BallDetector
from ball_tracker import BallTracker
from camera_geometry import CameraGeometry
from rail_coordinates import RailCoordinates
from ruler_reference import RulerReference as Original

spec=importlib.util.spec_from_file_location('candidate_ruler',str(HERE/'ruler_reference.py'))
candidate=importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)
profile=JET/'config_motion_ruler_digits_20261121.json'
assert hashlib.sha256(profile.read_bytes()).hexdigest()=='56dde81e57550aea1ecf77d7965437a1120bb3e1d79af0167d6ff3fd8a904669'
assert hashlib.sha256((JET/'ruler_reference.py').read_bytes()).hexdigest()=='25f7901f3b7410a300d02d27439526e20813eb48d71ae9220b3eba2563cb5f06'
cfg=json.loads(profile.read_text())
r=cfg['rail']['upper_reference']
ref=cv2.imread(str(JET/r['image']),0)
cv2.setNumThreads(1)

def pipeline(cls,workers=3):
    rd=RailDetector(cfg['rail'],1280,720)
    rd.upper_reference.matcher=cls(ref,r['boxes'],r['search_x'],r['search_y'],
                                   rotation_recheck=r['rotation_recheck'],adaptive_search=r['adaptive_search'],
                                   **({'search_workers':workers} if cls is not Original else {}))
    geom=CameraGeometry(cfg['camera_geometry'],1280,720,JET)
    return (rd,BallDetector(dict(cfg['ball'],ball_search=cfg['ball_search']),1280,720),
            BallTracker(cfg['tracking'],1280,720),RailCoordinates(cfg,geom,JET))

def run(pipe, im, g, ts, i, full):
    rd,bd,bt,rc=pipe
    start=time.perf_counter()
    rail=rd.detect(g,force=True)
    ms=1000*(time.perf_counter()-start)
    reg=getattr(rail,'registration',{})
    d=dict(ms=ms,valid=rail.valid,reason=rail.reason,mode=reg.get('registration_mode',''),
           rotated=reg.get('rotation_rechecked',False),ox=None,oy=None,residual=reg.get('residual_px'),
           pixel=None,s=None,x=None,px_reason=None)
    if rail.valid:
        d['ox'],d['oy']=np.linalg.solve(rail.reference_inverse,cfg['rail_coordinates']['origin_px'])+rail.reference_translation
    if full:
        balls=bd.detect(im,g,rail)
        tracked=bt.update(balls,ts)
        out=rc.process(tracked,balls,rail,i)
        d.update(pixel=out['rail_s_px'] is not None,s=out['rail_s_px'],x=out['measurement_x_px'],px_reason=out['rail_coordinate_status'])
    return d,reg

def stats(v):
    return [round(float(x),4) for x in np.percentile(v,[50,95,100])] if v else []
def summary(rows):
    a={'n':len(rows)}
    for side in ('old','new'):
        a[side]=dict(valid=sum(x[side+'_valid'] for x in rows),
                     pixels=sum(bool(x[side+'_pixel']) for x in rows),
                     modes=dict(Counter(x[side+'_mode'] for x in rows)),
                     ms=stats([x[side+'_ms'] for x in rows]),
                     mean_ms=round(np.mean([x[side+'_ms'] for x in rows]),4))
    a['rail_gains_losses']=[sum(x['new_valid'] and not x['old_valid'] for x in rows),
                            sum(x['old_valid'] and not x['new_valid'] for x in rows)]
    a['pixel_gains_losses']=[sum(bool(x['new_pixel']) and not x['old_pixel'] for x in rows),
                             sum(bool(x['old_pixel']) and not x['new_pixel'] for x in rows)]
    a['origin_delta']=stats([np.hypot(x['new_ox']-x['old_ox'],x['new_oy']-x['old_oy']) for x in rows if x['new_valid'] and x['old_valid']])
    a['point_delta']=stats([x['max_point_delta'] for x in rows if x['max_point_delta'] is not None])
    a['rail_s_delta']=stats([abs(x['new_s']-x['old_s']) for x in rows if x['new_pixel'] and x['old_pixel']])
    a['pixel_x_delta']=stats([abs(x['new_x']-x['old_x']) for x in rows if x['new_pixel'] and x['old_pixel']])
    a['old_archive_reason_mismatches']=sum(x['old_reason']!=x['archive'] for x in rows)
    return a

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--workers',type=int,default=3)
    parser.add_argument('--pilot',type=int,default=0)
    parser.add_argument('--scenes',nargs='+',default=['moving_free','parked_free','fixed_O_tilt','baseline','live_pixel'])
    args=parser.parse_args()
    for name in args.scenes:
        src=(ROOT.parent/'moving_check' if name in ('baseline','live_pixel') else ROOT/'runs')/name
        with (ROOT/('digits_'+name)/'measurements.csv').open() as f:archive={int(x['frame']):x for x in csv.DictReader(f)}
        board={}
        if name=='moving_free':
            with (src/'measurements.csv').open() as f:board={int(x['frame']):x for x in csv.DictReader(f)}
        systems={'all':[pipeline(Original),pipeline(candidate.RulerReference,args.workers)]}
        if board:systems['board366']=[pipeline(Original),pipeline(candidate.RulerReference,args.workers)]
        out={key:[] for key in systems}
        samples=decoded(replay(src))
        try:
            for i,ts,im,g,_ in samples:
                for key,pipes in systems.items():
                    if key=='board366' and i not in board:continue
                    log=archive[i] if key=='all' else board[i]
                    row=dict(frame=i,archive=log['rail_reason'],max_point_delta=None)
                    res={}
                    for j in ((0,1) if i%2==0 else (1,0)):
                        d,reg=run(pipes[j],im,g,ts,i,not args.pilot)
                        res[j]=reg
                        side=('old','new')[j]
                        row.update({side+'_'+k:v for k,v in d.items()})
                    if all(res.values()):
                        diff=np.array(res[0]['current_points'])-res[1]['current_points']
                        row['max_point_delta']=float(np.max(np.linalg.norm(diff,axis=1)))
                    out[key].append(row)
                if args.pilot and i+1>=args.pilot:break
        finally:
            samples.close()
        for key,rows in out.items():
            sub=summary(rows)
            print(name,key,json.dumps(sub),flush=True)
            if args.pilot:continue
            with (HERE/(name+'_'+key+'.csv')).open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
            (HERE/(name+'_'+key+'.json')).write_text(json.dumps(sub,indent=2))
if __name__=='__main__':main()
