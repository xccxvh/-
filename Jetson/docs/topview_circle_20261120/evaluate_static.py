from collections import Counter
import argparse, csv, hashlib, itertools, json
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
from rail_detector import RailDetector
from ball_detector import BallCandidate
from local_vision import PhysicalSearch, relative_caps

parser = argparse.ArgumentParser(description='Audit four static clips against the frozen raw-frame labels.')
parser.add_argument('--clips', required=True, type=Path, help='geometry_comparison directory with static_truth.json')
parser.add_argument('--logs', required=True, type=Path, help='one new Nano CSV per position')
parser.add_argument('--output', required=True, type=Path)
args = parser.parse_args()
jet = Path(__file__).resolve().parents[2]
base = args.clips
out = args.output
out.mkdir(exist_ok=True)
oldroot=jet/'docs/topview_geometry_20261120'
truth=json.loads((base/'static_truth.json').read_text())
previous=json.loads((oldroot/'static_results.json').read_text())
config=json.loads((jet/'config_geometry_topview_raw_20261120.json').read_text())

def read(path):
    with path.open() as f:return list(csv.DictReader(f))

def longest(flags):
    return max([sum(1 for _ in g) for k,g in itertools.groupby(flags) if k] or [0])

def strip(im,header):
    im=im.copy();cv2.putText(im,header,(5,14),cv2.FONT_HERSHEY_SIMPLEX,.44,(30,240,255),1,cv2.LINE_AA)
    return im

rows=[];stats={};examples=[];failures=[]
for name,entry in truth.items():
    video=base/entry['path'];sha=hashlib.sha256(video.read_bytes()).hexdigest()
    assert sha==previous[name]['sha256']
    actual=[r for r in read(oldroot/(name+'_pairs.csv')) if r['mode']=='raw']
    log,=args.logs.glob('track_'+name+'_*.csv')
    data=read(log);assert len(data)==len(actual)==120
    cap=cv2.VideoCapture(str(video));invalid=[];all_x=[];errors=[];sample_rows=[]

    for i,(a,n) in enumerate(zip(actual,data)):
        ok,f=cap.read();assert ok and int(n['frame'])==int(a['frame'])==i
        assert n['cm_valid']=='0' and n['feedback_valid']=='0'
        tx,ty=int(a['ball_raw_x']),int(a['ball_raw_y'])
        x=y=None;verdict='miss'
        if n['valid']=='1':
            x=float(n['x_px'])
            chosen=[d for d in json.loads(n['candidate_diagnostics']) if abs(d['center_px']-x)<.006]
            assert len(chosen)==1
            y=chosen[0]['center_y_px']
            verdict='correct' if np.hypot(x-tx,y-ty)<=entry['radius']+1 else 'false'
            assert n['predicted']=='0'
            if verdict=='correct':
                all_x.append(x);errors.append(x-tx)
        invalid.append(verdict!='correct')
        row=dict(position=name,frame=i,truth_x=tx,truth_y=ty,old=a['identity'],new=verdict,
                 circle_x=x,circle_y=y,rail_valid=n['rail_valid'],roi_min=n['roi_min_px'],roi_max=n['roi_max_px'],
                 candidates=n['candidate_count'],reason=n['tracker_reason'],detector=n['detector_reason'],
                 rail_s_px=n['rail_s_px'],cm_valid=n['cm_valid'])
        rows.append(row);sample_rows.append(row)
        if i!=60 and (name,i) not in [('O',0),('minus5',69),('plus5',97)]:continue
        if i==60:
            g=cv2.cvtColor(f,cv2.COLOR_BGR2GRAY);r=RailDetector(config['rail'],1280,720).detect(g,force=True)
            bounds=PhysicalSearch(config['ball_search']).bounds(r)
            masks=[];cc=cv2.connectedComponentsWithStats
            def capture(mask,*args):
                masks.append(mask.copy());return cc(mask,*args)
            with patch.object(cv2,'connectedComponentsWithStats',capture):
                relative_caps(f,r,config['ball']['relative_cap'],bounds,BallCandidate,[])
            left=max(0,int(np.ceil(bounds[0]))-67);right=min(1280,int(np.floor(bounds[1]))+68)
            top=max(0,int(np.floor(r.edge_profile[left:right].min()-48)))
            full=np.zeros_like(g);h,w=masks[0].shape;full[top:top+h,left:left+w]=255*masks[0]
            crop=(slice(ty-55,ty+49),slice(tx-190,tx+160))
            original=f.copy()
            old=actual[i]
            if old['valid']=='True':
                v=round(float(old['x']));cv2.line(original,(v,ty-35),(v,ty+40),(20,20,255),2)
            new=f.copy()
            if x is not None:
                cv2.drawMarker(new,(round(x),round(y)),(20,255,20),cv2.MARKER_CROSS,18,2)
            examples.append(np.hstack([strip(original[crop],name+' raw / old'),strip(cv2.cvtColor(full[crop],cv2.COLOR_GRAY2BGR),'old Lab mask'),strip(new[crop],'topview circle')]))
        else:
            crop=f[ty-60:ty+50,tx-170:tx+170]
            failures.append(strip(crop,'%s #%d %s'%(name,i,n['detector_reason'])))
    assert not cap.read()[0];cap.release()
    first=next(i for i,z in enumerate(invalid) if not z)
    stats[name]=dict(frames=len(data),original_sha256=sha,old=Counter(a['identity'] for a in actual),
                     new=Counter(r['new'] for r in sample_rows),reference=sum(r['reference_valid']=='1' for r in data),
                     first_valid_frame=first,max_missed_frames=longest(invalid),post_lock_max_missed_frames=longest(invalid[first:]),
                     jitter_x_std_px=float(np.std(all_x)),mean_abs_x_err_px=float(np.mean(np.abs(errors))),
                     max_abs_x_err_px=float(np.max(np.abs(errors))),rail_coordinate_frames=sum(bool(r['rail_s_px']) for r in data))
with (out/'frames.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
(out/'scores.json').write_text(json.dumps(stats,indent=2)+'\n')
cv2.imwrite(str(out/'raw_mask_result.jpg'),np.vstack(examples))
cv2.imwrite(str(out/'misses.jpg'),np.hstack(failures))
for k,s in stats.items():print(k,s['new'],s['max_missed_frames'],s['post_lock_max_missed_frames'],round(s['jitter_x_std_px'],2),round(s['mean_abs_x_err_px'],2),round(s['max_abs_x_err_px'],2))
