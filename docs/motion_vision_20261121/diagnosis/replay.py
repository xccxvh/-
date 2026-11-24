from pathlib import Path
import sys,csv,json,subprocess,time
import cv2,numpy as np
jet=Path('/home/user/h_contest_vision_repository/Jetson')
sys.path.insert(0,str(jet))
# 固定对照使用的轨沿模块，避免同一工作区其他并行任务改变参考定位。
import types
rev='67fe2a0ac2075924da52063d0ec517e109636c94'
def load_original(name):
    module=types.ModuleType(name)
    module.__file__=str(jet/(name+'.py'))
    exec(subprocess.check_output(['git','-C',str(jet),'show',rev+':Jetson/'+name+'.py']),module.__dict__)
    return module
sys.modules['ruler_reference']=load_original('ruler_reference')
from ball_detector import BallDetector,BallCandidate
from rail_detector import RailDetector
from ball_tracker import BallTracker
from rail_coordinates import RailCoordinates
from camera_geometry import CameraGeometry
import local_vision as lv
cv2.setNumThreads(1)
base=Path('/home/user/桌面/jetson_nano/7/moving_check')
dst=base.parent/'motion_ball_analysis'
cfg=json.loads((base/'config_moving_pixel.json').read_text())
cfg['rail']['upper_reference']['image']=str(base/'moving_reference.png')
def stream(name):
    data=(base/name/'raw.mjpg').read_bytes()
    pos=0
    for row in csv.DictReader((base/name/'frames.csv').open()):
        end=data.find(b'\xff\xd8\xff\xe0',pos+2)
        if end<0:end=len(data)
        assert data[pos:pos+2]==b'\xff\xd8'
        yield int(row['frame']),float(row['read_s']),np.frombuffer(data,np.uint8,end-pos,pos)
        pos=end
    assert pos==len(data)
def dets():
    return RailDetector(cfg['rail'],1280,720),BallDetector(dict(cfg['ball'],ball_search=cfg['ball_search']),1280,720)

# 用提交时的检测器在内存中做对照；原片按本批文件的 JPEG 起始标记分割，
# 即使某帧缺少 EOI 也不挪动后续帧号。在线子序列独立维护轨沿与跟踪状态。
original=load_original('local_vision')
lv.UpperEdgeReference=original.UpperEdgeReference
def write_csv(path,rows):
    with path.open('w') as fp:
        writer=csv.DictWriter(fp,rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

def evaluate(name,schedule):
    live={int(r['frame']):r for r in csv.DictReader((base/name/'live.csv').open())}
    rows=[]
    rd,bd=dets()
    geo=CameraGeometry(cfg['camera_geometry'],1280,720,jet)
    tracks=[BallTracker(cfg['tracking'],1280,720) for _ in range(2)]
    coords=[RailCoordinates(cfg,geo,base) for _ in range(2)]
    for i,t,j in stream(name):
        if schedule=='sampled' and i not in live:continue
        f=cv2.imdecode(j,1)
        assert f is not None,(name,i)
        g=cv2.cvtColor(f,cv2.COLOR_BGR2GRAY)
        r=rd.detect(g,force=True)
        b=bd.search.bounds(r)
        live_row=live.get(i,{})
        row=dict(frame=i,read_s=t,eoi=int(b'\xff\xd9' in j.tobytes()),rail=r.reason,
                 left=b[0] if b else '',right=b[1] if b else '',
                 logged_detector=live_row.get('detector_reason',''),logged_valid=live_row.get('pixel_valid',''))
        for k,(tag,fn) in enumerate([('old',original.topview_balls),('new',current)]):
            lv.topview_balls=fn
            start=time.perf_counter()
            c=bd.detect(f,g,r)
            elapsed=1000*(time.perf_counter()-start)
            o=tracks[k].update(c,t)
            m=coords[k].process(o,c,r,i)
            row.update({tag+'_reason':bd.last_reason,tag+'_xy':json.dumps([[round(z.x,2),round(z.extra['measurement_y_px'],2)] for z in c]),
                        tag+'_track':o.get('reason'),tag+'_valid':int(m['pixel_valid']),
                        tag+'_x':m['measurement_x_px'],tag+'_y':m['measurement_y_px'],tag+'_ms':round(elapsed,3)})
        rows.append(row)
    write_csv(dst/(name+'_'+schedule+'.csv'),rows)
    print(name,schedule,len(rows),[(k,sum(r[k+'_valid'] for r in rows),sum(r[k+'_xy']!='[]' for r in rows)) for k in ('old','new')],flush=True)
current=lv.topview_balls
if __name__=='__main__':
    for n in ('live_pixel','baseline'):
        evaluate(n,'all')
        if n=='live_pixel':evaluate(n,'sampled')
