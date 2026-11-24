"""离板逐帧取证。以 python3 -B 运行，所有输出位于本脚本目录。"""
from pathlib import Path
import sys, csv, json, hashlib, collections
import cv2, numpy as np
JET=Path('/home/user/h_contest_vision_repository/Jetson')
ROOT=Path('/home/user/桌面/jetson_nano/7/motion_20261121')
RUN=ROOT/'runs/fixed_O_tilt'
OUT=ROOT/'tilt_diagnosis'
sys.path.insert(0,str(JET))
from motion_observe import replay, decoded
from local_vision import UpperEdgeReference
from ruler_reference import RulerReference, similarity_from_points
cv2.setNumThreads(1)
cfg=json.loads((JET/'config_motion_topview_20261121.json').read_text())['rail']['upper_reference']
ref=cv2.imread(str(JET/cfg['image']),0)
boxes=cfg['boxes']
from rail_detector import RailDetector
from PIL import Image, ImageDraw, ImageFont

assert Path(__file__).resolve().parent == OUT
EXPECTED = {
    'config_motion_topview_20261121.json': 'bdd42b08fa7676e909b7d0861c068c5131945a150163c1a6e7c9dc998575287e',
    'ruler_reference.py': '25f7901f3b7410a300d02d27439526e20813eb48d71ae9220b3eba2563cb5f06',
    'local_vision.py': 'caab6cd1428bf1b1677111c281e3b4ce0d53ddf7c066bda5e71356a72cc59f32',
    'motion_observe.py': 'f833e3e55f0018a2bb1de524200d3d65cebfa9f7cdb0a8bdc7a3e80248870c31',
    'rail_detector.py': 'f4c246b96236e177e61e5e2e1d9f90bf2c831728ffefb867a50829b31ed32a1d',
    cfg['image']: cfg['sha256']
}
for filename, digest in EXPECTED.items():
    assert hashlib.sha256((JET/filename).read_bytes()).hexdigest() == digest, filename

# 四个对照窗与三个生产模板和三个候选模板均不重叠。各自在全窗独立搜索。
CHECK = [(302,632,70,18),(488,649,105,28),(758,661,110,17),(989,626,89,40)]
ALT = [(390,640,82,38),(639,659,104,19),(901,635,71,31)]
ORIGIN = np.array([649.883,642.087])
LIMIT = 96
def records(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    by_id = {int(r['frame']):r for r in rows}
    assert len(rows) == len(by_id)
    return by_id

def write_table(name, rows):
    with (OUT/name).open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

def center(box):
    x,y,w,h=box
    return np.array([x+w/2,y+h/2])

def peaks(g, box):
    x,y,w,h=box
    x0,y0=max(0,x-LIMIT),max(0,y-LIMIT)
    wnd=g[y0:min(g.shape[0],y+h+LIMIT),x0:min(g.shape[1],x+w+LIMIT)]
    template=ref[y:y+h,x:x+w]
    response=cv2.matchTemplate(wnd,template,cv2.TM_CCOEFF_NORMED)
    _,score,_,(u,v)=cv2.minMaxLoc(response)
    response[max(0,v-2):v+3,max(0,u-2):u+3]=-1
    _,second,_,(u2,v2)=cv2.minMaxLoc(response)
    xx,yy=x0+u,y0+v
    cur=g[yy:yy+h,xx:xx+w]
    return dict(score=score,margin=score-second,dx=xx-x,dy=yy-y,
                runner_dx=x0+u2-x,runner_dy=y0+v2-y,
                mean_shift=float(cur.mean()-template.mean()),std_ratio=float(cur.std()/template.std()),
                white_pct=float(100*(cur>=250).mean()),
                top_delta=float(cur[:h//3].mean()-template[:h//3].mean()),
                bottom_delta=float(cur[-(h//3):].mean()-template[-(h//3):].mean()))

def fit(boxes, peaks):
    points=[center(b) for b in boxes]
    current=[c+[v['dx'],v['dy']] for c,v in zip(points,peaks)]
    return similarity_from_points(points,current,2.5)

def mapped(pose,points):
    inverse,translation=pose
    return np.asarray(points).dot(np.linalg.inv(inverse).T)+translation

def candidate(g, evidence, metrics, proposal):
    # 仅诊断用的兜底：原始三峰须唯一且成形，至少一处分数强；
    # 候选三块须分别通过原门限、三点残差和上缘判定，还须回证原三峰。
    if proposal is None or min(v['margin'] for v in metrics)<.05 or max(v['score'] for v in metrics)<.85:
        return None
    alt=evidence.locate(g)
    if alt is None:return None
    pose=evidence.matcher.transform
    orig=np.array([center(b)+[p['dx'],p['dy']] for b,p in zip(boxes,metrics)])
    if np.max(np.linalg.norm(mapped(pose,[center(b) for b in boxes])-orig,axis=1))>2.5:
        return None
    return pose

def new_alt():
    # 独立冷启动，禁止从连续帧的小窗身份继承获益。自适应仅供完整可见的旋转复核。
    return UpperEdgeReference(dict(cfg,boxes=ALT))

def geometry_check(g, tracker, raw, row):
    pose,info=fit(boxes,raw)
    row.update(proposal_pass=info['valid'],angle_deg=info['image_rotation_deg'],
               scale=info['scale'],residual=info['residual_px'])
    # 即使原算法会提前拒绝，也检查另外两块，区分首个拒绝和全部瓶颈。
    rect=[]
    for box in tracker.patches:
        sc=ma=None
        if pose is not None and max(v['score'] for v in raw)>=.85:
            window=tracker._visible_rectified_search(g,box,np.linalg.inv(pose[0]),pose[1])
            if window is not None:
                img,_,mask=window
                sc,ma,_=tracker._peak(img,box[-1],mask)
        rect.append((sc,ma))
    for j,(sc,ma) in enumerate(rect,1):
        row['rect_score'+str(j)]=sc
        row['rect_margin'+str(j)]=ma
    chk=[peaks(g,b) for b in CHECK]
    trusted=[j for j,v in enumerate(chk) if v['score']>=.85 and v['margin']>=.05]
    row['witness_count']=len(trusted)
    actual=[center(CHECK[j])+[chk[j]['dx'],chk[j]['dy']] for j in trusted]
    def distance(t):
        if t is None or not trusted:return None
        pred=mapped(t,[center(CHECK[j]) for j in trusted])
        return float(np.max(np.linalg.norm(pred-actual,axis=1)))
    row['proposal_witness_max_px']=distance(pose)
    for j,v in enumerate(chk,1):
        for k in ('dx','dy','score','margin'):row['w'+str(j)+'_'+k]=v[k]
    return pose,distance

def add_reason(edge,gray):
    evidence=edge.locate(gray)
    return ('confirmed_upper_edge' if evidence is not None else edge.reason),evidence

def keyframes(images, plots):
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc'
    small=ImageFont.truetype(font,17,index=2)
    title=ImageFont.truetype(font,21,index=2)
    ids=[0,37,400,800]
    sheet=Image.new('RGB',(1190,190*(1+len(ids))+55),'white')
    draw=ImageDraw.Draw(sheet)
    draw.text((16,6),'固定 O 实拍  ·  同一原图裁区与冻结参考    绿框为独立核对窗，橙框为冻结模板',font=title,fill='black')
    for k,i in enumerate([None]+ids):
        g=ref if i is None else images[i]
        offset=45+190*k
        sampled=Image.fromarray(g[535:707,214:1140]).convert('RGB')
        overlay=ImageDraw.Draw(sampled)
        for b,color,p in [(b,'#30cbb0',q) for b,q in zip(CHECK,[None]*4 if i is None else plots[i][1])]+[(b,'#ff8e19',q) for b,q in zip(boxes,[None]*3 if i is None else plots[i][0])]:
            x,y,w,h=b
            if p is not None:x+=p['dx'];y+=p['dy']
            overlay.rectangle([x-214,y-535,x+w-215,y+h-536],outline=color,width=1)
        sheet.paste(sampled,(16,offset))
        label='冻结参考图' if i is None else '采集帧 '+str(i)
        draw.text((954,offset+8),label,font=title,fill='black')
        if i is not None:
            raw=plots[i][0]
            draw.text((954,offset+45),'原始三峰 NCC',font=small,fill='black')
            draw.text((954,offset+76),' / '.join('%.3f'%p['score'] for p in raw),font=small,fill='black')
            draw.text((954,offset+108),{'0':'复核分数不足','37':'三个原始峰均弱','400':'复核分数不足','800':'上缘确认通过'}[str(i)],font=small,fill='black')
    sheet.save(OUT/'typical_frames.png')
    # 同一位置的模板与实拍逐像素显示，上排为冻结，下排为按原峰位取出的区域。
    comp=Image.new('RGB',(1060,480),'white');d=ImageDraw.Draw(comp)
    d.text((10,5),'三块纹理放大 3 倍（最近邻）  ·  实拍按各自原始峰位裁取',font=title,fill='black')
    for r,i in enumerate([None,37,400,800]):
        d.text((10,65+103*r),'参考' if i is None else '帧 '+str(i),font=title,fill='black')
        for j,(x,y,w,h) in enumerate(boxes):
            if i is not None:
                m=plots[i][0][j];x+=m['dx'];y+=m['dy']
            p=(ref if i is None else images[i])[y:y+h,x:x+w]
            comp.paste(Image.fromarray(np.repeat(np.repeat(p,3,axis=0),3,axis=1)),(135+293*j,52+103*r))
    comp.save(OUT/'patches.png')

def main():
    live=records(RUN/'measurements.csv')
    offline=records(ROOT/'new_fixed_O_tilt/measurements.csv')
    live_edge=UpperEdgeReference(cfg)
    sequential=UpperEdgeReference(cfg)
    alt=new_alt()
    rows=[];photos={};plots={};neg=[]
    for i,ts,frame,g,_ in decoded(replay(RUN)):
        assert g is not None
        reason,ev=add_reason(sequential,g)
        src=live.get(i)
        lreason=lev=None
        if src:
            assert abs(float(src['timestamp'])-ts)<1e-6
            lreason,lev=add_reason(live_edge,g)
        row=dict(frame=i,read_s=ts,onboard=bool(src),board_reason=src['rail_reason'] if src else '',
                 subset_replay=lreason,all_replay=reason,archived_all_reason=offline[i]['rail_reason'])
        raw=[peaks(g,b) for b in boxes]
        for j,v in enumerate(raw,1):
            for k,value in v.items():row['p'+str(j)+'_'+k]=value
        pose,check=geometry_check(g,sequential.matcher,raw,row)
        row['published_witness_max_px']=check(live_edge.matcher.transform) if lev is not None else None
        alt.matcher.transform=None
        alternate=candidate(g,alt,raw,pose) if ev is None or (src and lev is None) else None
        row['alt_fallback']=alternate is not None
        row['alt_witness_max_px']=check(alternate)
        rows.append(row)
        if i in (0,37,400,800):
            photos[i]=g.copy()
            plots[i]=(raw,[peaks(g,b) for b in CHECK])
        if i%100==0:
            # 对本来含真尺的帧注入三种负例，分别考察遮挡、重复身份和中块非刚性移动。
            for name in ('occlusion','duplicate','nonrigid'):
                fake=g.copy()
                if name=='occlusion':fake[615:690,365:493]=127
                if name=='duplicate':fake[529:614,200:1150]=g[615:700,200:1150]
                if name=='nonrigid':fake[620:697,613:750]=g[609:686,613:750]
                m=[peaks(fake,b) for b in boxes]
                t,_=fit(boxes,m)
                cold=UpperEdgeReference(cfg)
                frozen_valid=cold.locate(fake) is not None
                alt.matcher.transform=None
                accepted=candidate(fake,alt,m,t) is not None
                neg.append(dict(frame=i,case=name,frozen_valid=frozen_valid,fallback_valid=accepted))
    assert len(rows)==len(offline) and set(live)<=set(offline)
    write_table('frames_evidence.csv',rows)
    write_table('challenges.csv',neg)
    keyframes(photos,plots)
    print('frames / processed / subset reason mismatches / full reason mismatches',
          len(rows),len(live),sum(r['onboard'] and r['board_reason']!=r['subset_replay'] for r in rows),
          sum(r['all_replay']!=r['archived_all_reason'] for r in rows))

if __name__=='__main__':
    main()
