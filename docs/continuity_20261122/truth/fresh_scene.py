"""新景先看原始轨道，再用历史模板标注，输出全帧球心裁片供复核。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2 as cv
import numpy as np
from label_raw import HERE, JETSON, exemplars, jpeg_frames, ruler_features, run_labels


def sheet(root, out, scene, labels=None):
    with (root/scene/"frames.csv").open() as f:
        times = list(csv.DictReader(f))
    count = len(times)
    selected = set(np.linspace(0,count-1,min(18,count),dtype=int))
    panels = []
    cols = 32
    if labels is not None:
        grid = np.zeros(((count+cols-1)//cols*42,cols*42,3),np.uint8)
        # 加入图像置信分最低和距离边界最近的帧；框线让复核者能检查错锁。
        selected.update(sorted(range(count),key=lambda i:float(labels[i]["template_ncc"]))[:4])
        selected.update(sorted(range(count),key=lambda i:abs(float(labels[i]["signed_margin_px"] or "inf")))[:4])
    decoded = 0
    for i, jpg in enumerate(jpeg_frames(root/scene/"raw.mjpg")):
        assert i < count and int(times[i]["frame"]) == i
        decoded += 1
        if i not in selected and labels is None:
            continue
        image = cv.imdecode(np.frombuffer(jpg,np.uint8),1)
        if labels is not None:
            r = labels[i]
            if r["center_x_px"]:
                x,y = round(float(r["center_x_px"])),round(float(r["center_y_px"]))
                tile = cv.resize(image[y-32:y+32,x-32:x+32],(42,42))
                gy,gx = divmod(i,cols)
                grid[gy*42:(gy+1)*42,gx*42:(gx+1)*42]=tile
                cv.drawMarker(image,(x,y),(0,255,0),cv.MARKER_CROSS,12,1)
            for k in ("roi_left_px","roi_right_px"):
                if r[k]:
                    x=round(float(r[k]))
                    cv.line(image,(x,460),(x,685),(255,205,0),2)
        if i in selected:
            panel=cv.resize(image[410:710],(480,112))
            caption=f"#{i}" if labels is None else f"#{i}  {labels[i]['label']}"
            cv.putText(panel,caption,(5,16),0,.46,(0,255,255),1)
            panels.append(panel)
    assert decoded == count and count > 0
    while len(panels)%3:
        panels.append(np.zeros_like(panels[0]))
    name = "preview" if labels is None else "review"
    assert cv.imwrite(str(out/f"{scene}_{name}.jpg"),np.vstack([np.hstack(panels[j:j+3]) for j in range(0,len(panels),3)]))
    if labels is not None:
        assert cv.imwrite(str(out/f"{scene}_all.jpg"),grid)


def tool_occlusion(root, scene, rows):
    """此段工具横穿球体。按原片读的粗锚点限制复读窗口，再用当帧可见的球面匹配。"""
    src=root/scene/"raw.mjpg"
    known={"parked_free_new":"12161aca203e480759c81a093f1a0998a933cb8c8ae4b9db119e5139a7f44c39",
           "parked_final":"83ff6feca47ddcbb80667d898d920ade5e538a2b31e4cd76bbe76b4b00671e66"}
    if scene not in known or hashlib.sha256(src.read_bytes()).hexdigest()!=known[scene]:
        return
    groups=[{0:(134,545),12:(111,540),24:(110,537),52:(99,534),80:(103,536),
             105:(112,538),115:(121,542),134:(116,542),139:(123,543)},
            {382:(500,599),389:(501,597),393:(499,598),395:(485,599),
             403:(332,585),410:(195,558),415:(140,550)}]
    if scene=='parked_final':
        groups=[{0:(618,604),150:(618,604),168:(623,605),184:(622,605),190:(621,604),191:(617,604)}]
    packets=list(jpeg_frames(src))
    for anchors in groups:
        frames=list(anchors)
        points=np.array(list(anchors.values()))
        templates=[]
        for f,(x,y) in anchors.items():
            gray=cv.imdecode(np.frombuffer(packets[f],np.uint8),0)
            templates.append(gray[y-20:y+21,x-19:x+20].copy())
        for f in range(frames[0],frames[-1]+1):
            ax,ay=[round(np.interp(f,frames,a)) for a in points.T]
            gray=cv.imdecode(np.frombuffer(packets[f],np.uint8),0)
            patch=gray[ay-32:ay+33,ax-34:ax+35]
            best=(-2,None)
            for t in templates:
                _,peak,_,pos=cv.minMaxLoc(cv.matchTemplate(patch,t,cv.TM_CCOEFF_NORMED))
                if peak>best[0]:
                    best=(peak,pos)
            peak,(u,v)=best
            x,y=ax-15+u,ay-12+v
            r=rows[f]
            margin=min(x-float(r["roi_left_px"]),float(r["roi_right_px"])-x)
            band=float(r["border_band_px"])
            r.update(center_x_px=x,center_y_px=y,template_ncc=round(peak,3),signed_margin_px=round(margin,2),
                     label="inside" if margin>band else "outside" if margin < -band else "border")
            r["center_method"]="manual_window_template"
    for r in rows:
        r.setdefault("center_method","historical_templates")


def dual_stationary_ball(root,scene,rows):
    # 原片抽查发现七处右端塑料反光误配，左侧同一钢珠持续可见；逐帧重测左侧真球。
    src=root/scene/"raw.mjpg"
    if scene!="moving_dual_live" or hashlib.sha256(src.read_bytes()).hexdigest()!="d58ab2fe4bc38aecdacfdebe9f253cbb56962b0fa73edee1fe6073191caf0487":
        return
    for i,jpg in enumerate(jpeg_frames(src)):
        gray=cv.imdecode(np.frombuffer(jpg,np.uint8),0)
        if i==0:
            exemplar=gray[551:600,278:327].copy()
        _,peak,_,(u,v)=cv.minMaxLoc(cv.matchTemplate(gray[480:665,170:460],exemplar,cv.TM_CCOEFF_NORMED))
        x,y=194+u,504+v
        r=rows[i]
        m=min(x-float(r["roi_left_px"]),float(r["roi_right_px"])-x)
        assert peak>.8 and m>float(r["border_band_px"])
        r.update(center_x_px=x,center_y_px=y,template_ncc=round(peak,3),signed_margin_px=round(m,2),
                 label="inside",center_method="audited_left_ball_template")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action",choices=("preview","label"))
    parser.add_argument("runs",type=Path)
    parser.add_argument("scenes",nargs="+")
    parser.add_argument("--seed-runs",type=Path)
    parser.add_argument("--out",type=Path,default=HERE)
    args=parser.parse_args()
    cv.setNumThreads(1)
    args.out.mkdir(parents=True,exist_ok=True)
    if args.action=="label":
        if not args.seed_runs:
            parser.error("label requires --seed-runs for the audited historical images")
        cfg=json.loads((JETSON/"config_motion_ruler_digits_20261121.json").read_text())
        patches,bounds=ruler_features(cfg)
        templates=exemplars(args.seed_runs)
    for scene in args.scenes:
        rows=None
        if args.action=="label":
            run_labels(args.runs,args.out,scene,templates,patches,bounds)
            with (args.out/(scene+".csv")).open() as stream:
                rows=list(csv.DictReader(stream))
            tool_occlusion(args.runs,scene,rows)
            dual_stationary_ball(args.runs,scene,rows)
            if rows and "center_method" in rows[0]:
                with (args.out/(scene+".csv")).open("w") as f:
                    writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
        sheet(args.runs,args.out,scene,rows)


if __name__=="__main__":
    main()
