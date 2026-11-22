"""−5 名义摆放中逐帧定位球、纸尺和挡具，分辨扰动前后。"""
import csv
import json
import sys
from pathlib import Path

import cv2 as cv
import numpy as np
from label_raw import HERE,JETSON,jpeg_frames,decode,pose,ruler_features
from fresh_scene import sheet


def main(root):
    cv.setNumThreads(1)
    scene="fixed_m5_final"
    source=root/scene
    with (source/"frames.csv").open() as f:
        times=list(csv.DictReader(f))
    packets=list(jpeg_frames(source/"raw.mjpg"))
    assert len(packets)==len(times)
    patches,bounds=ruler_features(json.loads((JETSON/"config_motion_topview_20261121.json").read_text()))
    seeds=[(0,349,584),(465,403,592)]
    templates=[]
    for i,x,y in seeds:
        g=decode(packets[i]);templates.append(g[y-18:y+19,x-18:x+19].copy())
    first=decode(packets[0])
    paper=first[622:649,296:403]
    stop=first[549:580,191:272]
    rows=[]
    for i,j in enumerate(packets):
        assert int(times[i]["frame"])==i
        g=decode(j)
        scores=[cv.minMaxLoc(cv.matchTemplate(g[499:663,206:623],t,cv.TM_CCOEFF_NORMED)) for t in templates]
        _,peak,_,(u,v)=max(scores,key=lambda s:s[1])
        x,y=u+224,v+517
        _,paper_score,_,(px,py)=cv.minMaxLoc(cv.matchTemplate(g[607:664,281:418],paper,cv.TM_CCOEFF_NORMED))
        _,stop_score,_,(sx,sy)=cv.minMaxLoc(cv.matchTemplate(g[529:600,171:292],stop,cv.TM_CCOEFF_NORMED))
        left,right,n,rms=pose(g,patches,bounds)
        margin=min(x-left,right-x)
        assert peak>.54 and margin>25
        rows.append(dict(frame=i,read_s=times[i]["read_s"],label="inside",center_x_px=x,center_y_px=y,
                         template_ncc=round(peak,3),roi_left_px=round(left,2),roi_right_px=round(right,2),
                         ruler_inliers=n,ruler_rms_px=round(rms,2),signed_margin_px=round(margin,2),border_band_px=18,
                         ball_minus_ruler_dx=x-349-(px-15),ball_minus_ruler_dy=y-584-(py-15),
                         ball_minus_stop_dx=x-349-(sx-20),local_paper_ncc=round(paper_score,3),
                         local_stop_ncc=round(stop_score,3),
                         placement="initial_nominal_minus5" if i<410 else "post_disturbance",
                         nominal_cm=-5 if i<410 else ""))
    with (HERE/(scene+".csv")).open("w") as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    sheet(root,HERE,scene,rows)
    # 保留原始颜色、印刷 7 刻线和挡具，分别展示扰动前后及最大相对位移。
    (HERE/"final_board").mkdir(exist_ok=True)
    tiles=[]
    for i in (0,409,410,426,450,455,465,600,735,894):
        im=cv.imdecode(np.frombuffer(packets[i],np.uint8),1)
        tile=im[531:666,206:557].copy()
        cv.putText(tile,f"#{i}  relative x {rows[i]['ball_minus_ruler_dx']:+d}px",(5,16),0,.43,(0,255,255),1)
        tiles.append(tile)
    assert cv.imwrite(str(HERE/"final_board"/"fixed_m5_marker.jpg"),np.vstack([np.hstack(tiles[j:j+2]) for j in range(0,len(tiles),2)]))


if __name__=="__main__":
    main(Path(sys.argv[1]))
