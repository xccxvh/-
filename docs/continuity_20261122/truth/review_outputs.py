"""输出所有越过中心容差或处于边界/界外/未知的有效标记，供按原图目视判定对象身份。"""
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import cv2 as cv
import numpy as np
from scoreboard import HERE, PROFILES, read_index, flag


def review(runs, paired, out, scenes, tol, points_only=False):
    out.mkdir(parents=True,exist_ok=True)
    records=[]
    for scene in scenes:
        truth=read_index(HERE/(scene+".csv"))
        files={"live":runs/scene/"measurements.csv"}
        if paired:
            files.update({p:paired/p/(scene+".csv") for p in (("baseline","release_points") if points_only else PROFILES[1:])})
        cells=defaultdict(list)
        for name,path in files.items():
            for i,r in read_index(path).items():
                if not flag(r["pixel_valid"]):
                    continue
                t=truth[i]
                x,y=float(r["x_px"]),float(r["y_px"])
                err=math.hypot(x-float(t["center_x_px"]),y-float(t["center_y_px"])) if t["center_x_px"] else None
                if t["label"]=="inside" and err is not None and err<=tol:
                    continue
                key=(i,round(x),round(y))
                cells[key].append(dict(scene=scene,profile=name,frame=i,label=t["label"],
                                       x_px=x,y_px=y,error_px=round(err,2) if err is not None else ""))
        # 固定 O 曾有一帧缺尾标；按本批相机 JFIF 包头切帧，避免帧号错位。
        if not cells:
            continue
        marker=b"\xff\xd8\xff\xe0"
        chunks=(runs/scene/"raw.mjpg").read_bytes().split(marker)[1:]
        assert len(chunks)==len(truth)
        tiles=[]
        for tile_id,((i,x,y),group) in enumerate(sorted(cells.items())):
            im=cv.imdecode(np.frombuffer(marker+chunks[i],np.uint8),1)
            # 左裁片以真值为中心，画出真值和输出。右裁片以输出为中心，显示其实际落点。
            t=truth[i]
            tx=round(float(t["center_x_px"] or x));ty=round(float(t["center_y_px"] or y))
            cv.drawMarker(im,(tx,ty),(60,240,60),cv.MARKER_CROSS,7,1)
            cv.drawMarker(im,(x,y),(255,30,240),cv.MARKER_TILTED_CROSS,9,1)
            crops=[]
            for cx,cy in [(tx,ty),(x,y)]:
                patch=cv.getRectSubPix(im,(70,70),(float(cx),float(cy)))
                crops.append(patch)
            tile=cv.copyMakeBorder(np.hstack(crops),0,16,0,0,cv.BORDER_CONSTANT)
            cv.putText(tile,f"{tile_id} #{i} {t['label']}",(2,82),0,.35,(255,255,255),1)
            tiles.append(tile)
            records.extend(dict(tile_id=tile_id,**r) for r in group)
        while len(tiles)%8:
            tiles.append(np.zeros_like(tiles[0]))
        assert cv.imwrite(str(out/(scene+"_outputs.jpg")),np.vstack([np.hstack(tiles[j:j+8]) for j in range(0,len(tiles),8)]))
    with (out/"output_review.csv").open("w") as f:
        w=csv.DictWriter(f,fieldnames=["tile_id","scene","profile","frame","label","x_px","y_px","error_px"])
        w.writeheader();w.writerows(records)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("runs",type=Path);p.add_argument("scenes",nargs="+")
    p.add_argument("--paired",type=Path);p.add_argument("--out",type=Path,required=True)
    p.add_argument("--tol-px",type=float,default=12)
    p.add_argument("--points-only-comparison",action="store_true")
    a=p.parse_args();review(a.runs,a.paired,a.out,a.scenes,a.tol_px,a.points_only_comparison)
