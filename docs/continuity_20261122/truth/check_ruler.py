"""用独立的 SIFT 尺纹特征复核等距原帧及全部边界帧的水平 ROI 端点。"""
import argparse
import csv
import json
import sys
from pathlib import Path

import cv2 as cv
import numpy as np
from label_raw import JETSON, HERE, RUNS, decode, jpeg_frames, ruler_features


def check(root, out, scenes=RUNS, name='ruler_crosscheck.csv', truth_dir=None):
    truth_dir = truth_dir or out
    cv.setNumThreads(1)
    cfg = json.loads((JETSON/"config_motion_ruler_digits_20261121.json").read_text())
    _, ends = ruler_features(cfg)
    ref = cfg["rail"]["upper_reference"]
    profile = np.asarray(ref["upper_edge_points"])
    # 仅在参考图的纸尺印刷面提特征。原图搜索带更宽，容许姿态变化。
    ref_gray = cv.imread(str(JETSON/ref["image"]), cv.IMREAD_GRAYSCALE)
    ys, xs = np.indices(ref_gray.shape)
    edge = np.interp(xs, *profile.T)
    mask = ((xs > 240) & (xs < 1115) & (ys > edge+3) & (ys < edge+45)).astype("uint8")*255
    sift = cv.SIFT_create()
    kp, des = sift.detectAndCompute(ref_gray[530:], mask[530:])
    matcher = cv.BFMatcher()
    with (out/name).open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(["run", "frame", "inliers", "inlier_x_span_px", "left_delta_px", "right_delta_px"])
        for run in scenes:
            with (truth_dir/(run+".csv")).open() as file:
                rows = list(csv.DictReader(file))
            for i, jpg in enumerate(jpeg_frames(root/run/"raw.mjpg")):
                if i%30 and rows[i]["label"] != "border" and i != 894:
                    continue
                pts, fd = sift.detectAndCompute(decode(jpg)[530:], None)
                pairs = [a for a,b in matcher.knnMatch(des,fd,k=2) if a.distance < .72*b.distance]
                src = np.float32([kp[m.queryIdx].pt for m in pairs])
                dst = np.float32([pts[m.trainIdx].pt for m in pairs])
                if len(src) < 4:
                    writer.writerow([run,i,0,0,"",""])
                    continue
                src[:,1] += 530
                dst[:,1] += 530
                mat, keep = cv.estimateAffinePartial2D(src,dst,ransacReprojThreshold=2.5)
                if mat is None:
                    writer.writerow([run,i,0,0,"",""])
                    continue
                good = keep.ravel().astype(bool)
                span = np.ptp(src[good,0])
                delta = (ends @ mat[:,:2].T + mat[:,2])[:,0] - [
                    float(rows[i]["roi_left_px"]), float(rows[i]["roi_right_px"])]
                writer.writerow([run,i,good.sum(),round(float(span),1),*np.round(delta,2)])


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs",type=Path);parser.add_argument("out",type=Path,nargs="?",default=HERE)
    parser.add_argument("--scene",action="append");parser.add_argument("--name",default="ruler_crosscheck.csv")
    parser.add_argument("--truth",type=Path)
    a=parser.parse_args();check(a.runs,a.out,a.scene or RUNS,a.name,a.truth)
