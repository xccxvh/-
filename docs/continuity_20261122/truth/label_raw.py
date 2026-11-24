"""从原始 MJPEG 构建离线逐帧人工取样模板标注；仅使用原片、采集时钟和参考尺几何。"""
import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

import cv2 as cv
import numpy as np


RUNS = ("moving_fast_confirmed", "parked_free", "moving_free")
# 在原始全幅图上目视指定的钢珠中心；不同位置、照明、拖影和端挡遮挡各有实例。
SEEDS = {
    RUNS[0]: {0: (250, 566), 30: (524, 603), 170: (182, 552), 280: (120, 537)},
    RUNS[1]: {0: (84, 519), 27: (98, 537), 37: (129, 553),
              460: (493, 610), 650: (104, 535)},
    RUNS[2]: {30: (140, 554), 60: (415, 605)},
}
HERE = Path(__file__).resolve().parent
JETSON = HERE.parents[2] / "Jetson"


def jpeg_frames(path):
    # 相机在 JPEG 末尾有填充字节。按 JPEG 标记顺序遍历，避免对裸 MJPEG 随机跳帧。
    data = path.read_bytes()
    pos = 0
    while True:
        pos = data.find(b"\xff\xd8\xff", pos)
        if pos < 0:
            return
        end = data.index(b"\xff\xd9", pos) + 2
        yield data[pos:end]
        pos = end


def decode(jpg):
    return cv.imdecode(np.frombuffer(jpg, np.uint8), cv.IMREAD_GRAYSCALE)


def exemplars(root):
    templates = []
    for run, seeds in SEEDS.items():
        for frame, jpg in enumerate(jpeg_frames(root / run / "raw.mjpg")):
            if frame in seeds:
                x, y = seeds[frame]
                patch = decode(jpg)[y-24:y+25, x-24:x+25]
                templates.append(cv.resize(patch, (25, 25), interpolation=cv.INTER_AREA))
            if frame >= max(seeds):
                break
    return templates


def ruler_features(config):
    ref = config["rail"]["upper_reference"]
    path = JETSON / ref["image"]
    assert hashlib.sha256(path.read_bytes()).hexdigest() == ref["sha256"]
    gray = cv.imread(str(path), cv.IMREAD_GRAYSCALE)
    profile = np.asarray(ref["upper_edge_points"])
    # 沿整段尺取数字和刻度，不含轨道内的钢珠；两端边界来自相同机位的搜索锚点。
    anchors = np.asarray(config["ball_search"]["anchors"])
    endpoints = anchors[[0, -1], 1]
    bounds = np.column_stack([endpoints, np.interp(endpoints, *profile.T)])
    patches = []
    for x in range(280, 1081, 66):
        y = round(np.interp(x, *profile.T)) + 23
        patches.append((x, y, gray[y-12:y+13, x-23:x+24]))
    return patches, bounds


def pose(gray, patches, bounds):
    src, dst = [], []
    for x, y, template in patches:
        # 局部平移匹配后由跨尺共识拟合二维相似变换，剔除周期刻纹的错误峰。
        x0, y0 = x - 23 - 47, y - 12 - 50
        patch = gray[y0:min(gray.shape[0], y+63), x0:x+71]
        scores = cv.matchTemplate(patch, template, cv.TM_CCOEFF_NORMED)
        _, score, _, (u, v) = cv.minMaxLoc(scores)
        if score >= .61:
            src.append((x, y))
            dst.append((u+x0+23, v+y0+12))
    if len(src) < 4:
        return None
    src, dst = np.float32(src), np.float32(dst)
    mat, keep = cv.estimateAffinePartial2D(src, dst, ransacReprojThreshold=3.2)
    if mat is None:
        return None
    good = keep.ravel().astype(bool)
    if good.sum() < 4 or np.ptp(src[good, 0]) < 350:
        return None
    err = np.linalg.norm(src[good] @ mat[:, :2].T + mat[:, 2] - dst[good], axis=1)
    left, right = (bounds @ mat[:, :2].T + mat[:, 2])[:, 0]
    return left, right, int(good.sum()), float(np.sqrt(np.mean(err**2)))


def ball(gray, templates):
    # 搜全轨含两端挡，候选不受待判断的 ROI 限制。每帧独立，停球和折返也可定位。
    small = cv.resize(gray[460:670], (640, 105), interpolation=cv.INTER_AREA)
    yy, xx = np.indices((81, 616))
    x = (xx + 12)*2 + .5
    y = 460 + (yy + 12)*2 + .5
    band = (x > 48) & (x < 1230) & (abs(y - (615 - .00024*(x-700)**2)) < 55)
    matches = []
    for template in templates:
        score = cv.matchTemplate(small, template, cv.TM_CCOEFF_NORMED)
        score[~band] = -2
        _, peak, _, (u, v) = cv.minMaxLoc(score)
        matches.append((peak, x[v, u], y[v, u]))
    matches.sort(reverse=True)
    peak, bx, by = matches[0]
    close = [m for m in matches if m[0] > peak-.075 and np.hypot(m[1]-bx, m[2]-by) < 14]
    bx, by = np.median(np.asarray(close)[:, 1:], axis=0)
    return bx, by, peak


def run_labels(root, out, run, templates, patches, bounds):
    source = root/run
    with (source/"frames.csv").open() as stream:
        times = list(csv.DictReader(stream))
    rows = []
    for i, jpg in enumerate(jpeg_frames(source/"raw.mjpg")):
        assert int(times[i]["frame"]) == i
        gray = decode(jpg)
        assert gray.shape == (720, 1280)
        x, y, score = ball(gray, templates)
        geom = pose(gray, patches, bounds)
        row = dict(frame=i, read_s=times[i]["read_s"], center_x_px=x, center_y_px=y,
                   template_ncc=round(score, 3), roi_left_px="", roi_right_px="",
                   ruler_inliers="", ruler_rms_px="", signed_margin_px="", border_band_px="",
                   label="uncertain")
        if geom is not None:
            l, r, n, rms = geom
            margin = min(x-l, r-x)
            band = (8 if score >= .78 else 13) + int(np.ceil(rms))
            # 行驶自由段采用 18 px 下限，覆盖复核中两种尺纹配准的分歧和球心复读差。
            if run == "moving_free" or run not in RUNS:
                band = max(band, 18)
            label = "inside" if margin > band else "outside" if margin < -band else "border"
            row.update(roi_left_px=round(l, 2), roi_right_px=round(r, 2), ruler_inliers=n,
                       ruler_rms_px=round(rms, 2), signed_margin_px=round(margin, 2),
                       border_band_px=band, label=label)
        if score < .55:
            row.update(center_x_px="", center_y_px="", signed_margin_px="", label="uncertain")
        rows.append(row)
    assert rows and len(rows) == len(times)
    if run in RUNS:
        assert len(rows) == 895
    with (out/(run+".csv")).open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    print(run, Counter(row["label"] for row in rows), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path)
    parser.add_argument("--out", type=Path, default=HERE)
    parser.add_argument("--fresh", help="同机位自由球新场景名，只处理该段")
    parser.add_argument("--seed-runs", type=Path, help="保存原始模板帧的本次三场景 runs 目录")
    args = parser.parse_args()
    cv.setNumThreads(1)
    config = json.loads((JETSON/"config_motion_ruler_digits_20261121.json").read_text())
    patches, bounds = ruler_features(config)
    templates = exemplars(args.seed_runs or args.runs)
    args.out.mkdir(parents=True, exist_ok=True)
    for run in ([args.fresh] if args.fresh else RUNS):
        run_labels(args.runs, args.out, run, templates, patches, bounds)


if __name__ == "__main__":
    main()
