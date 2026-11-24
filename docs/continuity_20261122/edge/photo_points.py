"""仅从原始参考照片定位白纸上唇。运行本文件可复算并核对两个配置。"""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
JETSON = ROOT / 'Jetson'
IMAGE = JETSON / 'assets/topview_20261120/moving_reference.png'
DIGEST = '3c6da668646e530e87fc80368bd68fdef1be0c5f59344afd466ba7cc1505eff3'

# 逐点查看参考照片灰度纵剖面后标出的纸面上唇正梯度；规避钢珠及下方印刷刻线。
SEEDS = [(255,600),(303,611),(352,619),(431,629),(510,638),(559,642),
         (608,646),(657,649),(706,652),(746,651),(785,652),(825,651),
         (864,647),(903,644),(943,641),(982,635),(1022,631),(1061,623),(1111,615)]


def photo_points():
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest() == DIGEST
    gray = cv2.imread(str(IMAGE), cv2.IMREAD_GRAYSCALE)
    grad = cv2.Sobel(gray, cv2.CV_32F, 0, 1)
    sx, sy = np.array(SEEDS).T
    # 保留原有支持区两个端点。沿弯曲的上唇每隔16像素重新定位，横向中值抑制单刻线。
    knots = [255.019, *range(271, 1100, 16), 1110.614]
    points = []
    for knot in knots:
        cols = np.arange(round(knot)-8, round(knot)+9)
        expected = np.interp(cols, sx, sy)
        offsets = np.arange(-3,4)
        rows = np.rint(expected).astype(int) + offsets[:,None]
        strength = np.median(grad[rows,cols],axis=1)
        i = int(strength.argmax())
        # 未满足35的局部仅用照片中相邻标注间的插值，不采信更远的亮边。
        offset = offsets[i] if strength[i] > 35 else 0
        y = np.interp(knot, sx, sy) + offset
        points.append([knot, round(float(y),3)])
    return points


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--create', action='store_true')
    args = parser.parse_args()
    for mode in ['points','print']:
        baseline = JETSON / f'config_motion_left_{mode}_20261121.json'
        candidate = Path(__file__).resolve().parent / 'rejected_dense' / f'config_motion_left_{mode}_photoedge_20261122.json'
        config = json.loads(baseline.read_text())
        config['rail']['upper_reference']['upper_edge_points'] = photo_points()
        if args.create:
            with candidate.open('x') as f:
                f.write(json.dumps(config, indent=2, ensure_ascii=False) + '\n')
        assert config == json.loads(candidate.read_text())
        print(candidate.name, hashlib.sha256(candidate.read_bytes()).hexdigest())
