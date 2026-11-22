"""只修正原采样节点中、照片纸唇落在原搜索带以外的点。"""
import json

import numpy as np
from photo_points import JETSON, photo_points


def corrected(config):
    # 参考照片曲线和原有几何窗口唯一确定修正范围，运行时仍必须逐帧验证。
    ref = config['rail']['upper_reference']
    photo = np.array(photo_points())
    for point in ref['upper_edge_points']:
        observed = float(np.interp(point[0], photo[:, 0], photo[:, 1]))
        if abs(observed - point[1]) > ref['max_edge_adjust_px']:
            point[1] = round(observed, 3)
    return config


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--create', action='store_true')
    args = parser.parse_args()
    for mode in ['points','print']:
        cfg = corrected(json.loads((JETSON / f'config_motion_left_{mode}_20261121.json').read_text()))
        candidate = JETSON / f'config_motion_left_{mode}_edgeknot_20261122.json'
        if args.create:
            with candidate.open('x') as f:
                f.write(json.dumps(cfg, indent=2, ensure_ascii=False) + '\n')
        assert cfg == json.loads(candidate.read_text())
        print(candidate)
