#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""复现151窗审计实际执行的600组合扫描；不修改核心源码、不连接硬件。

运行：python3 -B docs/buffer_recovery_20261116/verify_partial_cap_sweep.py
依赖：当前环境的OpenCV、NumPy及下述仓库中的视觉模块。
本脚本默认读取当前仓库版本；打印源码哈希以便识别版本差异。
参考配准固定为单位变换；边缘支持仍由真实UpperEdgeReference.locate梯度路径生成。
仅扫描原审计的参数网格，不代表所有图像或全部误检场景。
"""
import hashlib
import sys
from pathlib import Path

# 禁止导入模块时写入__pycache__；该扫描只在内存中创建图像与诊断。
sys.dont_write_bytecode = True
REPO = Path(__file__).resolve().parents[2]
AUDITED_SOURCE_SHA256 = 'c968fc180802a7f2d9e84d25b8c95ada220bbee04b87feea8d71ee62392232f6'
sys.path.insert(0, str(REPO))

import cv2
import numpy as np
from local_vision import relative_caps, UpperEdgeReference
from rail_detector import RailLine
from ball_detector import BallCandidate


class M:
    # 仅固定参考配准，不伪造边缘支持；locate仍对合成图像计算Sobel证据。
    transform = (np.eye(2), np.zeros(2))

    def update(self, g):
        return {'valid': True, 'reason': 'ok'}


def main():
    actual_sha = hashlib.sha256((REPO / 'local_vision.py').read_bytes()).hexdigest()
    print('local_vision_sha256', actual_sha)
    print('matches_audited_source', actual_sha == AUDITED_SOURCE_SHA256)

    r = UpperEdgeReference.__new__(UpperEdgeReference)
    r.matcher = M()
    r.profile = np.array([[200., 200.], [320., 200.], [440., 200.]])
    r.max_adjust = 5
    r.support_margin = 80
    found = []
    total = 0

    # 与原审计完全相同的顺序与参数。每个rx分别有5/7/8个截断位置：
    # 2侧 * 5种ry * 3种区外距离 * (5+7+8) = 600。
    for side in [-1, 1]:
        for rx in [21, 26, 32]:
            for ry in [6, 7, 9, 13, 23]:
                for dist in [1, 3, 10]:
                    center = 200-dist if side == -1 else 440+dist
                    for delta in range(-rx+2, 1, 4):
                        cut = center+delta if side == -1 else center-delta
                        im = np.full((360, 640, 3), (190, 110, 40), np.uint8)
                        im[200:220] = 210
                        cv2.ellipse(im, (center, 201), (rx, ry), 0, 180, 360,
                                    (170, 170, 170), -1)
                        # 只擦除纸尺上方对应一侧球冠，保留纸尺及另一侧真实证据。
                        if side == -1:
                            im[:200, :cut] = (190, 110, 40)
                        else:
                            im[:200, cut+1:] = (190, 110, 40)

                        e = r.locate(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
                        rail = RailLine(e['slope'], e['intercept'], True)
                        rail.identity_valid = True
                        for k, v in e.items():
                            setattr(rail, k, v)
                        ds = []
                        cs, _ = relative_caps(im, rail, {}, (200, 440),
                                              BallCandidate, ds)
                        total += 1
                        if cs:
                            found.append((side, center, rx, ry, cut,
                                          [(c.x, c.width) for c in cs], ds))

    print('REAL_GRADIENT_FIXED_REGISTRATION_SWEEP total', total,
          'outside_accepted', len(found))
    for a in found[:12]:
        print(a)
    # 仅增加可复现脚本的退出状态；不改变原扫描组合或候选判定。
    assert total == 600, 'Unexpected scan count'
    return 1 if found else 0


if __name__ == '__main__':
    raise SystemExit(main())
