"""逐帧复核六次上沿失败。纵剖面及负对照只用于分析，绝不传入检测器。"""
import csv
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

from photo_points import IMAGE, JETSON

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(Path(os.environ.get('MOTION_REPLAY_JETSON', JETSON)).resolve()))
from motion_observe import packets
from rail_detector import RailDetector

RUNS = Path.home() / '桌面/jetson_nano/7/motion_20261121/runs'
FOCUS = (0, 1, 2, 22, 23, 24)


def settings(name):
    return json.loads((JETSON / name).read_text())['rail']


def control_cases(config):
    gray = cv2.imread(str(IMAGE), cv2.IMREAD_GRAYSCALE)
    ref = config['upper_reference']
    profile = np.asarray(ref['upper_edge_points'])
    center = np.interp(np.arange(gray.shape[1]), *profile.T)
    dy = np.arange(gray.shape[0])[:, None] - center
    erased = gray.copy()
    erased[abs(dy) <= 10] = 128
    # 恢复三块原刻纹，让无边缘对照确实通过本帧的纹理匹配。
    for x,y,w,h in ref['boxes']:
        erased[y:y+h,x:x+w] = gray[y:y+h,x:x+w]
    foreign = erased.copy()
    foreign[abs(dy+20) < 2] = 255
    fragment = foreign.copy()
    x_domain = np.arange(gray.shape[1])
    fragment[(dy >= 0) & (dy < 3) & (x_domain > 450) & (x_domain < 600)] = 240
    dup = gray.copy()
    for x,y,w,h in ref['boxes']:
        dup[y-65:y+h-65,x:x+w] = gray[y:y+h,x:x+w]
    cases = [('intact',gray,True),('intact_print_erased_edge',erased,False),
             ('offband_bright_curve',foreign,False),('150_column_fragment',fragment,False),
             ('duplicate_print',dup,False),('blank',np.zeros_like(gray),False)]
    rows = []
    for name,img,expected in cases:
        d = RailDetector(config,1280,720)
        assert d.detect(gray, True).valid
        result = d.detect(img, True)
        assert result.valid == expected
        if not expected:
            assert not result.identity_valid and not result.geometry_valid and d.last is None
        if name in ('intact_print_erased_edge','offband_bright_curve','150_column_fragment'):
            assert result.reason == 'upper_edge_geometry'
        if name == 'duplicate_print':
            # 满搜索域中每块都有两处近乎等强峰，旋转复核可显式报告提案不一致。
            _, scores, margins = d.upper_reference.matcher._raw_peaks(img)
            assert min(scores) > .99 and max(margins) < .0001
            assert result.reason.startswith('reference_')
        rows.append(dict(control=name, valid=result.valid, reason=result.reason))
    return rows


def trace():
    cv2.setNumThreads(1)
    configs = {mode:settings('config_motion_left_print_'+suffix+'.json')
               for mode,suffix in [('original','20261121'),('knot','edgeknot_20261122')]}
    detectors = {mode:RailDetector(config,1280,720) for mode,config in configs.items()}
    rows = []
    stream = packets(['ffmpeg','-nostdin','-loglevel','error','-f','mjpeg','-i',
                      str(RUNS/'verify_m5_02/raw.mjpg'),'-c:v','copy','-f','mpjpeg','pipe:1'])
    for i,jpg in enumerate(stream):
        # 解码所有帧保持原时序，六个失败帧保留两组配置的逐列、逐行剖面。
        frame = cv2.imdecode(np.frombuffer(jpg,np.uint8),1)
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        for mode,d in detectors.items():
            line = d.detect(gray,True)
            if i not in FOCUS:
                continue
            edge = d.upper_reference
            inv,t = edge.matcher.transform
            p = edge.profile @ np.linalg.inv(inv).T + t
            x = np.arange(int(np.ceil(p[0,0])), int(np.floor(p[-1,0]))+1)
            expected = np.interp(x,*p.T)
            ys = np.floor(expected).astype(int) + np.arange(-5,7)[:,None]
            around = np.rint(expected).astype(int) + np.arange(-8,9)[:,None]
            r = dict(frame=i, config=mode, rail_reason=line.reason, domain=len(x))
            for name,img in [('gray',gray),('blue',frame[:,:,0]),('green',frame[:,:,1]),('red',frame[:,:,2])]:
                grad = cv2.Sobel(img,cv2.CV_32F,0,1)
                values = np.where(abs(ys-expected)<=5, grad[ys,x], -np.inf)
                supported = values.max(axis=0)>35
                r[name+'_supported'] = int(supported.sum())
                r[name+'_sectors'] = [round(float(np.mean(a)),3) for a in np.array_split(supported,6)]
                r[name+'_median_sobel_offsets_minus8_to_plus8'] = np.median(grad[around,x],axis=1).tolist()
            assert line.valid == (r['gray_supported'] >= .55 * len(x))
            rows.append(r)
    assert i == 149 and len(rows) == 12
    return rows, control_cases(configs['knot'])


if __name__ == '__main__':
    rows, controls = trace()
    for name, data in [('six_frame_profiles.csv',rows),('negative_controls.csv',controls)]:
        with (HERE/name).open('x',newline='') as out:
            w = csv.DictWriter(out,data[0],lineterminator='\n')
            w.writeheader()
            w.writerows(data)
