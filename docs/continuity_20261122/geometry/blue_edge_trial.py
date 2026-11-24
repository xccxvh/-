"""单帧蓝通道上缘实验，只接受已通过灰度纸尺身份检查的几何失败帧。"""
import argparse
import copy
import json
from pathlib import Path

import cv2
import numpy as np

from edge_probe import column_evidence
from refit import old
from motion_observe import packets


def probe(frame, gray, detector, baseline, settings):
    if baseline.reason != 'upper_edge_geometry':
        return baseline
    # 独立复制匹配器。蓝色输入只用于上缘；另从本帧灰度全窗重新确认尺面身份。
    # 本实验的第二次检测不能继承上一帧的变换或覆盖正常检测器的跟踪状态。
    trial = copy.copy(detector)
    edge = trial.upper_reference = copy.copy(detector.upper_reference)
    matcher = edge.matcher = copy.copy(edge.matcher)
    matcher.transform = None
    matcher._last_rectified = False
    register_gray = matcher.update
    matcher.update = lambda _: register_gray(gray)
    trial.reference_sequence = baseline.reference_sequence - 1
    blue = np.ascontiguousarray(frame[:,:,0])
    rail = trial.detect(blue,force=True)
    if not rail.valid:
        return baseline
    # 用实测上缘对注册曲线做分区复核，避免仅局部挡具、纸尺刻线提供足够列数。
    inv,t = matcher.transform
    profile = edge.profile @ np.linalg.inv(inv).T + t
    _,supported,offset = column_evidence(blue,profile,settings['max_edge_adjust_px'])
    supported_near = supported & (abs(offset)<=2)
    if any(np.mean(zone)<.25 for zone in np.array_split(supported_near,3)):
        return baseline
    return rail


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--run',default='verify_m5_02')
    args = parser.parse_args()
    cfg = json.loads((old.JET/'config_motion_left_print_20261121.json').read_text())
    folder = old.HOST/'motion_20261121/runs'/args.run
    cfg['rail']['upper_reference']['image'] = str(old.JET/cfg['rail']['upper_reference']['image'])
    detector = old.RailDetector(cfg['rail'],1280,720)
    balls = old.BallDetector(dict(cfg['ball'],ball_search=cfg['ball_search']),1280,720)
    tracker = old.BallTracker(cfg['tracking'],1280,720)
    points = json.loads((old.STAGE/'config_board_points.json').read_text())['camera_geometry']
    coord = old.RailCoordinates(cfg,old.CameraGeometry(points,1280,720,old.JET),old.JET)
    assert coord.calibration is None
    cv2.setNumThreads(1)
    frames = old.read_csv(folder/'frames.csv')
    clock,_ = old.clip_clock(folder/'raw.mjpg',len(frames))
    stream = packets(['ffmpeg','-nostdin','-loglevel','error','-f','mjpeg','-i',
                      str(folder/'raw.mjpg'),'-c:v','copy','-f','mpjpeg','pipe:1'])
    rows = []
    for i,jpeg in enumerate(stream):
        img = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        baseline = detector.detect(gray,force=True)
        rail = probe(img,gray,detector,baseline,cfg['rail']['upper_reference'])
        candidates = balls.detect(img,gray,rail)
        track = tracker.update(candidates,clock[i])
        pos = coord.process(track,candidates,rail,i)
        assert not (pos['cm_valid'] or pos['feedback_valid'])
        if rail is not baseline:
            # 两个破坏性图像对照。沿用原灰度纹理但把蓝通道展平、或把灰度纹理清空。
            # 两种对照均需拒绝，结果只在本进程内存在，绝不生成运行时模型。
            flat = img.copy()
            flat[:,:,0] = 128
            assert not probe(flat,gray,detector,baseline,cfg['rail']['upper_reference']).valid
            assert not probe(img,np.zeros_like(gray),detector,baseline,cfg['rail']['upper_reference']).valid
        rows.append(dict(frame=i,baseline_rail=baseline.reason,blue_recovered=rail is not baseline,
                         candidates=len(candidates),tracker_reason=track['reason'],
                         status=pos['rail_coordinate_status'],axis_px=pos['rail_s_px'],
                         x_px=pos['measurement_x_px'],y_px=pos['measurement_y_px']))
    assert len(rows)==len(clock)
    old.write_rows(args.output, rows)


if __name__ == '__main__':
    main()
