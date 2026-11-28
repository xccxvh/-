# -*- coding: utf-8 -*-
"""G3519 控制专用的仅超时丢帧规则，覆盖真实主循环的串口与日志分支。"""
import csv
import io
import json
import struct
import sys
import tempfile
import unittest
from contextlib import ExitStack, redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

import main
from position_feedback import G3519MonitorEncoder, G3519PositionEncoder, LinkJournal
from static_control_geometry import StaticControlCoordinates


GOOD = dict(valid=True, predicted=False, cm_valid=True, pixel_valid=True, reference_valid=True,
            calibration_valid=True, rail_x_cm=1.25, confidence=.9)


class StaleDropTests(unittest.TestCase):
    def test_remote_pwm_uses_local_admission_and_keeps_board_auto_invalid(self):
        enc = G3519PositionEncoder()
        enc.remote_pwm = True
        for stamp, measurement, verified, reason in (
                (4., GOOD, True, 'remote_observation'),
                (5., dict(GOOD, predicted=True), True, 'vision'),
                (6., dict(GOOD, reference_valid=False), True, 'pose'),
                (7., GOOD, False, 'unverified_capture_time')):
            packet, valid, why = enc.encode(measurement, stamp, stamp+.125, True, True, verified)
            _, cm, _, flags, _ = struct.unpack_from('<IfBBH', packet, 7)
            self.assertEqual((valid, why, flags), (False, reason, 2))
            self.assertEqual(cm, GOOD['rail_x_cm'] if why == 'remote_observation' else 0.)

    def test_age_boundary_and_sequence_wrap(self):
        enc = G3519PositionEncoder()
        enc.seq = 65535
        packet, valid, _ = enc.encode(GOOD, 2., 2.1, True, True, True)
        self.assertTrue(valid)
        self.assertEqual(struct.unpack_from('<H', packet, 5)[0], 65535)
        for ts in (3., 4.):
            self.assertEqual(enc.encode(GOOD, ts, ts+.100001, True, True, True),
                             (None, False, 'stale_only_drop'))
            self.assertEqual(enc.seq, 0)
        # 已跳过的真实曝光仍参加单调性判定，重复/倒退帧必须通知接收端。
        for seq, ts in enumerate((4., 3.5)):
            packet, valid, reason = enc.encode(GOOD, ts, 4.2, True, True, True)
            self.assertEqual((valid,reason,struct.unpack_from('<H',packet,5)[0]),(False,'duplicate',seq))
        packet, valid, _ = enc.encode(GOOD, 5., 5.01, True, True, True)
        self.assertTrue(valid)
        self.assertEqual(struct.unpack_from('<H',packet,5)[0],2)

    def test_age_cannot_mask_other_faults(self):
        cases = [({'valid': False},'vision'), ({'predicted': True},'vision'),
                 ({'reference_valid': False},'pose'), ({'pixel_valid': False},'calibration'),
                 ({'calibration_valid': False},'calibration'), ({'cm_valid': False},'calibration'),
                 ({'rail_x_cm': None},'calibration'), ({'rail_x_cm': float('nan')},'calibration'),
                 ({'rail_x_cm': 30.01},'position_range'), ({'rail_x_cm': -30.01},'position_range'),
                 ({'confidence': .194},'low_confidence'), ({'confidence': 0.},'low_confidence'),
                 ({'confidence': 1.5},'calibration'), ({'confidence': float('nan')},'calibration')]
        for age in (.02,.15):
            for change, expected in cases:
                with self.subTest(age=age,change=change):
                    enc=G3519PositionEncoder()
                    packet,valid,reason=enc.encode(dict(GOOD,**change),1.,1.+age,True,True,True)
                    self.assertEqual((valid,reason),(False,expected))
                    _,cm,conf,flags,_=struct.unpack_from('<IfBBH',packet,7)
                    self.assertEqual((cm,conf,flags & 1,enc.seq),(0.,0,0,1))
            enc=G3519PositionEncoder()
            self.assertEqual(enc.encode(GOOD,1.,1.+age,False,True,True)[2],'pose')
            # 板端以实收百分比判定，20 和两个坐标端点均可接受。
            for i,x in enumerate((-30.,30.)):
                packet,valid,reason=enc.encode(dict(GOOD,rail_x_cm=x,confidence=.196),2.+i,2.+i+age,True,True,True)
                self.assertEqual((valid,reason),(True,'ok') if age==.02 else (False,'stale_only_drop'))

    def test_unverified_or_bad_clock_transmits(self):
        enc=G3519PositionEncoder()
        # 未核验的时间要真发无效；后续倒退时间仍按原合同拒绝，恢复单调后可发送。
        packet,valid,reason=enc.encode(GOOD,12.,12.2,True,True,False)
        self.assertEqual((valid,reason),(False,'unverified_capture_time'))
        self.assertIsNotNone(packet)
        self.assertEqual(enc.encode(GOOD,11.99,12.05,True,True,True)[2],'duplicate')
        self.assertTrue(enc.encode(GOOD,12.1,12.15,True,True,True)[1])
        for ts,now in ((None,12.),(float('nan'),12.),(13.,12.),(12.,float('inf'))):
            packet,valid,reason=enc.encode(GOOD,ts,now,True,True,True)
            self.assertEqual((valid,reason),(False,'clock'))
            self.assertIsNotNone(packet)

    def test_monitor_still_transmits_every_frame(self):
        for encoder,enabled in ((G3519PositionEncoder(),False),(G3519MonitorEncoder(),True)):
            for seq,ts in enumerate((3.,4.)):
                packet,valid,reason=encoder.encode(GOOD,ts,ts+.15,True,enabled,True)
                self.assertEqual((valid,reason),(False,'stale'))
                self.assertEqual(struct.unpack_from('<H',packet,5)[0],seq)

    def test_main_logs_drop_and_really_sends_shutdown(self):
        # 三张已过期的图依次给出合格、低置信度、合格测量；最后一次跳过后停机。
        root=Path(main.BASE)
        cfg=json.loads((root/'config_static_control_c.json').read_text())
        camera = Mock(width=1280, height=720, fps_set=30, timestamp_verified=True,
                      frames_captured=0, skipped_for_age=0)
        image=np.zeros((720,1280,3),np.uint8)
        def read(fresh, **options):
            self.assertTrue(fresh)
            index=camera.read.call_count
            camera.last_ts=11.+index
            camera.last_capture_ts=10.+index
            if index > 3:
                return False, None
            camera.frames_captured += 1
            return True, image
        camera.read.side_effect=read
        detector=Mock(last_roi=None,last_diagnostics=[],last_reason='candidates')
        detector.detect.return_value=[SimpleNamespace(extra={})]
        tracker=Mock()
        tracker.update.side_effect=[dict(valid=True,confidence=c,predicted=False,method='topview_circle',
                                         state='TRACKED',x_px=350.) for c in (.9,.1,.9)]
        original=StaticControlCoordinates.process
        def coords(obj,out,cs,rail,frame_id):
            fields=original(obj,{},[],rail,frame_id)
            fields.update(cm_valid=True,calibration_valid=True,pixel_valid=True,reference_valid=True,
                          rail_coordinate_status='ok',rail_x_cm=1.25)
            return fields
        port=Mock(fd=42)
        with tempfile.TemporaryDirectory() as tmp, ExitStack() as stack:
            cfg['output']=dict(logs_dir=tmp,recordings_dir=tmp)
            for name,val in [('load_config',cfg),('Camera',camera),('BallDetector',detector),
                             ('BallTracker',tracker),('SerialFeedback',port)]:
                stack.enter_context(patch.object(main,name,return_value=val))
            stack.enter_context(patch.object(StaticControlCoordinates,'process',coords))
            stack.enter_context(patch.object(LinkJournal,'_receive',return_value=None))
            stack.enter_context(patch.object(sys,'argv',['main.py','--serial','memory','--uart-enable-valid']))
            # main 安装系统信号处理器；测试无需改变本进程后续测试的信号状态。
            stack.enter_context(patch.object(main.signal,'signal'))
            with redirect_stdout(io.StringIO()):
                main.main()
            def table(glob):
                with next(Path(tmp).glob(glob)).open() as stream:
                    return list(csv.DictReader(stream))
            track=table('track_*.csv')
            events=table('link_*.csv')
        self.assertEqual([r['feedback_reason'] for r in track],['stale_only_drop','low_confidence','stale_only_drop'])
        self.assertEqual([(r['event'],r['frame'],r['position_seq']) for r in events],
                         [('drop','0',''),('tx','1','0'),('drop','2',''),('tx','','1')])
        self.assertEqual((events[0]['capture_ms32'],events[0]['timestamp_verified'],events[0]['position_flags']),
                         ('11000','1',''))
        packets=[call[0][0] for call in port.send.call_args_list]
        self.assertEqual([struct.unpack_from('<H',p,5)[0] for p in packets],[0,1])
        self.assertEqual([struct.unpack_from('<IfBBH',p,7)[3] for p in packets],[2,0])
