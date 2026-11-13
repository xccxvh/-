import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from calibration import Calibration
from rail_detector import RailDetector, RailLine
from position_feedback import FeedbackEncoder, SerialFeedback

class ReviewTests(unittest.TestCase):
    def test_numeric_strings_fail_closed(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'c.json'
            c=Calibration(str(path))
            for cm,x in zip((-5,0,5),(100,500,900)):c.add_point(cm,x,1280,720)
            c.fit();data=json.loads(path.read_text())
            data['model']['a']=str(data['model']['a']);path.write_text(json.dumps(data))
            self.assertFalse(Calibration(str(path)).ready)
    def test_unknown_coordinate_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'c.json';c=Calibration(str(path))
            for cm,x in zip((-5,0,5),(100,500,900)):c.add_point(cm,x,1280,720)
            c.fit();data=json.loads(path.read_text());data['coordinate']='unknown'
            path.write_text(json.dumps(data));self.assertFalse(Calibration(str(path)).ready)
    def test_pose_uses_unsmoothed_measurement(self):
        r=RailDetector({},1280,720);r.last=RailLine(0,330,True,.9)
        gray=np.zeros((720,1280),np.uint8);gray[340:]=200
        rail=r.detect(gray,force=True)
        with tempfile.TemporaryDirectory() as d:
            c=Calibration(str(Path(d)/'c.json'))
            for cm,x in zip((-5,0,5),(100,500,900)):
                c.add_point(cm,x,1280,720,evidence={'rail_slope':0,'rail_intercept':330})
            c.fit();self.assertFalse(c.pose_ok(rail))
    def test_future_does_not_poison_sequence(self):
        e=FeedbackEncoder();out={'valid':True,'predicted':False,'x_cm':0.,'confidence':1.}
        self.assertFalse(e.encode(out,100,1,True,True)[1])
        self.assertTrue(e.encode(out,2,2.01,True,True)[1])
    def test_extreme_timestamp_invalid(self):
        e=FeedbackEncoder()
        self.assertFalse(e.encode({},1e308,1e308,True,True)[1])
    def test_monitor_preserves_failure_reason(self):
        e=FeedbackEncoder()
        self.assertEqual(e.encode({'valid':False},1,1.01,True,False)[2],'vision')
    def test_unverified_sensor_time_never_control_valid(self):
        e=FeedbackEncoder()
        self.assertFalse(e.encode({'valid':True,'x_cm':0.,'confidence':1.},1,1.01,True,True,source_time_verified=False)[1])
    def test_short_writes_and_zero_write_close(self):
        s=SerialFeedback.__new__(SerialFeedback);s.fd=123
        with patch('position_feedback.select.select',return_value=([],[123],[])),patch('position_feedback.os.write',side_effect=[2,3]) as write:
            s.send(b'abcde');self.assertEqual(write.call_args_list[1][0][1],b'cde')
        with patch('position_feedback.select.select',return_value=([],[123],[])),patch('position_feedback.os.write',return_value=0),patch('position_feedback.os.close'):
            with self.assertRaises(OSError):s.send(b'a')
            self.assertIsNone(s.fd)

    def test_detector_init_failure_releases_camera(self):
        import main
        import sys
        from unittest.mock import MagicMock
        source=MagicMock(width=1280,height=720,fps_set=30)
        with tempfile.TemporaryDirectory() as d:
            cfg=json.loads((Path(main.BASE)/'config.json').read_text())
            cfg['output']={'logs_dir':d,'recordings_dir':d}
            with patch.object(sys,'argv',['main.py']),patch.object(main,'load_config',return_value=cfg),patch.object(main,'Camera',return_value=source),patch.object(main,'RailDetector',side_effect=RuntimeError('init')):
                with self.assertRaises(RuntimeError):main.main()
            source.release.assert_called_once_with()
