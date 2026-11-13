import unittest
import importlib.util
from types import SimpleNamespace
from calibration import Calibration
import tempfile
from pathlib import Path

class FeedbackTests(unittest.TestCase):
    def test_module_available(self):
        self.assertIsNotNone(importlib.util.find_spec('position_feedback'))
    def test_wire_crc_and_freshness(self):
        if importlib.util.find_spec('position_feedback') is None: self.skipTest('module absent')
        from position_feedback import FeedbackEncoder, crc16
        self.assertEqual(crc16(b'123456789'),0x29B1)
        e=FeedbackEncoder(session=123)
        out={'valid':True,'predicted':False,'x_cm':1.25,'confidence':.9}
        line, valid, reason=e.encode(out,10.0,10.01,True,True)
        self.assertTrue(valid)
        payload, crc=line.strip()[1:].split(b'*')
        self.assertEqual(int(crc,16),crc16(payload))
        self.assertIn(b',1,1.2500,',payload)
        _,valid,reason=e.encode(out,10.0,10.02,True,True)
        self.assertFalse(valid);self.assertEqual(reason,'duplicate')
        _,valid,reason=e.encode(out,10.1,10.5,True,True)
        self.assertFalse(valid);self.assertEqual(reason,'stale')
    def test_invalid_does_not_leak_old_coordinate(self):
        if importlib.util.find_spec('position_feedback') is None: self.skipTest('module absent')
        from position_feedback import FeedbackEncoder
        e=FeedbackEncoder(session=1)
        for changed,allowed,pose in [({'valid':False},True,True),({'predicted':True},True,True),({'x_cm':float('nan')},True,True),({},False,True),({},True,False)]:
            out={'valid':True,'predicted':False,'x_cm':1.25,'confidence':.9};out.update(changed)
            line,valid,_=e.encode(out,1,1.01,pose,allowed)
            self.assertFalse(valid);self.assertNotIn(b'1.2500',line)
            e=FeedbackEncoder(session=1)
    def test_pose_reference_required_and_checked(self):
        with tempfile.TemporaryDirectory() as d:
            c=Calibration(str(Path(d)/'calibration.json'))
            self.assertTrue(hasattr(c,'pose_ok'))
            if not hasattr(c,'pose_ok'):return
            for cm,px in zip((-5,0,5),(100,500,900)):
                c.add_point(cm,px,1280,720,evidence={'rail_slope':0.,'rail_intercept':330.})
            c.fit()
            self.assertTrue(c.pose_ok(SimpleNamespace(valid=True,slope=0.,intercept=330.)))
            self.assertFalse(c.pose_ok(SimpleNamespace(valid=True,slope=.04,intercept=330.)))
            self.assertFalse(c.pose_ok(SimpleNamespace(valid=False,slope=0.,intercept=330.)))
    def test_validation_statistics(self):
        from calibrate import error_report
        r=error_report([1.8,2.,2.2],2.,3)
        self.assertAlmostEqual(r['mean_error_cm'],0.)
        self.assertAlmostEqual(r['mae_cm'],.4/3)
        self.assertAlmostEqual(r['max_abs_error_cm'],.2)
        self.assertAlmostEqual(r['jitter_peak_to_peak_cm'],.4)
        self.assertFalse(r['passed']) # too few samples to claim precision

if __name__=='__main__':unittest.main()
