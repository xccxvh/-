import unittest
from pathlib import Path
import cv2
import numpy as np
from ball_detector import BallDetector
from rail_detector import RailLine

class HighlightTests(unittest.TestCase):
    def detect(self, im):
        return BallDetector({},1280,720).detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),RailLine(0,330,True))
    def cap(self, highlights=True):
        im=np.zeros((720,1280,3),np.uint8)
        im[330:345]=180
        cv2.ellipse(im,(525,330),(24,15),0,180,360,(80,80,80),-1)
        if highlights:
            # Same physical cap with two separated specular fragments.
            mask=np.zeros((720,1280),np.uint8)
            cv2.ellipse(mask,(525,330),(24,15),0,180,360,255,-1)
            for s,e in [(509,521),(533,543)]:
                part=im[:,s:e];part[mask[:,s:e]>0]=200
        return im
    def test_two_highlights_recover_full_cap(self):
        cs=self.detect(self.cap())
        self.assertEqual(len(cs),1)
        self.assertAlmostEqual(cs[0].x,525,delta=2)
        self.assertIn('recovered',cs[0].method)
    def test_dim_patch_without_highlight_seeds_not_promoted(self):
        self.assertEqual(self.detect(self.cap(False)),[])
    def test_two_disconnected_glints_not_merged(self):
        im=np.zeros((720,1280,3),np.uint8);im[330:345]=180
        im[317:330,509:521]=200;im[317:330,533:543]=200
        self.assertEqual(self.detect(im),[])
    def test_flat_bright_patch_not_recovered(self):
        im=np.zeros((720,1280,3),np.uint8);im[330:345]=180
        im[315:330,501:550]=80
        im[315:330,509:521]=200;im[315:330,533:543]=200
        self.assertEqual(self.detect(im),[])
    def test_single_highlight_on_flat_patch_not_recovered(self):
        im=np.zeros((720,1280,3),np.uint8);im[330:345]=180
        im[315:330,501:550]=80
        im[315:330,514:536]=200
        self.assertEqual(self.detect(im),[])

    def test_real_manual48_ball_recovered(self):
        im=cv2.imread(str(Path(__file__).parent/'fixtures/split_highlight_manual48.png'))
        self.assertIsNotNone(im)
        d=BallDetector({},1280,720)
        cs=d.detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),RailLine(.0257882123,230.5223904,True))
        self.assertTrue(any(520<c.x<538 for c in cs))
    def test_recovered_coordinates_not_accepted_for_old_calibration(self):
        from measurement_contract import calibrated_method
        self.assertFalse(calibrated_method('geom_recovered'))
        self.assertFalse(calibrated_method('hybrid_recovered'))
        self.assertTrue(calibrated_method('geom'))
        self.assertTrue(calibrated_method('hybrid'))

class RecoveryIntegrationTests(unittest.TestCase):
    def test_recovery_is_pixel_only_in_csv_and_calibration_collection(self):
        import tempfile,json,subprocess,sys,csv
        from calibration import Calibration
        from calibrate import collect_window
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp);path=tmp/'split.avi'
            w=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'MJPG'),30,(1280,720))
            self.assertTrue(w.isOpened())
            for _ in range(12):w.write(HighlightTests().cap())
            w.release()
            cal=Calibration(str(tmp/'cal.json'))
            for cm,x in zip((-5,0,5),(100,500,900)):
                cal.add_point(cm,x,1280,720,evidence={'rail_slope':0,'rail_intercept':330})
            self.assertTrue(cal.fit()[0])
            cfg=json.loads((root/'config.json').read_text());cfg['calibration']['file']=str(tmp/'cal.json');cfg['output']={'logs_dir':str(tmp/'logs'),'recordings_dir':str(tmp/'rec')}
            config=tmp/'cfg.json';config.write_text(json.dumps(cfg))
            p=subprocess.run([sys.executable,str(root/'main.py'),'--config',str(config),'--video',str(path),'--csv'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            self.assertEqual(p.returncode,0,p.stderr.decode())
            with next((tmp/'logs').glob('*.csv')).open() as f:rows=list(csv.DictReader(f))
            valid=[r for r in rows if r['valid']=='1'];self.assertTrue(valid)
            self.assertTrue(all(r['pose_ok']=='1' and r['x_cm']=='' and r['feedback_valid']=='0' for r in valid))
            samples,_,_=collect_window(cfg,str(path),4,11)
            self.assertEqual(samples,[])

class RollingRecoveryTests(unittest.TestCase):
    def test_small_single_highlight_caps_recovered(self):
        import json
        root=Path(__file__).parent/'fixtures'
        poses=json.loads((root/'rolling_poses.json').read_text())
        for idx, bounds in [(145,(195,220)),(146,(198,224)),(149,(216,240)),(153,(249,280))]:
            with self.subTest(frame=idx):
                im=cv2.imread(str(root/('rolling_%d.png'%idx)))
                p=poses[str(idx)]
                cs=BallDetector({},1280,720).detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),RailLine(p['rail_slope'],p['rail_intercept'],True))
                self.assertTrue(any(bounds[0]<c.x<bounds[1] and 'recovered' in c.method for c in cs))
