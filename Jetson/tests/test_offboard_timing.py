import csv,json,sys,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch,MagicMock
import numpy as np
import main

class TimingTests(unittest.TestCase):
    def run_source(self, video=False, preview_mode=False):
        source=MagicMock(width=1280,height=720,fps_set=.5 if video else 30,timestamp_verified=False)
        start=time.perf_counter()-1;times=iter([start,start+.1,start+.3])
        frame=np.zeros((720,1280,3),np.uint8)
        def read():
            try:source.last_ts=next(times)
            except StopIteration:return False,None
            return True,frame
        source.read.side_effect=read
        preview=MagicMock(valid=False,reference_x_px=None,status='vision')
        with tempfile.TemporaryDirectory() as d:
            cfg=json.loads((Path(main.BASE)/'config.json').read_text())
            cfg['output']={'logs_dir':d,'recordings_dir':d}
            args=['main.py','--csv']+(['--video','dummy'] if video else [])+(['--reference-preview','dummy'] if preview_mode else [])
            with patch.object(main,'ReferencePreview',return_value=preview),patch.object(sys,'argv',args),patch.object(main,'load_config',return_value=cfg),patch.object(main,'VideoFile' if video else 'Camera',return_value=source):
                main.main()
            with next(Path(d).glob('*.csv')).open() as f:
                rows=list(csv.DictReader(f))
        if preview_mode:
            preview.update.assert_not_called()
            self.assertEqual(preview.invalidate.call_count,3)
            self.assertTrue(all(r['cm_status']=='vision' and r['cm_valid']=='0' for r in rows))
        return [float(r['video_time_s']) for r in rows]
    def test_live_time_retains_capture_gaps(self):
        self.assertEqual(self.run_source(),[0,.1,.3])
    def test_slow_media_time(self):
        self.assertEqual(self.run_source(True),[0,2,4])

    def test_no_ball_skips_reference_matching(self):
        self.run_source(preview_mode=True)

    def test_success_then_invalid_inputs_clear_preview_state(self):
        from reference_preview import ReferencePreview
        from types import SimpleNamespace
        from rail_detector import RailLine
        p=ReferencePreview.__new__(ReferencePreview)
        p.reference=SimpleNamespace(transform=None)
        p.invalidate('not_updated')
        def update(gray):
            p.valid=True;p.reference.transform='current';p.status='reference_ready'
            return True
        def position(out,candidates):
            p.reference_x_px=300.;p.status='ok_preview';return 0.
        p.update=MagicMock(side_effect=update);p.position=MagicMock(side_effect=position)
        source=MagicMock(width=1280,height=720,fps_set=30,timestamp_verified=False)
        index=[0];frame=np.zeros((720,1280,3),np.uint8)
        def read():
            index[0]+=1;source.last_ts=time.perf_counter()
            return (True,frame) if index[0]<=4 else (False,None)
        source.read.side_effect=read
        outputs=[dict(valid=v,predicted=pred,x_px=300.,confidence=.9,state='DETECTED' if v else 'LOST',method='geom') for v,pred in [(True,False),(True,False),(True,True),(False,False)]]
        tracker=MagicMock();tracker.update.side_effect=outputs
        rail=MagicMock();rail.detect.side_effect=[RailLine(0,300,v,.9) for v in [True,False,True,True]]
        with tempfile.TemporaryDirectory() as d:
            cfg=json.loads((Path(main.BASE)/'config.json').read_text());cfg['output']={'logs_dir':d,'recordings_dir':d}
            with patch.object(sys,'argv',['main.py','--csv','--reference-preview','dummy']),patch.object(main,'load_config',return_value=cfg),patch.object(main,'Camera',return_value=source),patch.object(main,'ReferencePreview',return_value=p),patch.object(main,'BallTracker',return_value=tracker),patch.object(main,'RailDetector',return_value=rail),patch.object(main,'BallDetector'):
                main.main()
            with next(Path(d).glob('*.csv')).open() as f:rows=list(csv.DictReader(f))
        self.assertEqual([r['cm_status'] for r in rows],['ok_preview','rail','vision','vision'])
        self.assertEqual(rows[0]['cm_valid'],'1')
        for r in rows[1:]:
            self.assertEqual((r['x_cm'],r['reference_x_px'],r['cm_valid']),('','','0'))
        p.update.assert_called_once();p.position.assert_called_once()
        self.assertIsNone(p.reference.transform);self.assertFalse(p.valid)
