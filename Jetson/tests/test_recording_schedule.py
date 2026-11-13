import unittest
import recording_schedule as rs

class RecordingScheduleTests(unittest.TestCase):
    def test_30hz_measurements_make_15hz_recording(self):
        s=rs.RecordingSchedule(15)
        self.assertEqual([i for i in range(30) if s.due(i/30.)],list(range(0,30,2)))
    def test_delayed_frame_does_not_trigger_catchup_work(self):
        s=rs.RecordingSchedule(15)
        self.assertTrue(s.due(10.))
        self.assertTrue(s.due(11.))
        self.assertFalse(s.due(11.001))
        self.assertFalse(s.due(10.5))
    def test_nonfinite_timestamp_does_not_poison_schedule(self):
        s=rs.RecordingSchedule(15)
        self.assertFalse(s.due(float('nan')))
        self.assertTrue(s.due(0))
    def test_invalid_rate_rejected(self):
        for v in [0,-1,float('inf')]:
            with self.assertRaises(ValueError):rs.RecordingSchedule(v)

class RecordingIntegrationTests(unittest.TestCase):
    def test_preview_sampling_preserves_every_measurement(self):
        import cv2,numpy as np,json,subprocess,sys,tempfile,csv
        from pathlib import Path
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp);video=tmp/'input.avi'
            w=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'MJPG'),30,(1280,720))
            self.assertTrue(w.isOpened())
            for _ in range(12):w.write(np.zeros((720,1280,3),np.uint8))
            w.release()
            cfg=json.loads((root/'config.json').read_text());cfg['output']={'logs_dir':str(tmp/'logs'),'recordings_dir':str(tmp/'rec')};p=tmp/'cfg.json';p.write_text(json.dumps(cfg))
            result=subprocess.run([sys.executable,str(root/'main.py'),'--config',str(p),'--video',str(video),'--record','--csv','--record-fps','15','--record-scale','.5'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            self.assertEqual(result.returncode,0,result.stderr.decode())
            with next((tmp/'logs').glob('*.csv')).open() as f:rows=list(csv.DictReader(f))
            self.assertEqual(len(rows),12)
            movie=next((tmp/'rec').glob('*.mp4'));cap=cv2.VideoCapture(str(movie))
            self.assertEqual(int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),640)
            self.assertEqual(int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),360)
            decoded=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));cap.release()
            events=[json.loads(line) for line in Path(str(movie)+'.timestamps.jsonl').read_text().splitlines()]
            self.assertTrue(events[-1]['complete'])
            self.assertEqual(decoded,events[-1]['accepted'])
            self.assertGreater(decoded,0)
            self.assertEqual(events[-1]['accepted']+events[-1]['dropped'],6)
            self.assertEqual([e['frame_index'] for e in events[:-1] if e['accepted']],list(range(decoded)))
            self.assertEqual([round(e['timestamp'],6) for e in events[:-1]],[round(i/30.,6) for i in range(0,12,2)])

class LowRateRecordingTests(unittest.TestCase):
    def test_low_requested_rate_is_not_silently_changed_to_30(self):
        import tempfile
        from pathlib import Path
        import cv2,numpy as np
        from video_writer import AsyncVideoWriter
        for fps in [1.,.5]:
            with self.subTest(fps=fps),tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/'slow.mp4';w=AsyncVideoWriter(str(path),32,32,fps)
                try:
                    w.write(np.zeros((32,32,3),np.uint8),timestamp=0.)
                    w.write(np.zeros((32,32,3),np.uint8),timestamp=1/fps)
                finally:w.release()
                cap=cv2.VideoCapture(str(path));actual=cap.get(cv2.CAP_PROP_FPS);cap.release()
                self.assertAlmostEqual(actual,fps)
