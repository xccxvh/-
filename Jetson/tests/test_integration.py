import csv
import os
import pty
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from position_feedback import SerialFeedback
from calibrate import collect_window
import json

ROOT=Path(__file__).resolve().parents[1]
class IntegrationTests(unittest.TestCase):
    def test_offline_uart_rejected_before_open(self):
        result=subprocess.run([sys.executable,str(ROOT/'main.py'),'--video','missing.mp4','--serial','/dev/does-not-exist'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        self.assertIn(b'offline replay',result.stderr)
    def test_real_pty_transport(self):
        master,slave=pty.openpty()
        serial=SerialFeedback(os.ttyname(slave))
        try:
            serial.send(b'$test*ABCD\n')
            self.assertEqual(os.read(master,100),b'$test*ABCD\n')
        finally:
            serial.close();os.close(master);os.close(slave)
    def test_selected_window_excludes_later_rolling(self):
        with tempfile.TemporaryDirectory() as d:
            video=Path(d)/'known_then_roll.avi'
            writer=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'MJPG'),30,(1280,720))
            self.assertTrue(writer.isOpened())
            for i in range(20):
                frame=np.zeros((720,1280,3),dtype=np.uint8)
                frame[330:,:,:]=200
                cv2.ellipse(frame,(300 if i<10 else 600,330),(24,14),0,180,360,(255,200,100),-1)
                writer.write(frame)
            writer.release()
            cfg=json.loads((ROOT/'config.json').read_text())
            rows,w,h=collect_window(cfg,str(video),5,9)
            self.assertEqual([r['frame'] for r in rows],list(range(5,10)))
            self.assertTrue(all(abs(r['x_px']-300)<3 for r in rows))
            calibration=Path(d)/'calibration.json'
            cmd=[sys.executable,str(ROOT/'calibrate.py'),'--calibration',str(calibration),'add','--cm','-5','--video',str(video),'--start-frame','5','--end-frame','9','--known-position']
            result=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            self.assertEqual(result.returncode,0,result.stderr.decode())
            before=calibration.read_bytes()
            data=json.loads(before.decode())
            self.assertEqual(len(data['points'][0]['evidence']['samples']),5)
            self.assertEqual(len(data['points'][0]['evidence']['sha256']),64)
            self.assertIsNone(data['model'])
            cmd[cmd.index('--end-frame')+1]='99'
            result=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            self.assertNotEqual(result.returncode,0)
            self.assertEqual(calibration.read_bytes(),before)


    def test_video_and_csv_invalid_measurements(self):
        with tempfile.TemporaryDirectory() as d:
            d=Path(d)
            video=d/'blank.avi'
            w=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'MJPG'),30,(1280,720))
            self.assertTrue(w.isOpened())
            for _ in range(8):w.write(np.zeros((720,1280,3),dtype=np.uint8))
            w.release()
            cfg=json.loads((ROOT/'config.json').read_text())
            cfg['output']={'logs_dir':str(d/'logs'),'recordings_dir':str(d/'rec')}
            config=d/'config.json';config.write_text(json.dumps(cfg))
            result=subprocess.run([sys.executable,str(ROOT/'main.py'),'--config',str(config),'--video',str(video),'--csv'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            self.assertEqual(result.returncode,0,result.stderr.decode())
            with next((d/'logs').glob('*.csv')).open() as f:rows=list(csv.DictReader(f))
            self.assertEqual(len(rows),8)
            self.assertIn('feedback_valid',rows[0])
            for row in rows:
                self.assertEqual(row['x_cm'],'')
                self.assertEqual(row['valid'],'0')
                self.assertEqual(row['rail_valid'],'0')
            with self.assertRaises(ValueError):collect_window(cfg,str(video),0,20)

if __name__=='__main__':unittest.main()
