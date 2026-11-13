import unittest,tempfile,json,hashlib
from pathlib import Path
import cv2
import numpy as np
from reference_preview import ReferencePreview, algorithm_digest
from ball_detector import BallCandidate

class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.im=np.random.RandomState(6).randint(0,256,(240,640)).astype(np.uint8)
        self.image=self.root/'ref.png';cv2.imwrite(str(self.image),self.im)
        self.manifest={'experimental':True,'reference_image':'ref.png','reference_sha256':hashlib.sha256(self.image.read_bytes()).hexdigest(),'detector_sha256':hashlib.sha256((Path(__file__).resolve().parents[1]/'ball_detector.py').read_bytes()).hexdigest(),'boxes':[[90,100,40,24],[290,100,40,24],[490,100,40,24]],'model':{'type':'piecewise','xs':[100,300,500],'ys':[-5,0,5]}}
        self.manifest['reference_algorithm_sha256']=hashlib.sha256((Path(__file__).resolve().parents[1]/'ruler_reference.py').read_bytes()).hexdigest()
        self.manifest['algorithm_sha256']=algorithm_digest({})
        self.path=self.root/'preview.json';self.save()
    def tearDown(self):self.tmp.cleanup()
    def save(self):self.path.write_text(json.dumps(self.manifest))
    def test_current_valid_reference_maps_recovered_measurement(self):
        preview=ReferencePreview(str(self.path));self.assertTrue(preview.update(self.im))
        c=BallCandidate(380,30,12,.8,'geom_recovered',{'measurement_y_px':100})
        out={'valid':True,'predicted':False,'x_px':380,'method':'geom_recovered'}
        self.assertAlmostEqual(preview.position(out,[c]),2)
        self.assertFalse(preview.update(np.zeros_like(self.im)))
        self.assertIsNone(preview.position(out,[c]))
    def test_changed_reference_and_detector_are_rejected(self):
        for key in ['reference_sha256','detector_sha256']:
            old=self.manifest[key];self.manifest[key]='0'*64;self.save()
            with self.assertRaises(ValueError):ReferencePreview(str(self.path))
            self.manifest[key]=old
    def test_nonfinite_or_nonmonotonic_mapping_rejected(self):
        for xs in [[100,100,500],[100,float('nan'),500]]:
            self.manifest['model']['xs']=xs;self.save()
            with self.assertRaises(ValueError):ReferencePreview(str(self.path))
    def test_no_extrapolation(self):
        preview=ReferencePreview(str(self.path));preview.update(self.im)
        c=BallCandidate(510,30,12,.8,'geom',{'measurement_y_px':100})
        self.assertIsNone(preview.position({'valid':True,'x_px':510,'method':'geom'},[c]))

    def test_preview_rejects_uart_before_opening_devices(self):
        import subprocess,sys
        main=Path(__file__).resolve().parents[1]/'main.py'
        p=subprocess.run([sys.executable,str(main),'--reference-preview',str(self.path),'--serial','/dev/should_not_open'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        self.assertNotEqual(p.returncode,0)
        self.assertIn(b'reference preview cannot open UART',p.stderr)

    def test_algorithm_settings_mismatch_rejected(self):
        with self.assertRaises(ValueError):ReferencePreview(str(self.path),{'ball':{'envelope_threshold':80}})

    def test_invalidation_clears_transform_and_diagnostics(self):
        p=ReferencePreview(str(self.path));p.update(self.im)
        p.invalidate('vision')
        self.assertFalse(p.valid)
        self.assertIsNone(p.reference.transform)
        self.assertIsNone(p.reference_x_px)
        self.assertEqual(p.status,'vision')

    def test_outside_range_has_distinct_reason_and_coordinate(self):
        p=ReferencePreview(str(self.path));p.update(self.im)
        c=BallCandidate(510,30,12,.8,'geom',{'measurement_y_px':100})
        self.assertIsNone(p.position({'valid':True,'x_px':510,'method':'geom'},[c]))
        self.assertEqual(p.status,'above_calibrated_range')
        self.assertAlmostEqual(p.reference_x_px,510)
