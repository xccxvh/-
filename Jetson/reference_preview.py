"""Explicit experimental cm preview; never authorizes UART control."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from ruler_reference import RulerReference, map_measurement


def algorithm_digest(config):
    relevant={key:config.get(key,{}) for key in ('ball','rail')}
    return hashlib.sha256(json.dumps(relevant,sort_keys=True,separators=(',',':')).encode()).hexdigest()


class ReferencePreview(object):
    def __init__(self, path, config=None):
        path=Path(path)
        with path.open() as f:data=json.load(f)
        if data.get('experimental') is not True:
            raise ValueError('explicit experimental reference manifest required')
        base=Path(__file__).resolve().parent
        for key,name in [('detector_sha256','ball_detector.py'),('reference_algorithm_sha256','ruler_reference.py')]:
            if data.get(key)!=hashlib.sha256((base/name).read_bytes()).hexdigest():
                raise ValueError('reference estimator differs from validated version')
        if data.get('algorithm_sha256')!=algorithm_digest(config or {}):
            raise ValueError('detector/rail settings differ from validated version')
        image=path.parent/data['reference_image']
        if data.get('reference_sha256')!=hashlib.sha256(image.read_bytes()).hexdigest():
            raise ValueError('reference image hash mismatch')
        gray=cv2.imread(str(image),cv2.IMREAD_GRAYSCALE)
        self.reference=RulerReference(gray,data['boxes'])
        model=data['model']
        self.xs=np.asarray(model['xs'],dtype=float);self.ys=np.asarray(model['ys'],dtype=float)
        if (model.get('type')!='piecewise' or self.xs.ndim!=1 or self.ys.ndim!=1 or
                len(self.xs)<3 or len(self.xs)!=len(self.ys) or
                not np.all(np.isfinite(self.xs)) or not np.all(np.isfinite(self.ys)) or
                not np.all(np.diff(self.xs)>0) or not np.all(np.diff(self.ys)>0)):
            raise ValueError('invalid preview mapping')
        self.invalidate('not_updated')

    def invalidate(self, reason):
        self.valid = False
        self.reference.transform = None
        self.reference_x_px = None
        self.status = reason
        self.registration = {}

    def update(self, gray):
        self.invalidate('not_updated')
        self.registration = self.reference.update(gray)
        self.valid = bool(self.registration['valid'])
        self.status = ('reference_ready' if self.valid else
                       'reference_' + self.registration.get('reason', 'invalid'))
        return self.valid

    def position(self, out, candidates):
        self.reference_x_px = None
        if not self.valid:
            return None
        point = map_measurement(self.reference, out, candidates)
        if point is None:
            self.status = 'measurement_unmatched'
            return None
        self.reference_x_px = float(point[0])
        if point[0] < self.xs[0]:
            self.status = 'below_calibrated_range'
            return None
        if point[0] > self.xs[-1]:
            self.status = 'above_calibrated_range'
            return None
        self.status = 'ok_preview'
        return float(np.interp(point[0], self.xs, self.ys))
