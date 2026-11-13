"""Per-frame ruler reference registration; no camera or actuator ownership.

Three or more textured patches must agree on a 2-D similarity transform.
This corrects image translation, in-plane rotation and uniform scale, not
arbitrary perspective. A failed update clears the transform immediately.
Coordinates are reference-image pixels, never implicitly centimeters.
"""
import math
import cv2
import numpy as np


class RulerReference(object):
    def __init__(self, gray, boxes, search_x=32, search_y=24,
                 min_score=.85, min_margin=.05, max_residual_px=2.5):
        if gray is None or gray.ndim != 2 or gray.dtype != np.uint8:
            raise ValueError('reference must be uint8 grayscale')
        self.shape = gray.shape
        self.search_x, self.search_y = int(search_x), int(search_y)
        self.min_score, self.min_margin = float(min_score), float(min_margin)
        self.max_residual = float(max_residual_px)
        if (self.search_x < 1 or self.search_y < 1 or
                not 0 < self.min_score <= 1 or not 0 <= self.min_margin < 1 or
                not math.isfinite(self.max_residual) or self.max_residual <= 0):
            raise ValueError('invalid registration limits')
        self.patches = []
        points = []
        for box in boxes:
            x, y, w, h = [int(v) for v in box]
            if x < 0 or y < 0 or w < 8 or h < 8 or x+w > gray.shape[1] or y+h > gray.shape[0]:
                raise ValueError('invalid reference patch')
            patch = gray[y:y+h, x:x+w].copy()
            if float(patch.std()) < 5:
                raise ValueError('reference patch lacks texture')
            self.patches.append((x, y, w, h, patch))
            points.append((x+w/2.0, y+h/2.0))
        self.points = np.asarray(points, dtype=np.float64)
        if len(points) < 3 or np.ptp(self.points[:, 0]) < 100:
            raise ValueError('need at least three separated ruler patches')
        self.center = self.points.mean(axis=0)
        self.centered = self.points-self.center
        self.denominator = float((self.centered**2).sum())
        self.transform = None

    def update(self, gray):
        self.transform = None
        if gray is None or gray.shape != self.shape or gray.dtype != np.uint8:
            return {'valid': False, 'reason': 'resolution_or_format'}
        points, scores, margins = [], [], []
        H, W = self.shape
        for x, y, w, h, patch in self.patches:
            x0, x1 = max(0, x-self.search_x), min(W, x+w+self.search_x)
            y0, y1 = max(0, y-self.search_y), min(H, y+h+self.search_y)
            response = cv2.matchTemplate(gray[y0:y1, x0:x1], patch,
                                         cv2.TM_CCOEFF_NORMED)
            _, score, _, loc = cv2.minMaxLoc(response)
            xx, yy = loc
            # Exclude the local correlation peak before comparing other
            # ruler ticks/labels. Ambiguous repetitive texture is invalid.
            rx, ry = 2, 2
            response[max(0,yy-ry):yy+ry+1, max(0,xx-rx):xx+rx+1] = -1
            margin = score-float(response.max())
            if (not math.isfinite(score) or not math.isfinite(margin) or
                    score < self.min_score or margin < self.min_margin):
                return {'valid': False, 'reason': 'weak_or_ambiguous_patch'}
            points.append((x0+xx+w/2.0, y0+yy+h/2.0))
            scores.append(score); margins.append(margin)
        current = np.asarray(points, dtype=np.float64)
        center = current.mean(axis=0)
        v = current-center
        u = self.centered
        a = float((u*v).sum()/self.denominator)
        b = float((u[:,0]*v[:,1]-u[:,1]*v[:,0]).sum()/self.denominator)
        scale = math.hypot(a,b)
        angle = math.degrees(math.atan2(b,a))
        matrix = np.array([[a,-b],[b,a]])
        translation = center-matrix.dot(self.center)
        residual = float(np.max(np.linalg.norm(self.points.dot(matrix.T)+translation-current,axis=1)))
        if not .9 <= scale <= 1.1 or abs(angle) > 5 or residual > self.max_residual:
            return {'valid': False, 'reason': 'inconsistent_reference'}
        self.transform = (matrix.T/(scale*scale), translation)
        return {'valid': True, 'reason': 'ok', 'min_score': min(scores),
                'min_margin': min(margins), 'residual_px': residual,
                'scale': scale, 'image_rotation_deg': angle}

    def to_reference(self, x, y):
        if self.transform is None or not math.isfinite(x) or not math.isfinite(y):
            return None
        inverse, translation = self.transform
        point = inverse.dot(np.array([x,y])-translation)
        return float(point[0]), float(point[1])


def map_measurement(reference, out, candidates):
    if not out['valid'] or out.get('predicted',False):
        return None
    selected=[c for c in candidates if c.x==out['x_px'] and c.method==out['method']]
    if len(selected)!=1 or 'measurement_y_px' not in selected[0].extra:
        return None
    return reference.to_reference(out['x_px'],selected[0].extra['measurement_y_px'])

