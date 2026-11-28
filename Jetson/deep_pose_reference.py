"""Offline extension: a second strictly verified pose from the first unloaded scan."""
import hashlib
from pathlib import Path

import cv2
import numpy as np

from observed_pose_reference import ANCHOR, ObservedPoseReference, bind_offline_coordinates
from rail_calibration import binding_id
from ruler_reference import RulerReference


DEEP = ANCHOR.with_name('13488.576544.jpg')
DEEP_SHA = '9527d65149b2c67458830add80f655ebde45a13e15bff2c9218470433c611e75'
# Disjoint from the production banks. Selected on first-scan photos only.
LANDMARKS = [[493,648,34,26], [769,663,34,12], [1005,638,34,12]]
EXTRA = [[537,655,44,12],[587,657,44,12],[838,661,54,12]]


def warp_bank(gray, boxes, transform, cfg):
    inverse, shift = transform
    matrix = np.linalg.inv(inverse)
    out = np.zeros_like(gray)
    for x,y,w,h in boxes:
        yy,xx = np.indices((h,w),dtype=np.float32)
        coords = np.stack((xx+x,yy+y),axis=-1) @ matrix.T + shift
        if np.any(coords<1) or np.any(coords>np.array(gray.shape[::-1])-2):
            raise ValueError('incomplete photographed patch')
        out[y:y+h,x:x+w] = cv2.remap(gray,coords.astype(np.float32),None,cv2.INTER_LINEAR)
    return RulerReference(out,boxes,cfg['search_x'],cfg['search_y'])


class DeepPoseReference:
    def __init__(self,cfg):
        self.neutral = ObservedPoseReference(cfg)
        if hashlib.sha256(DEEP.read_bytes()).hexdigest()!=DEEP_SHA:
            raise ValueError('deep anchor mismatch')
        gray = cv2.imread(str(DEEP),0)
        path = Path(cfg['image'])
        if not path.is_absolute():
            path = Path(__file__).parent/path
        photo = cv2.imread(str(path),0)
        marks = RulerReference(photo, LANDMARKS,cfg['search_x'],cfg['search_y'])
        fit = marks.update(gray)
        if not fit['valid'] or self.neutral.base.edge.measure(gray,marks.transform,fit) is None:
            raise ValueError('deep anchor lacks original photo geometry')
        extra = RulerReference(photo, EXTRA,cfg['search_x'],cfg['search_y'])
        p,s,m = extra._raw_peaks(gray)
        inv,shift = marks.transform
        self.matrix = np.linalg.inv(inv)
        self.shift = shift
        predicted = extra.points @ self.matrix.T + shift
        supports = (np.array(s)>=extra.min_score) & (np.array(m)>=extra.min_margin)
        if sum(supports & (np.linalg.norm(predicted-p,axis=1)<=extra.max_residual))<2:
            raise ValueError('independent original photo patches disagree')
        self.primary = warp_bank(gray,cfg['boxes'],marks.transform,cfg)
        self.check = warp_bank(gray,LANDMARKS,marks.transform,cfg)
        self.reason = 'not_updated'

    def locate(self,gray):
        found = self.neutral.locate(gray)
        self.reason = self.neutral.reason
        if found is not None or self.reason not in (
                'reference_inconsistent_reference','reference_rotation_proposal_inconsistent_reference',
                'reference_rotation_recheck_weak_or_ambiguous_patch'):
            return found
        # Six current-frame winners in two separated groups. No near-window second-best repair.
        fit = self.primary.update(gray)
        if not fit['valid']:
            return None
        predicted = self.primary.points @ self.matrix.T + self.shift
        if np.max(np.abs(predicted-fit['current_points'])) > 32:
            return None
        check = self.check.update(gray)
        if not check['valid']:
            return None
        inverse, shift = self.primary.transform
        predicted = self.check.points @ np.linalg.inv(inverse).T + shift
        if np.max(np.linalg.norm(predicted-check['current_points'],axis=1)) > self.check.max_residual:
            return None
        for bank in self.neutral.base.tracks.values():
            _,scores,margins = bank._raw_peaks(gray)
            if any(s>=bank.min_score and m<bank.min_margin for s,m in zip(scores,margins)):
                self.reason='reference_competing_patch'
                return None
        fit.update(reference_set='deep_pose', registration_mode='deep_pose/six_full')
        found = self.neutral.base.edge.measure(gray,self.primary.transform,fit)
        self.reason = 'ok' if found is not None else 'upper_edge_geometry'
        return found


def bind_deep_offline(coords):
    bind_offline_coordinates(coords)
    coords.binding['deep_observed_pose'] = dict(base_binding_id=coords.binding_id, observed_sha256=DEEP_SHA,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    coords.binding_id=binding_id(coords.binding)
