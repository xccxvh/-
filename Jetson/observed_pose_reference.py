"""Offline candidate. Supplement a failed static-C registration with an observed ruler pose."""
import hashlib
from pathlib import Path

import cv2
import numpy as np

from rail_calibration import binding_id
from ruler_reference import RulerReference
from static_fast_reference import FastUpperEdgeReference


ANCHOR = Path(__file__).parent / 'docs/roll_platform_20261126/field/recordings/ruler_1774349931986/13480.552628.jpg'
ANCHOR_SHA = 'cc45da7bc7b5e63e685f677d612a374f79cbdbbc8a0410d8b586f252f7b3ff92'


class ObservedPoseReference:
    def __init__(self, cfg):
        if hashlib.sha256(ANCHOR.read_bytes()).hexdigest() != ANCHOR_SHA:
            raise ValueError('observed ruler photo hash mismatch')
        gray = cv2.imread(str(ANCHOR), cv2.IMREAD_GRAYSCALE)
        witness = FastUpperEdgeReference(cfg).locate(gray)
        if witness is None or witness['registration']['reference_set'] != 'primary':
            raise ValueError('unconfirmed observed ruler pose')
        reg = witness['registration']
        self.centers = np.asarray(reg['current_points'])
        matrix = np.linalg.inv(witness['reference_inverse'])
        template_image = np.zeros_like(gray)
        for (x, y, w, h), center in zip(cfg['boxes'], self.centers):
            yy, xx = np.indices((h, w), dtype=np.float32)
            offsets = np.stack((xx-w/2, yy-h/2), axis=-1)
            sampled = (offsets @ matrix.T + center).astype(np.float32)
            if np.any(sampled < 1) or np.any(sampled > np.array(gray.shape[::-1])-2):
                raise ValueError('observed patch footprint clipped')
            template_image[y:y+h, x:x+w] = cv2.remap(gray, sampled, None, cv2.INTER_LINEAR)
        self.matcher = RulerReference(template_image, cfg['boxes'], cfg['search_x'], cfg['search_y'])
        self.base = FastUpperEdgeReference(cfg)
        self.reason = 'not_updated'

    def locate(self, gray):
        evidence = self.base.locate(gray)
        self.reason = self.base.reason
        if evidence is not None or self.reason not in (
                'reference_inconsistent_reference','reference_rotation_proposal_inconsistent_reference',
                'reference_rotation_recheck_weak_or_ambiguous_patch'):
            return evidence
        # Full-window original thresholds and joint similarity; no temporal proposal.
        fit = self.matcher.update(gray)
        if not fit['valid'] or np.max(np.abs(np.asarray(fit['current_points'])-self.centers)) > 32:
            return None
        # An alternate exposure/pose cannot resolve a strong tied identity in an old bank.
        for bank in self.base.tracks.values():
            _, scores, margins = bank._raw_peaks(gray)
            if any(s >= bank.min_score and m < bank.min_margin for s,m in zip(scores,margins)):
                self.reason = 'reference_competing_patch'
                return None
        fit.update(reference_set='observed_pose', registration_mode='observed_pose/full')
        evidence = self.base.edge.measure(gray, self.matcher.transform, fit)
        self.reason = 'ok' if evidence is not None else 'upper_edge_geometry'
        return evidence


def bind_offline_coordinates(coords):
    """Attach candidate provenance to an unloaded coordinate sampler before any measurement."""
    if coords.calibration is not None:
        raise ValueError('candidate requires a separate calibration')
    coords.binding['observed_pose_reference'] = dict(
        base_binding_id=coords.binding_id, observed_sha256=ANCHOR_SHA,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    coords.binding_id = binding_id(coords.binding)
