"""Offline observed-pose candidate evaluation. No camera, UART or live entry point."""
import multiprocessing
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

JET = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(JET), str(JET/'docs/roll_platform_20261126'), str(JET/'docs/static_control_c')]
from diagnose_right_patch import RUNS, frames, templates
from ruler_reference import RulerReference
from deep_pose_reference import DeepPoseReference, bind_deep_offline
from observed_pose_reference import ObservedPoseReference, bind_offline_coordinates
from static_fast_reference import FastUpperEdgeReference
import replay as placement_replay


# Selected on the neutral anchor only, outside all three template banks.
WITNESS_BOXES = [[493,648,34,26],[537,655,44,12],[587,657,44,12],[769,663,34,12],
                 [808,653,34,26],[838,661,54,12],[1005,638,34,12]]


def audit_pose(witness, gray, evidence):
    current, scores, margins = witness._raw_peaks(gray)
    mask = (np.array(scores) >= witness.min_score) & (np.array(margins) >= witness.min_margin)
    if not mask.any():
        return dict(n=0)
    model = np.linalg.inv(evidence['reference_inverse'])
    ref, measured = witness.points[mask], np.asarray(current)[mask]
    predicted = ref @ model.T + evidence['reference_translation']
    errors = np.linalg.norm(predicted-measured,axis=1)
    point_errors = dict(n=int(mask.sum()),rms=round(float(np.sqrt(np.mean(errors**2))),3),
                        worst=round(float(errors.max()),3))
    if mask.sum() < 3:
        return point_errors
    # Independent least-squares similarity for diagnostics, never used by the detector.
    u = ref-ref.mean(axis=0)
    v = measured-measured.mean(axis=0)
    den = np.sum(u*u)
    a, b = np.sum(u*v)/den, np.sum(u[:,0]*v[:,1]-u[:,1]*v[:,0])/den
    return dict(**point_errors,
                angle_delta=round(float(np.rad2deg(np.arctan2(model[1,0],model[0,0])-np.arctan2(b,a))),4),
                scale_delta_pct=round(float(100*(np.hypot(*model[:,0])/np.hypot(a,b)-1)),4))


def assess_field(deep=False):
    cfg = json.loads((JET/'config_static_fast_c.json').read_text())['rail']['upper_reference']
    bank = templates(cfg)
    witness = RulerReference(cv2.imread(str(JET/cfg['image']),0),WITNESS_BOXES,96,96)
    candidate_type = DeepPoseReference if deep else ObservedPoseReference
    deep_witness = RulerReference(cv2.imread(str(JET/cfg['image']),0),
                                  [WITNESS_BOXES[i] for i in (1,2,4,5)],96,96)
    for j, (tag, _) in enumerate(RUNS):
        data = frames(j,bank)
        base, new = FastUpperEdgeReference(cfg), candidate_type(cfg)
        print('scan',tag)
        summary = []
        for stamp, group, gray, pos in data:
            original = base.locate(gray)
            candidate = new.locate(gray)
            error = None
            if candidate is not None:
                est = base.tracks['primary'].points @ np.linalg.inv(candidate['reference_inverse']).T
                est += candidate['reference_translation']
                error = np.linalg.norm(est-pos,axis=1).max()
            if original is None and candidate is not None:
                group_name = candidate['registration']['reference_set']
                checks = deep_witness if group_name=='deep_pose' else witness
                print('witness',stamp,group_name,audit_pose(checks,gray,candidate))
            if original is not None:
                assert candidate is not None
                np.testing.assert_array_equal(original['reference_translation'],candidate['reference_translation'])
                np.testing.assert_array_equal(original['reference_inverse'],candidate['reference_inverse'])
            summary.append((stamp,group,original is not None,candidate is not None,error))
        for group in range(-1,11):
            rows = [r for r in summary if r[1]==group]
            print(group,len(rows),sum(r[2] for r in rows),sum(r[3] for r in rows))
        print('gains',[(r[0],round(r[4],2)) for r in summary if r[3] and not r[2]])
        # Same 177 historical controls: copy +86 px, remove/copy, remove only.
        for kind in ('duplicate','shift_only','blank'):
            counts = [0,0,0]
            for _, group, gray, pos in data:
                if group not in (0,2,4,7,10):
                    continue
                x,y = np.rint(pos[2]).astype(int)
                src = gray[y-24:y+25,x-41:x+42].copy()
                altered = gray.copy()
                if kind != 'duplicate':
                    altered[y-24:y+25,x-41:x+42] = int(np.median(src))
                if kind != 'blank':
                    altered[y-24:y+25,x+45:x+128] = src
                counts[0] += 1
                counts[1] += FastUpperEdgeReference(cfg).locate(altered) is not None
                counts[2] += candidate_type(cfg).locate(altered) is not None
            print(kind,counts)


original_components = placement_replay.components


def candidate_components(config, deep=False):
    coords, rd, bd, motion, tracker = original_components(config)
    cfg = json.loads(config.read_text())
    candidate_type = DeepPoseReference if deep else ObservedPoseReference
    rd.upper_reference = candidate_type(cfg['rail']['upper_reference'])
    (bind_deep_offline if deep else bind_offline_coordinates)(coords)
    return coords,rd,bd,motion,tracker


def deep_components(config):
    return candidate_components(config, deep=True)


# Top-level picklable adapters install only the offline component provider inside each
# spawned child. Python 3.6 ProcessPoolExecutor lacks mp_context and initializer.
original_clip = placement_replay.replay_clip


def neutral_clip(task):
    placement_replay.components = candidate_components
    return original_clip(task)


def deep_clip(task):
    placement_replay.components = deep_components
    return original_clip(task)


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--placements', type=Path, help='New offline output directory for known-position validation')
    parser.add_argument('--data', type=Path, default=Path('/home/user/桌面/jetson_nano'))
    parser.add_argument('--deep', action='store_true', help='Evaluate frozen additional 1246-tick viewpoint')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    if args.placements:
        multiprocessing.set_start_method('spawn')
        placement_replay.components = deep_components if args.deep else candidate_components
        placement_replay.replay_clip = deep_clip if args.deep else neutral_clip
        placement_replay.run(args.data, args.placements, args.placements/'candidate_model.json', JET/'config_static_fast_c.json')
    else:
        assess_field(args.deep)
