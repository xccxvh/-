"""Offline dynamic-reference evaluation. Does not enable cm feedback or UART."""
import argparse
import hashlib
import json
import time
import cv2
import numpy as np
from camera import VideoFile
from rail_detector import RailDetector
from ball_detector import BallDetector
from ball_tracker import BallTracker
from ruler_reference import RulerReference, map_measurement


def evaluate(config, reference_video, video, boxes, reference_frame=4):
    source=VideoFile(reference_video)
    try:
        for _ in range(reference_frame+1):
            ok,frame=source.read()
            if not ok:raise ValueError('reference frame unavailable')
        reference=RulerReference(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),boxes)
    finally:source.release()
    source=VideoFile(video);W,H=source.width,source.height
    rail_detector=RailDetector(config['rail'],W,H)
    ball_detector=BallDetector(config['ball'],W,H)
    tracker=BallTracker(config['tracking'],W,H)
    samples=[];durations=[]
    try:
        while True:
            ok,frame=source.read()
            if not ok:break
            index=len(samples);gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            start=time.perf_counter();registration=reference.update(gray)
            durations.append((time.perf_counter()-start)*1000)
            rail=rail_detector.detect(gray,force=True)
            candidates=ball_detector.detect(frame,gray,rail)
            out=tracker.update(candidates,index/source.fps_set)
            point=None
            if out['valid'] and registration['valid'] and rail.valid:
                point=map_measurement(reference,out,candidates)
            samples.append({'frame':index,'measurement_valid':bool(out['valid']),
                            'x_px':out['x_px'],'method':out['method'],
                            'reference_valid':registration['valid'],
                            'reference_x_px':point[0] if point else None,
                            'registration':registration})
    finally:source.release()
    valid=[s for s in samples if s['reference_x_px'] is not None]
    raw=np.array([s['x_px'] for s in valid]);corrected=np.array([s['reference_x_px'] for s in valid])
    def stats(values):
        return {'median':float(np.median(values)),'std':float(values.std()),
                'ptp':float(np.ptp(values))} if len(values) else None
    return {'experimental':True,'runtime_enabled':False,'video':video,
            'reference_video':reference_video,'reference_frame':reference_frame,
            'boxes':boxes,'frames':len(samples),'valid':len(valid),
            'reference_valid_frames':sum(s['reference_valid'] for s in samples),
            'raw_px':stats(raw),'reference_px':stats(corrected),
            'registration_ms':{'mean':float(np.mean(durations)) if durations else None,
                               'p95':float(np.percentile(durations,95)) if durations else None},
            'first_reference_patch_sha256':hashlib.sha256(reference.patches[0][4].tobytes()).hexdigest(),
            'samples':samples}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config',required=True);ap.add_argument('--reference-video',required=True)
    ap.add_argument('--video',required=True);ap.add_argument('--boxes',required=True,
        help='JSON [[x,y,w,h],...] in the reference image')
    ap.add_argument('--output',required=True);args=ap.parse_args()
    with open(args.config) as f:config=json.load(f)
    report=evaluate(config,args.reference_video,args.video,json.loads(args.boxes))
    with open(args.output,'x') as f:json.dump(report,f,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))

if __name__=='__main__':main()
