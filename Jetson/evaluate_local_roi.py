# -*- coding: utf-8 -*-
"""同段视频五版本离板对照；抽样标签与未标注帧严格分开，不输出虚构厘米值。"""
import argparse
import copy
import csv
import hashlib
import json
import subprocess
import time
from pathlib import Path
import cv2
import numpy as np
from rail_detector import RailDetector
from ball_detector import BallDetector
from ball_tracker import BallTracker
from local_vision import PhysicalSearch
from visualization import draw_overlay


def validate_labels(labels, expected):
    truth={}
    for item in labels:
        frame=item['frame']
        if type(frame) is not int or frame<0 or frame>=expected or frame in truth:
            raise ValueError('duplicate or outside label frame')
        truth[frame]=item
    return truth


def assert_coverage(actual, expected, shape):
    if actual!=expected or shape!=(720,1280,3):
        raise ValueError('incomplete decode or unsupported resolution')


def assess_pixel(out, true_x, tolerance, region):
    if true_x is None:return 'visibility_unknown',None
    error=abs(out['x_px']-true_x) if out['valid'] else None
    if region=='outside':return ('outside_false_valid' if out['valid'] else 'outside_rejected'),error
    return ('correct' if out['valid'] and error<=tolerance else 'false_valid' if out['valid'] else 'miss'),error


def versions(config, baseline):
    result={}
    for name in ['Baseline','V1','V2','V3','V4']:
        c=copy.deepcopy(config if name!='Baseline' else baseline)
        if name in ('Baseline','V1'):c['ball_search']={'enabled':False}
        if name in ('V1','V2'):c['ball']['mode']='legacy'
        c['tracking']['identity_check']=name=='V4'
        bc=dict(c['ball']);bc['ball_search']=c.get('ball_search',{})
        result[name]=(RailDetector(c['rail'],1280,720),BallDetector(bc,1280,720),BallTracker(c['tracking'],1280,720))
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--video',required=True);ap.add_argument('--config',required=True)
    ap.add_argument('--baseline-config',default='config_newpose_20261113.json')
    ap.add_argument('--labels',required=True);ap.add_argument('--output',required=True)
    ap.add_argument('--fps',type=float,default=29.8)
    ap.add_argument('--expected-frames',type=int,default=1043)
    args=ap.parse_args()
    if not np.isfinite(args.fps) or args.fps<=0 or args.expected_frames<1:raise ValueError('positive FPS and frame count required')
    cfg=json.loads(Path(args.config).read_text());baseline=json.loads(Path(args.baseline_config).read_text())
    labels=json.loads(Path(args.labels).read_text());truth=validate_labels(labels['labels'],args.expected_frames)
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    v=versions(cfg,baseline);evaluation_rail=RailDetector(cfg['rail'],1280,720);search=PhysicalSearch(cfg['ball_search'])
    cap=cv2.VideoCapture(args.video);rows={k:[] for k in v};sample_metrics={k:[] for k in v}
    target=output/'V4_annotated.mp4'
    # 离线逐帧阻塞编码，保证原视频帧i、CSV行i和标注帧i一一对应。
    encoder=subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','bgr24','-s','1280x720','-r',str(args.fps),'-i','-','-an','-c:v','libx264','-preset','ultrafast','-crf','21','-threads','1','-pix_fmt','yuv420p',str(target)],stdin=subprocess.PIPE)
    idx=0;shape=None
    try:
        while True:
            ok,frame=cap.read()
            if not ok:break
            shape=frame.shape
            if shape!=(720,1280,3):raise ValueError("unsupported resolution")
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            er=evaluation_rail.detect(gray,True);region='unlabeled';true_x=None
            label=truth.get(idx)
            if label and label['status']=='visible' and label['x_px'] is not None:
                true_x=float(label['x_px'])
                if er.identity_valid:
                    y=np.interp(true_x,np.arange(1280),er.edge_profile)
                    ref=er.reference_inverse.dot(np.array([true_x,y])-er.reference_translation)[0]
                    a=search.anchors
                    bounds=np.interp([cfg['ball_search']['target_min_cm'],cfg['ball_search']['target_max_cm'],search.low,search.high],a[:,0],a[:,1])
                    region='core' if bounds[0]<=ref<=bounds[1] else 'buffer' if bounds[2]<=ref<=bounds[3] else 'outside'
                else:region='reference_unknown'
            elif label:region='visibility_unknown'
            for name,(rd,bd,bt) in v.items():
                start=time.perf_counter();rail=rd.detect(gray,True);cs=bd.detect(frame,gray,rail);out=bt.update(cs,idx/args.fps);ms=(time.perf_counter()-start)*1000
                assessment='unlabeled';err=None
                if label:
                    # 参考失效不能让可见真值样本消失：仍统计像素正确性和未知分区。
                    assessment,err=assess_pixel(out,true_x,label.get('tolerance_px',6),region)
                    sample_metrics[name].append({'frame':idx,'region':region,'assessment':assessment,'error_px':err})
                row=dict(timestamp='',frame=idx,video_time_s=idx/args.fps,x_px=out['x_px'],x_cm='',valid=int(out['valid']),confidence=out['confidence'],state=out['state'],predicted=int(out['predicted']),method=out['method'],fps=args.fps,algo_ms=ms,wait_ms='',pipe_ms='',draw_ms='',write_ms='',frame_ms='',rail_slope=rail.slope,rail_intercept=rail.intercept,rail_valid=int(rail.valid),pose_ok=0,capture_monotonic_ms='',feedback_valid=0,feedback_reason='offline',cm_valid=0,cm_status='new_coordinate_uncalibrated',reference_x_px='',reference_ms='',rail_geometry_valid=int(rail.geometry_valid),rail_identity_valid=int(rail.identity_valid),rail_reason=rail.reason,roi_min_px='' if bd.last_roi is None else bd.last_roi[0],roi_max_px='' if bd.last_roi is None else bd.last_roi[1],candidate_count=len(cs),detector_reason=bd.last_reason,tracker_reason=out.get('reason',''),evaluation_region=region,manual_x_px=true_x,assessment=assessment,error_px=err,coordinate_version='relative-cap-v1' if name in ('V3','V4') else 'legacy')
                rows[name].append(row)
                if name=='V4':
                    out['cm_status']='uncalibrated';vis=draw_overlay(frame,rail,cs,out,args.fps,ms,None,calibrated=False)
                    if bd.last_roi is not None:
                        for bound in bd.last_roi:
                            x=int(round(bound));y=int(round(rail.y_at(x)));cv2.line(vis,(x,y-45),(x,y+15),(0,255,255),2)
                    cv2.putText(vis,'frame %d | %s / %s'%(idx,bd.last_reason,out.get('reason','')),(15,700),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
                    encoder.stdin.write(vis.tobytes())
            idx+=1
    finally:
        cap.release();encoder.stdin.close();returncode=encoder.wait(timeout=30)
    if returncode:raise RuntimeError('encoder failed')
    assert_coverage(idx,args.expected_frames,shape)
    encoded=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-count_frames','-show_entries','stream=nb_read_frames','-of','json',str(target)]))
    if int(encoded['streams'][0]['nb_read_frames'])!=idx:raise ValueError('encoded video frame loss')
    result={'frames':idx,'media_fps':args.fps,'source_sha256':hashlib.sha256(Path(args.video).read_bytes()).hexdigest(),'labels_scope':labels['audit_type'],'unlabeled_frames':idx-len(truth),'labeled_frames':len(truth),'encoded_frames':idx,'versions':{}}
    for name,rr in rows.items():
        with (output/(name+'.csv')).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rr[0]));w.writeheader();w.writerows(rr)
        counts={}
        for s in sample_metrics[name]:
            key=s['region']+'_'+s['assessment'];counts[key]=counts.get(key,0)+1
        errors=[s['error_px'] for s in sample_metrics[name] if s['assessment']=='correct']
        longest=run=0
        for r in rr:
            run=run+1 if not r['valid'] else 0;longest=max(longest,run)
        result['versions'][name]={'reported_valid_frames':sum(r['valid'] for r in rr),'rail_valid_frames':sum(r['rail_valid'] for r in rr),'sample_counts':counts,'sample_correct_error_mean_px':float(np.mean(errors)) if errors else None,'sample_correct_error_max_px':float(max(errors)) if errors else None,'algo_mean_ms_desktop':float(np.mean([r['algo_ms'] for r in rr])),'longest_invalid_run_all_frames_not_core_only':longest}
    (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
