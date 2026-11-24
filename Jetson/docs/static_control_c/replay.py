"""从已确认摆放的完整原片重建 C 模型，复核运动补测中心。仅离线运行。"""
import argparse
import csv
import hashlib
import io
import json
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from types import SimpleNamespace

JET = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(JET))
from motion_observe import BallDetector, BallTracker, CameraGeometry, RailDetector, cv2, decoded, packets, replay
import numpy as np
from calibrate_rail import collect, sha256
from fusion_motion import MotionCandidates
from rail_calibration import RailCalibration, build_model, map_position
from ruler_reference import similarity_from_points
from static_control_detection import supplement
from static_control_geometry import StaticControlCoordinates

REV = '08147c003476eb2b0eed792978d09b8ec8b6b820'
CFG = JET/'config_static_control_c.json'


def archive(name):
    # Git 稀疏检出可能没有展开标注；按冻结提交读取既有实物标注和核对清单。
    return subprocess.check_output(['git', 'show', REV+':'+name], cwd=str(JET))


def save(path, obj):
    with path.open('x', encoding='utf-8') as f:
        json.dump(obj, f, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
        f.write('\n')


def rows(raw):
    return list(csv.DictReader(io.StringIO(raw.decode('utf-8-sig'))))


def write_rows(path, records):
    with path.open('x', newline='') as f:
        w = csv.DictWriter(f, records[0].keys())
        w.writeheader()
        w.writerows(records)


def components(config=CFG):
    cfg = json.loads(config.read_text())
    ref = cfg['rail']['upper_reference']
    ref['image'] = str(JET/ref['image'])
    geo = CameraGeometry(cfg['camera_geometry'], 1280, 720, JET)
    # 采样与结果模型的文件路径不参与测量绑定；所有采样均在未加载厘米模型时进行。
    cfg['rail_coordinates']['calibration_file'] = None
    if ref.get('fast_search') is True:
        from static_fast_reference import make_rail_detector
        from static_fast_geometry import StaticFastCoordinates
        coords = StaticFastCoordinates(cfg, geo, JET)
        detector = make_rail_detector(cfg['rail'], 1280, 720)
    elif 'digit_boxes' in ref:
        # 使用与生产入口相同的工厂；历史配置保持旧的参考和原有 C 绑定。
        from static_control_reference import make_rail_detector
        from static_reference_geometry import StaticReferenceCoordinates
        coords = StaticReferenceCoordinates(cfg, geo, JET)
        detector = make_rail_detector(cfg['rail'], 1280, 720)
    else:
        coords = StaticControlCoordinates(cfg, geo, JET)
        detector = RailDetector(cfg['rail'], 1280, 720)
    return (coords, detector,
            BallDetector(dict(cfg['ball'], ball_search=cfg['ball_search']),1280,720),
            MotionCandidates(), BallTracker(cfg['tracking'],1280,720))


def events(folder, nominal):
    if not nominal:
        yield from replay(folder)
        return
    for i, jpg in enumerate(packets(['ffmpeg','-nostdin','-loglevel','error','-f','mjpeg',
                                    '-i',str(folder/'raw.mjpg'),'-c:v','copy','-f','mpjpeg','pipe:1'])):
        yield i, i / 30., jpg


def replay_clip(task):
    dest, tag, folder, expected, dynamic, truth, schedule, config = task
    cv2.setNumThreads(1)
    coords, rd, bd, motion, tracker = components(config)
    video = folder/'raw.mjpg'
    assert sha256(video) == expected
    nominal = tag == 'old_fit_m6'
    clock = None if nominal else sha256(folder/'frames.csv')
    result = []
    for i, ts, frame, gray, _ in decoded(events(folder, nominal)):
        if schedule is not None and i not in schedule:
            continue
        if schedule is not None:
            assert abs(schedule[i]-ts) < 1e-6
        assert gray is not None
        rail = rd.detect(gray, force=True)
        original = bd.detect(frame, gray, rail, timestamp=ts)
        proposals, bounds = supplement(motion, gray, rail, bd.last_roi, ts, original)
        out = tracker.update(proposals, ts, bounds)
        p = coords.process(out, proposals, rail, i)
        measured = p['rail_s_px'] is not None
        source = proposals[0].extra.get('proposal_source', 'hough') if measured else ''
        r = dict(frame=i, timestamp=ts, original_count=len(original), candidate_count=len(proposals),
                 source=source, predicted=out['predicted'], pixel_valid=p['pixel_valid'],
                 reference_valid=p['reference_valid'], calibration_valid=p['calibration_valid'],
                 cm_valid=p['cm_valid'], rail_s_px=p['rail_s_px'], rail_binding_id=coords.binding_id,
                 x_px=p['measurement_x_px'], y_px=p['measurement_y_px'])
        if 'static_reference' in coords.binding:
            # 留存各段实际走过的参考路径，便于区分旧参考通过与数字补测。
            registration = getattr(rail, 'registration', None) or {}
            r.update(rail_reason=rail.reason, reference_set=registration.get('reference_set'),
                     registration_mode=registration.get('registration_mode'))
        if dynamic:
            # 用同一帧的 POINTS 内参、尺配准和轴分别投影实测/标注中心；动态真值仅为图像中心。
            if source != 'motion':
                continue
            tr = truth[i]
            assert abs(float(tr['read_s'])-ts) < 1e-6
            r.update(label=tr['label'], truth_x=tr['center_x_px'], truth_y=tr['center_y_px'],
                     truth_s_px=None, center_error_px=None)
            if tr['center_x_px'] and tr['center_y_px']:
                xy = np.array([float(tr['center_x_px']),float(tr['center_y_px'])])
                geo = coords.geometry
                transform, _ = similarity_from_points(geo.correct_points(rail.registration['reference_points']),
                                                      geo.correct_points(rail.registration['current_points']))
                inv, shift = transform
                point = inv.dot(geo.correct_points([xy])[0]-shift)
                r['truth_s_px'] = float(np.dot(point-coords.axis_points[1],coords.tangent))
                r['center_error_px'] = float(np.linalg.norm(xy-[r['x_px'],r['y_px']]))
        result.append(r)
    if not dynamic:
        assert [r['frame'] for r in result] == list(range(150))
    assert sha256(video) == expected and (nominal or sha256(folder/'frames.csv') == clock)
    if result:
        write_rows(dest/(tag+'.csv'), result)
    return tag, result, coords.binding, clock


def run(data, dest, target, config=CFG):
    dest.mkdir(parents=True, exist_ok=False)
    config_hash = sha256(config)
    binding = components(config)[0].binding
    side = dest/'binding.json'
    save(side, binding)
    historical = rows(archive('docs/continuity_20261122/geometry/release/checks.csv'))
    placements = [r for r in historical if r['mode']=='points' and r['confirmed_cm']]
    # 独立验证沿用历史正式模型的预定分组，仅复用位置、原片身份和角色。
    # 独立零点复拍、开发摆放及第二次负五均另做逐摆放门控，避免合并掩盖覆盖缺口。
    oldmodel = json.loads((JET/'assets/topview_20261120/rail_cm_motion_20261122.json').read_text())
    verify_hash = {s['source']['video_sha256'] for s in oldmodel['validation']['sessions']}
    todo, metadata = [], {}
    for p in placements:
        group, name = p['run'].split('_',1)
        folder = data/('7/motion_20261121' if group=='old' else '8/continuity_20261122')/'runs'/name
        todo.append((dest, p['run'], folder, p['video_sha256'], False, None, None, config))
        metadata[p['run']] = (p, folder)
    fit, verify, all_sessions, checks = [], [], {}, []
    with ProcessPoolExecutor(max_workers=4) as pool:
        for tag, result, actual, clock in pool.map(replay_clip, todo):
            assert binding == actual
            p, folder = metadata[tag]
            args = SimpleNamespace(known_position=True, cm=float(p['confirmed_cm']), start_frame=0, end_frame=149,
                                   video=folder/'raw.mjpg', csv=dest/(tag+'.csv'))
            session = collect(args, binding, side)
            all_sessions[tag] = session
            if p['role']=='fit':
                fit.append(session)
            elif p['video_sha256'] in verify_hash:
                verify.append(session)
            checks.append(dict(run=tag, role=p['role'], cm=args.cm, total=150, clock_sha256=clock,
                               video_sha256=p['video_sha256'], csv_sha256=sha256(args.csv),
                               motion_outputs=sum(r['source']=='motion' for r in result)))
            print(tag, flush=True)
    assert len(fit)==3 and {s['source']['video_sha256'] for s in verify} == verify_hash
    model = build_model(binding, fit, verify, 'linear')
    for c in checks:
        ss = all_sessions[c['run']]
        values = [map_position(model['model'], r['rail_s_px']) for r in ss['rows']
                  if r['pixel_valid'] and r['reference_valid'] and r['predicted'] is False and r['rail_s_px'] is not None]
        errors = [abs(v-c['cm']) for v in values if v is not None]
        c.update(axis_samples=len(values), mapped=len(errors), mae_cm=float(np.mean(errors)) if errors else None,
                 max_cm=max(errors) if errors else None)
        c['passed'] = None if c['role']=='fit' else len(errors)>=143 and max(errors)<=.3 and len(errors)==len(values)
    write_rows(dest/'placements.csv', checks)
    if sha256(config) != config_hash or components(config)[0].binding != binding:
        raise RuntimeError('configuration or bound source changed during replay')
    # 按完整录像逐摆放门控；失败的开发片也保留在表内，不以跨摆放的均值覆盖。
    if not model['validated'] or any(c['passed'] is False for c in checks):
        save(dest/'rejected_candidate.json', model)
        raise ValueError('placement validation failed; see placements.csv and rejected_candidate.json')
    save(target, model)
    assert RailCalibration(str(target), binding).validated
    return model


def dynamic(data, dest, model, config=CFG):
    scene='moving_final'
    folder=data/'8/continuity_20261122/runs'/scene
    manifests=rows(archive('docs/vision_candidate_fusion/results/frozen/processed/inputs.csv'))
    inputs={r['kind']:r['sha256'] for r in manifests if r['scene']==scene}
    tr=archive('docs/continuity_20261122/truth/moving_final.csv')
    assert hashlib.sha256(tr).hexdigest()==inputs['truth']
    assert sha256(folder/'frames.csv')==inputs['capture'] and sha256(folder/'measurements.csv')==inputs['schedule']
    truth={int(r['frame']):r for r in rows(tr)}
    schedule={int(r['frame']):float(r['timestamp']) for r in rows((folder/'measurements.csv').read_bytes())}
    metrics=[]
    for tag, selected in [('processed',schedule),('all_frames',None)]:
        _, result, actual, _ = replay_clip((dest, 'moving_'+tag, folder, inputs['video'], True, truth, selected, config))
        assert actual==model['binding']
        # 原始冻结实验在同一日程下的全部 C 运动输出应逐帧复现。
        frozen=rows(archive('docs/vision_candidate_fusion/results/frozen/'+tag+'/frames.csv'))
        ref={int(r['frame']):r for r in frozen if r['scene']==scene and r['mode']=='C' and r['source']=='motion'}
        assert set(ref)=={r['frame'] for r in result}
        errors=[]
        for r in result:
            base=ref[r['frame']]
            assert abs(float(base['x'])-r['x_px'])<.0001 and abs(float(base['y'])-r['y_px'])<.0001
            est=map_position(model['model'],r['rail_s_px'])
            expected=map_position(model['model'],r['truth_s_px'])
            r.update(measured_cm=est, image_truth_cm=expected,
                     center_delta_cm=abs(est-expected) if est is not None and expected is not None else None)
            if r['center_delta_cm'] is not None:
                errors.append(r['center_delta_cm'])
        write_rows(dest/('moving_'+tag+'_projected.csv'),result)
        metrics.append(dict(schedule=tag, total_motion=len(result), in_domain_pairs=len(errors),
                            published_cm=sum(r['measured_cm'] is not None for r in result),
                            max_center_error_px=max(r['center_error_px'] for r in result),
                            mean_center_delta_cm=float(np.mean(errors)),max_center_delta_cm=max(errors),
                            above_03=sum(x>.3 for x in errors)))
    save(dest/'dynamic.json', dict(archive_commit=REV, inputs=inputs, metrics=metrics))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('data', type=Path)
    p.add_argument('new_evidence_dir', type=Path)
    p.add_argument('new_model', type=Path)
    p.add_argument('--config', type=Path, default=CFG, help='与生产入口一致的配置；默认复现原方案 C')
    a=p.parse_args()
    config=a.config.resolve()
    model=run(a.data.resolve(),a.new_evidence_dir.resolve(),a.new_model.resolve(),config)
    # 旧动态对照锁定旧参考的逐帧输出，数字参考的正式任务仅验收已知静态刻度。
    if 'static_reference' not in model['binding']:
        dynamic(a.data.resolve(),a.new_evidence_dir.resolve(),model,config)
