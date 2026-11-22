"""用现行运动观测入口与真实搜索边界采样，两种坐标共享同一帧检测与身份判断。"""
import argparse
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

from refit import old, CAL
import motion_observe as live
from calibrate_rail import save_new


TODAY = old.HOST.parent/'8/continuity_20261122/runs'
YESTERDAY = old.HOST/'motion_20261121/runs'
MODES = ('raw', 'points')


def events(folder, nominal):
    if not nominal:
        yield from live.replay(folder)
        return
    # 首次静态负六没有采集时钟，使用已有审计所允许的唯一名义时钟。
    stream = live.packets(['ffmpeg','-nostdin','-loglevel','error','-f','mjpeg','-i',
                           str(folder/'raw.mjpg'),'-c:v','copy','-f','mpjpeg','pipe:1'])
    for i,jpeg in enumerate(stream):
        yield i,i/30.,jpeg


def replay(folder, dest, configs, count, cm, nominal=False):
    logs = {mode:[] for mode in MODES}
    bindings = {}
    roi = {}
    # 守护式模糊补搜需要真实读帧时间；静态旧入口允许省略该实参。
    # 时间来自原始回放迭代器，每帧同时核对检测与跟踪的时钟。
    raw_config = old.json.loads(configs['raw'].read_text())
    guarded = raw_config['ball'].get('motion_blur_recovery',False)
    def timed_frames():
        for sample in live.decoded(events(folder,nominal)):
            roi['clock'] = sample[1]
            yield sample
    dest.mkdir()
    class Detector(live.BallDetector):
        def detect(self,frame,gray,rail,ts=None,*,timestamp=None):
            if timestamp is not None:
                assert ts is None
                ts = timestamp
            if ts is None:
                assert not guarded, 'guarded detector received no timestamp'
                result = super().detect(frame,gray,rail)
            else:
                assert ts == roi['clock']
                result = super().detect(frame,gray,rail,ts)
            roi['value'] = self.last_roi
            return result
    class Tracker(live.BallTracker):
        def update(self,candidates,ts,search_bounds):
            # 缺省或遗漏第三个实参会直接失败；每帧核对实参来自原检测器。
            assert ts == roi.pop('clock') and search_bounds == roi.pop('value')
            return super().update(candidates,ts,search_bounds)
    class Coordinates(live.RailCoordinates):
        def __init__(self,cfg,geometry,base):
            super().__init__(cfg,geometry,base)
            other = old.json.loads(configs['points'].read_text())
            other['rail']['upper_reference']['image'] = cfg['rail']['upper_reference']['image']
            assert {k:v for k,v in other.items() if k!='camera_geometry'} == {k:v for k,v in cfg.items() if k!='camera_geometry'}
            self.points_mode = old.RailCoordinates(other,old.CameraGeometry(other['camera_geometry'],1280,720,base),base)
            assert self.calibration is None and self.points_mode.calibration is None
            bindings.update(raw=self.binding,points=self.points_mode.binding)
        def process(self,out,candidates,rail,i):
            raw = super().process(out,candidates,rail,i)
            points = self.points_mode.process(out,candidates,rail,i)
            for mode,pos in zip(MODES,(raw,points)):
                assert not pos['cm_valid'] and not pos['feedback_valid']
                logs[mode].append(dict(frame=i,predicted=out['predicted'],**pos))
            return raw
    with ExitStack() as stack:
        for name,obj in [('RailCoordinates',Coordinates),('BallTracker',Tracker),('BallDetector',Detector)]:
            stack.enter_context(patch.object(live,name,obj))
        live.observe(configs['raw'],timed_frames(),dest/'production.csv',False)
    sessions = {}
    for mode in MODES:
        data = logs[mode]
        assert [r['frame'] for r in data] == list(range(count))
        path,side = dest/(mode+'.csv'),dest/(mode+'.binding.json')
        old.write_rows(path,data)
        old.json_out(side,bindings[mode])
        if cm is not None:
            args = SimpleNamespace(video=folder/'raw.mjpg',csv=path,cm=cm,known_position=True,start_frame=0,end_frame=count-1)
            sessions[mode] = old.collect(args,bindings[mode],side)
    return sessions


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('dest',type=Path)
    p.add_argument('--variant',choices=('original','photoedge','edgeknot'),default='original')
    p.add_argument('--raw-config', type=Path)
    p.add_argument('--points-config', type=Path)
    group=p.add_mutually_exclusive_group()
    group.add_argument('--blind', choices=['verify_m5_blind'], help='独立盲测；之前两次负五均为开发检验')
    group.add_argument('--repeat',help='操作者确认的今日第二次独立负五摆放目录名')
    a=p.parse_args()
    dest=a.dest.resolve()
    dest.mkdir()
    suffix='20261121' if a.variant=='original' else a.variant+'_20261122'
    configs={m:old.JET/('config_motion_left_'+s+'_'+suffix+'.json') for m,s in zip(MODES,('print','points'))}
    if a.raw_config or a.points_config:
        if not (a.raw_config and a.points_config):
            p.error('supply both --raw-config and --points-config')
        configs={'raw':a.raw_config.resolve(),'points':a.points_config.resolve()}
    rcfg=old.json.loads(configs['raw'].read_text())
    pcfg=old.json.loads(configs['points'].read_text())
    assert rcfg['ball_search']['enabled'] and [c['camera_geometry']['mode'] for c in (rcfg,pcfg)]==list(MODES)
    assert rcfg['rail']['upper_reference']['image']==pcfg['rail']['upper_reference']['image']
    paths=[old.JET/f for f in ('motion_observe.py','local_vision.py','ball_detector.py','ball_tracker.py','rail_detector.py',
           'ruler_reference.py','camera_geometry.py','rail_coordinates.py','measurement_contract.py','rail_calibration.py','calibrate_rail.py')]
    paths += [*configs.values(),old.JET/rcfg['rail']['upper_reference']['image'],old.JET/pcfg['camera_geometry']['intrinsics_file']]
    hashes={str(f):old.sha256(f) for f in paths}
    entries=[]
    for r in old.read_csv(CAL/'motion_left_print_20261121_v1/statistics.csv'):
        if r['mode']=='raw':
            cm=int(r['cm']) if r['cm'] else None
            entries.append((YESTERDAY/r['run'],'old_'+r['run'],r['role'],cm,int(r['frames']),r['video_sha256']))
    for run,role,cm in [('verify_m5','development',-5),('verify_p5','verify',5),('verify_zero','repeat_zero',0)]:
        entries.append((TODAY/run,'new_'+run,role,cm,150,None))
    if a.repeat:
        assert Path(a.repeat).name==a.repeat and a.repeat!='verify_m5'
        entries.append((TODAY/a.repeat,'new_'+a.repeat,'verify',-5,150,None))
    if a.blind:
        entries.append((TODAY/'verify_m5_repeat','new_verify_m5_repeat','development',-5,150,None))
    entries.append((YESTERDAY/'fixed_O_tilt','old_fixed_O_tilt','tilt',None,895,'98f1a56007f9a966939aca87a0dffb3f63695365c557c3cfe59ef989d730a2e7'))
    if a.blind:
        entries.append((TODAY/a.blind,'new_'+a.blind,'blind',-5,150,None))
    grouped={m:[] for m in MODES}
    ids={m:set() for m in MODES}
    manifest=[]
    seen=set()
    for folder,run,role,cm,count,expected in entries:
        if role=='blind':
            # 操作者在录制结束后复制到本机；保留整段 150 帧，解码器还会核对帧索引及 JPEG 数量。
            import time
            while not ((folder/'frames.csv').exists() and (folder/'raw.mjpg').exists()
                       and len(old.read_csv(folder/'frames.csv'))>=count):
                time.sleep(1)
        vid=old.sha256(folder/'raw.mjpg')
        assert (expected is None or vid==expected) and vid not in seen
        seen.add(vid)
        nominal=run=='old_fit_m6'
        clock=None if nominal else old.sha256(folder/'frames.csv')
        pair=replay(folder,dest/run,configs,count,cm,nominal)
        if not nominal:
            assert old.sha256(folder/'frames.csv')==clock
        for m in MODES:
            binding=old.json.loads((dest/run/(m+'.binding.json')).read_text())
            ids[m].add(old.json.dumps(binding,sort_keys=True))
            if m in pair:
                grouped[m].append((role,run,pair[m]))
        manifest.append(dict(run=run,role=role,cm=cm,frames=count,video_sha256=vid,clock_sha256=clock))
        print(run,flush=True)
    assert all(len(v)==1 for v in ids.values())
    assert all(old.sha256(f)==h for f,h in hashes.items()), 'source changed during replay'
    old.json_out(dest/'provenance.json',dict(files=hashes,opencv=cv2.__version__,numpy=np.__version__,raw_config=str(configs['raw']),points_config=str(configs['points'])))
    old.write_rows(dest/'manifest.csv',manifest)
    reports=[]
    for m,ss in grouped.items():
        fit=[s for role,_,s in ss if role=='fit']
        verify=[s for role,run,s in ss if role in ('verify','blind') and run!='old_verify_m5_02']
        binding=fit[0]['binding']
        proposed=old.build_model(binding,fit,verify)
        checks=[]
        for role,run,s in ss:
            if role=='fit':
                continue
            values=[old.map_position(proposed['model'],r['rail_s_px']) if r['pixel_valid'] and r['reference_valid'] and r['predicted'] is False and r['rail_s_px'] is not None else None for r in s['rows']]
            checks.append(dict(mode=m,run=run,role=role,cm=s['cm'],**old.metrics(s['cm'],values)))
        # 加载器按同刻度汇总；外部逐摆放门控及独立复拍全部通过后才交付其验证文档。
        # 门控失败时重新调用原生拟合器，不手改 validated、ID、系数或证据。
        passed=bool((a.blind or a.repeat) and proposed['validated'] and all(r['passed'] for r in checks))
        candidate=proposed if passed else old.build_model(binding,fit)
        target=dest/(m+'_candidate.json')
        save_new(target,candidate)
        loader=old.RailCalibration(str(target),binding)
        assert loader.ready and loader.validated==passed and (loader.position(0) is not None)==passed
        old.json_out(dest/(m+'_decision.json'),dict(status=loader.status,static_passed=passed,
                     independent_repeat=a.blind or a.repeat,grouped_validation=proposed['validation']['metrics'],
                     binding_id=candidate['rail_binding_id'],candidate_sha256=old.sha256(target)))
        reports.extend(checks)
    old.write_rows(dest/'heldouts.csv',reports)


if __name__=='__main__':
    main()
