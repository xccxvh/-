"""俯视运动采集及同帧回放。原始 MJPEG 全幅直存，输出以 Nano 读帧单调时钟关联。"""
import argparse
import csv
import json
import os
import subprocess
import threading
import time
from pathlib import Path

# Nano 的 OpenBLAS 需选择 ARMv8。此入口仅对很小的矩阵求逆，多线程调度开销大。
if os.uname().machine == 'aarch64':
    os.environ.setdefault('OPENBLAS_CORETYPE', 'ARMV8')
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import cv2
import numpy as np
from camera_geometry import CameraGeometry
from rail_coordinates import RailCoordinates
from rail_detector import RailDetector
from ball_detector import BallDetector
from ball_tracker import BallTracker


def packets(command):
    """通过长度读取完整 JPEG。采集使用 ffmpeg 直拷相机负载，不二次压缩。"""
    proc = subprocess.Popen(command, stdout=subprocess.PIPE)
    try:
        while True:
            line = proc.stdout.readline()
            if not line:
                break
            if line.lower().startswith(b'content-length:'):
                size = int(line.split(b':')[1])
                if proc.stdout.readline().strip():
                    raise ValueError('invalid MJPEG header')
                jpeg = proc.stdout.read(size)
                if len(jpeg) != size:
                    raise EOFError('incomplete JPEG')
                yield jpeg
        if proc.wait():
            raise RuntimeError('ffmpeg input failed')
    finally:
        proc.stdout.close()
        if proc.poll() is None:
            proc.terminate()
        proc.wait()


def live(directory, seconds):
    """单槽保存最新帧。录像保留处理线程跳过的帧，frames.csv 保留每帧读完时刻。"""
    cond = threading.Condition()
    latest = []
    done = threading.Event()
    stop = threading.Event()
    errors = []
    def collect():
        source = packets(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-f','v4l2',
                          '-input_format','mjpeg','-video_size','1280x720','-framerate','30',
                          '-i','/dev/video0','-c:v','copy','-f','mpjpeg','pipe:1'])
        try:
            with (directory/'raw.mjpg').open('xb') as raw, (directory/'frames.csv').open('x') as timing:
                writer = csv.writer(timing)
                writer.writerow(('frame','read_s','raw_write_ms'))
                for i, jpeg in enumerate(source):
                    ts = time.monotonic()
                    if i == 0:
                        start = ts
                    with cond:
                        latest[:] = [(i,ts,jpeg)]
                        cond.notify()
                    write_start = time.monotonic()
                    raw.write(jpeg)
                    # 单独记录压缩负载交给文件缓冲的耗时，不含强制同步到磁盘的耗时。
                    writer.writerow((i,ts,(time.monotonic()-write_start)*1000))
                    if ts-start >= seconds or stop.is_set():
                        break
                if not latest:
                    raise EOFError('camera produced no frames')
        except Exception as exc:
            errors.append(exc)
        finally:
            source.close()
            with cond:
                done.set()
                cond.notify()
    thread = threading.Thread(target=collect)
    thread.start()
    last = -1
    try:
        while True:
            with cond:
                cond.wait_for(lambda: done.is_set() or (latest and latest[0][0] != last))
                if not latest or latest[0][0] == last:
                    break
                item = latest[0]
            last = item[0]
            yield item
        if errors:
            raise errors[0]
    finally:
        stop.set()
        thread.join()


def replay(directory):
    """回放复用实时时间戳计算速度；帧号、原始视频和采集日志必须一一对应。"""
    source = packets(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-f','mjpeg',
                      '-i',str(directory/'raw.mjpg'),'-c:v','copy','-f','mpjpeg','pipe:1'])
    try:
        with (directory/'frames.csv').open() as f:
            for expected, row in enumerate(csv.DictReader(f)):
                if int(row['frame']) != expected:
                    raise ValueError('non-consecutive capture index')
                jpg = next(source, None)
                if jpg is None:
                    raise ValueError('missing JPEG')
                yield expected, float(row['read_s']), jpg
        if next(source, None) is not None:
            raise ValueError('extra JPEG')
    finally:
        source.close()


def decoded(samples):
    """使用与既有检测相同的 BGR 解码和灰度转换，避免直接取 JPEG 亮度造成测量变化。"""
    for i, ts, jpeg in samples:
        start = time.monotonic()
        frame = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        gray = None
        if frame is not None and frame.shape[:2] == (720,1280):
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        yield i,ts,frame,gray,time.monotonic()-start


def latest_decoded(samples):
    """解码与上一帧的 Hough 并行。单槽可替换，不要求识别线程处理积压的图像。"""
    cond = threading.Condition()
    slot = []
    stop = threading.Event()
    done = threading.Event()
    errors = []
    def decode_loop():
        try:
            for item in decoded(samples):
                with cond:
                    slot[:] = [item]
                    cond.notify()
                if stop.is_set():
                    break
        except Exception as exc:
            errors.append(exc)
        finally:
            samples.close()
            with cond:
                done.set()
                cond.notify()
    worker = threading.Thread(target=decode_loop)
    worker.start()
    try:
        while True:
            with cond:
                cond.wait_for(lambda: slot or done.is_set())
                if not slot:
                    break
                item = slot.pop()
            yield item
        if errors:
            raise errors[0]
    finally:
        stop.set()
        worker.join()


def observe(cfgfile, samples, log, realtime):
    cfg = json.loads(cfgfile.read_text())
    # 相对资源路径以配置文件所在目录解析，旧版回放可使用独立基线代码与参考图。
    ref = cfg['rail']['upper_reference']
    ref['image'] = str((cfgfile.parent / ref['image']).resolve())
    geo = CameraGeometry(cfg['camera_geometry'],1280,720,cfgfile.parent)
    if cfg['ball']['mode'] != 'topview_circle' or geo.context['mode'] not in ('raw','points'):
        raise ValueError('requires raw topview circle pixels')
    coords = RailCoordinates(cfg,geo,cfgfile.parent)
    rd = RailDetector(cfg['rail'],1280,720)
    bd = BallDetector(dict(cfg['ball'],ball_search=cfg['ball_search']),1280,720)
    tracker = BallTracker(cfg['tracking'],1280,720)
    cv2.setNumThreads(1)
    fields = ('frame timestamp result_s pixel_valid x_px y_px x_cm cm_valid confidence '
              'reference_valid rail_s_px origin_x_px origin_y_px residual_px rail_reason detector_reason '
              'tracker_state tracker_reason coordinate_status candidates rotation_deg rail_angle_deg reference_scale registration_mode decode_ms reference_and_edge_ms detector_ms '
              'tracker_ms coordinates_ms total_ms read_to_output_ms').split()
    with log.open('x') as f:
        writer = csv.DictWriter(f,fields)
        writer.writeheader()
        for i,ts,frame,gray,decode_time in samples:
            if gray is None:
                writer.writerow(dict(frame=i,timestamp=ts,pixel_valid=False,cm_valid=False,
                                     reference_valid=False,confidence=0,coordinate_status='decode_failed'))
                continue
            t1 = time.monotonic()
            rail = rd.detect(gray,force=True)
            t2 = time.monotonic()
            # 补检测只读取同一原帧的时间戳，日志和测速均使用与跟踪一致的时钟。
            if cfg['ball'].get('motion_blur_recovery'):
                candidates = bd.detect(frame, gray, rail, timestamp=ts)
            else:
                candidates = bd.detect(frame,gray,rail)
            t3 = time.monotonic()
            # 本帧的物理搜索边界供出界重获核验；参考失效时清除边界证据。
            bounds = bd.last_roi if cfg.get('ball_search', {}).get('enabled') else None
            out = tracker.update(candidates, ts, bounds)
            t4 = time.monotonic()
            pos = coords.process(out,candidates,rail,i)
            end = time.monotonic()
            pixel = pos['rail_s_px'] is not None
            # 预测状态、参考失效或候选歧义都保留为空；厘米还需当前绑定通过独立验证。
            row = [i,ts,end if realtime else None,pixel,pos['measurement_x_px'] if pixel else None,
                   pos['measurement_y_px'] if pixel else None,pos['rail_x_cm'],pos['cm_valid'],
                   out['confidence'] if pixel else 0,pos['reference_valid'],pos['rail_s_px'],
                   pos['origin_x_px'],pos['origin_y_px'],pos['reference_residual_px'],rail.reason,
                   bd.last_reason,out['state'],out.get('reason'),pos['rail_coordinate_status'],len(candidates),
                   pos['reference_rotation_deg'],pos['rail_angle_deg'],pos['reference_scale'],
                   (getattr(rail,'registration',None) or {}).get('registration_mode')]
            row.append(round(decode_time*1000,3))
            row.extend(round((b-a)*1000,3) for a,b in zip((t1,t2,t3,t4),(t2,t3,t4,end)))
            row.append(round((decode_time+end-t1)*1000,3))
            row.append(round((end-ts)*1000,3) if realtime else None)
            writer.writerow(dict(zip(fields,row)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config',type=Path)
    parser.add_argument('output',type=Path,help='本轮新目录')
    parser.add_argument('--replay',type=Path,help='含 raw.mjpg 和 frames.csv 的既有录像目录')
    parser.add_argument('--seconds',type=float,default=30)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    stream = replay(args.replay) if args.replay else live(args.output,args.seconds)
    frames = decoded(stream) if args.replay else latest_decoded(stream)
    try:
        observe(args.config.resolve(),frames,args.output/'measurements.csv',args.replay is None)
    finally:
        frames.close()
        stream.close()
