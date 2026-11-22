"""从原片逐帧解码，按现场实际处理帧号驱动四组独立跟踪器；时钟使用采集时间。"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import types

dest = Path(__file__).resolve().parent
jet = dest.parents[2] / 'Jetson'
sys.path.insert(0, str(jet))
from motion_observe import decoded, replay, cv2, RailDetector, BallDetector, BallTracker
import local_vision

def original(name):
    source = subprocess.check_output(['git', '-C', str(jet), 'show', '1e30933:Jetson/'+name+'.py'])
    module = types.ModuleType(name)
    module.__file__ = str(jet / (name+'.py'))
    exec(compile(source, module.__file__, 'exec'), module.__dict__)
    return module

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--guarded', action='store_true', help='启用仅运动模式的受限模糊恢复并独立保存结果')
    args = parser.parse_args()
    prefix = 'guarded_' if args.guarded else ''
    root = Path('/home/user/桌面/jetson_nano/7/motion_20261121/runs/moving_fast_confirmed')
    cfg = json.loads((jet/'config_motion_ruler_digits_20261121.json').read_text())
    cv2.setNumThreads(1)
    with (root/'measurements.csv').open() as fp:
        live = {int(row['frame']): row for row in csv.DictReader(fp)}
    rd = RailDetector(cfg['rail'], 1280, 720)
    options = dict(cfg['ball'], ball_search=cfg['ball_search'], motion_blur_recovery=args.guarded)
    detectors_by_name = {'archived': original('ball_detector').BallDetector(options, 1280, 720),
                         'checkout': BallDetector(options, 1280, 720)}
    old_tracker = original('ball_tracker').BallTracker
    detectors = [('archived', original('local_vision').topview_balls),
                 ('checkout', local_vision.topview_balls)]
    trackers = {k: (old_tracker(cfg['tracking'], 1280, 720), BallTracker(cfg['tracking'], 1280, 720))
                for k, _ in detectors}
    # 两版跟踪器始终共享当帧检测结果；绝不将在线跳过的 JPEG 插入状态历史。
    review = []
    review_frames = (156, 157, 158, 196, 199, 200, 201, 202, 203)
    with (dest/(prefix+'chronology.csv')).open('w') as fp:
        out = csv.writer(fp)
        out.writerow(['frame','read_s','detector','left','right','observed_x_y_diameter_contrast_conf',
                      'archived_valid','archived_reason','new_valid','new_reason','new_x','live_valid',
                      'detector_sources','enclosed_glint_rejections'])
        for i,ts,frame,gray,_ in decoded(sample for sample in replay(root) if sample[0] in live):
            assert abs(ts-float(live[i]['timestamp'])) < 1e-6
            rail = rd.detect(gray, force=True)
            for k, fn in detectors:
                local_vision.topview_balls = fn
                bd = detectors_by_name[k]
                # 新模块用采集时钟检查普通 Hough 圆的时效；固定历史快照仍走当年的接口。
                if k == 'checkout' and args.guarded:
                    candidates = bd.detect(frame, gray, rail, timestamp=ts)
                else:
                    candidates = bd.detect(frame, gray, rail)
                bounds = bd.last_roi if bd.search.enabled else None
                old, new = trackers[k]
                a = old.update(candidates, ts)
                b = new.update(candidates, ts, search_bounds=bounds)
                if b['valid']:
                    assert not b['predicted'] and any(c.x == b['x_px'] for c in candidates)
                values = [[round(x, 4) for x in (c.x,c.extra['measurement_y_px'],c.width,
                           c.extra['appearance'],c.confidence)] for c in candidates]
                out.writerow([i,ts,k,*(bounds or ('','')),json.dumps(values),int(a['valid']),
                              a['reason'],int(b['valid']),b['reason'],b['x_px'] if b['valid'] else '',
                              int(live[i]['pixel_valid']=='True'),
                              ','.join(record.get('proposal_source', 'hough') for record in bd.last_diagnostics
                                       if record.get('reason') == 'accepted'),
                              sum(record.get('reason') == 'no_enclosed_glint' for record in bd.last_diagnostics)])
                if k == 'checkout' and i in review_frames:
                    # 现场同帧裁图，蓝线标出物理左界，十字为圆心，图像独立用于身份目视核验。
                    x, y = (round(value) for value in values[0][:2])
                    tile = frame[y-55:y+55, x-85:x+85].copy()
                    boundary = round(bounds[0])-x+85
                    cv2.line(tile, (boundary, 0), (boundary, 109), (245,120,0), 1)
                    cv2.drawMarker(tile, (85, 55), (0,240,0), cv2.MARKER_CROSS, 10, 1)
                    cv2.putText(tile, '#%d'%i, (4,16), 0, .45, (0,255,255), 1)
                    review.append(tile)
    assert len(review) == len(review_frames)
    sheet = cv2.vconcat([cv2.hconcat(review[j:j+3]) for j in (0, 3, 6)])
    assert cv2.imwrite(str(dest/(prefix+'review.jpg')), sheet)


if __name__ == '__main__':
    main()
