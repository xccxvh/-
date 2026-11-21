"""按静态拟合的绑定回放固定 O 倾角录像，单列名义零点误差与缺测。"""
import argparse
import json
from statistics import median
from pathlib import Path
import cv2
from replay import JET, STAGE, HOST, read_csv, json_out, replay, sha256, metrics, map_position


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('static', type=Path, help='replay.py 的静态回放输出')
    parser.add_argument('dest', type=Path)
    args = parser.parse_args()
    dest = args.dest.resolve()
    if (HOST/'motion_calibration').resolve() not in dest.parents:
        raise ValueError('输出须位于 7/motion_calibration')
    dest.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    cfg = json.loads(args.config.read_text())
    ref = cfg['rail']['upper_reference']
    ref['image'] = str((args.config.resolve().parent / ref['image']).resolve())
    cfg['rail_coordinates']['calibration_file'] = None
    points = json.loads((STAGE/'config_board_points.json').read_text())['camera_geometry']
    folder = HOST/'motion_20261121/runs/fixed_O_tilt'
    video = folder/'raw.mjpg'
    clock = folder/'frames.csv'
    # 该动作只评估相对现场名义 O 的变化，不为每帧赋机械实测角。
    clip = dict(video=video, run=folder.name, role='unlabelled', cm=None,
                sha256=sha256(video), count=len(read_csv(clock)))
    rows, _, _ = replay(clip, cfg, points, dest)
    models = {m['mode']: m for m in json.loads((args.static/'evaluation.json').read_text())}
    output = []
    for row in rows:
        mode = row['mode']
        model = models[mode]
        static = args.static/'fit_zero'/(mode+'.binding.json')
        tilt = dest/folder.name/(mode+'.binding.json')
        if json.loads(static.read_text()) != json.loads(tilt.read_text()):
            raise ValueError('静态与倾角的配置绑定不一致')
        data = read_csv(dest/folder.name/(mode+'.csv'))
        samples = [float(t['rail_s_px']) for t in data if t['rail_s_px']]
        info = dict(mode=mode, nominal_cm=0, reference=row['reference'],
                    frames=len(data), axis_samples=len(samples), axis_coverage=len(samples)/len(data),
                    median_s_px=median(samples) if samples else None, binding=row['replay_binding'],
                    rail_reasons=json.loads(row['rail_reasons']), video_sha256=clip['sha256'],
                    clock_sha256=sha256(clock), fit_status=model['status'])
        if 'candidate_linear' in model:
            values = [map_position(model['candidate_linear'], float(t['rail_s_px']))
                      if t['rail_s_px'] else None for t in data]
            info.update(metrics(0, values))
        else:
            info.update(valid=0, coverage=0, mae_cm=None, max_abs_cm=None, passed=False)
        output.append(info)
    json_out(dest/'summary.json', output)
    print(output)


if __name__ == '__main__':
    main()
