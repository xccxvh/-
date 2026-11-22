"""归档板端实录、同帧重放和录制时钟。原片保留在采集目录并以 SHA-256 标识。"""
import csv
import hashlib
import json
import statistics
import sys
import zipfile
from pathlib import Path

from board_metrics import analyze, rows


def main(root, dest):
    live = ('fixed_O_observed', 'parked_free_new', 'moving_free_new', 'moving_dual_live',
            'fixed_m5_final', 'parked_final', 'moving_final')
    metrics = []
    manifest = []
    for run in sorted((root/'runs').iterdir()):
        clock = run/'frames.csv'
        raw = run/'raw.mjpg'
        if not raw.is_file():
            continue
        n = len(rows(clock))
        # 日志可跨电脑检查：保留原板单调时钟，系统日历不用于计算延迟。
        manifest.append(dict(run=run.name, frames=n, bytes=raw.stat().st_size,
                             raw_path=str(raw), raw_sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),
                             clock_sha256=hashlib.sha256(clock.read_bytes()).hexdigest()))
        if run.name in live:
            metrics.append(analyze(run))
    (dest/'live_metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
    with (dest/'captures.csv').open('w') as f:
        w=csv.DictWriter(f,manifest[0].keys());w.writeheader();w.writerows(manifest)
    # 统计独立姿态和历史固定球整段。保留所有缺测，只在有有效厘米时计算误差。
    stats=[]
    for p in sorted((root/'paired/release_cm').glob('*.csv')):
        data=rows(p)
        xs=[float(r['x_cm']) for r in data if r['cm_valid']=='True']
        truth=-5 if p.stem=='verify_m5_blind' else 0
        if xs:
            stats.append(dict(run=p.stem, truth_cm=truth, frames=len(data), valid=len(xs),
                              mean_cm=statistics.mean(xs), sd_cm=statistics.pstdev(xs),
                              mae_cm=statistics.mean(abs(x-truth) for x in xs),
                              max_abs_cm=max(abs(x-truth) for x in xs), lo_cm=min(xs), hi_cm=max(xs)))
    with (dest/'tilt.csv').open('w') as f:
        w=csv.DictWriter(f,stats[0].keys());w.writeheader();w.writerows(stats)
    with zipfile.ZipFile(str(dest/'per_frame.zip'),'w',zipfile.ZIP_DEFLATED) as z:
        for run in sorted((root/'runs').iterdir()):
            for p in sorted(run.glob('*.csv')):
                z.write(str(p),str(p.relative_to(root)))
        for profile in ('baseline','release_digits','release_points','release_cm'):
            for p in sorted((root/'paired'/profile).glob('*.csv')):
                z.write(str(p),str(p.relative_to(root)))


if __name__ == '__main__':
    main(Path(sys.argv[1]).resolve(), Path(__file__).resolve().parent)
