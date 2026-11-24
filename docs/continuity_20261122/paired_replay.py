"""在两个代码快照上重放完全相同的实采帧和时刻，输出可按 frame 联结的 CSV。"""
import argparse
import csv
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('jetson', type=Path, help='待测代码快照的 Jetson 目录')
    parser.add_argument('config', help='该快照内的配置名')
    parser.add_argument('capture', type=Path, help='包含 raw.mjpg 和 frames.csv 的目录')
    parser.add_argument('output', type=Path, help='新的 measurements.csv 路径')
    parser.add_argument('--processed', type=Path, help='可选，复现实板当时实际处理的帧号')
    args = parser.parse_args()
    sys.path.insert(0, str(args.jetson.resolve()))
    from motion_observe import decoded, observe, replay
    times = {}
    if args.processed:
        with args.processed.open() as f:
            for row in csv.DictReader(f):
                times[int(row['frame'])] = float(row['timestamp'])
        if not times:
            raise ValueError('empty process log')
    # 先筛原始帧再驱动有状态的检测器，保持原实测的冷启动和漏帧间隔。
    def selected(source):
        seen = 0
        for i, ts, jpg in source:
            if times and i not in times:
                continue
            if times and abs(ts - times[i]) > 1e-6:
                raise ValueError('capture and measurements clocks disagree')
            seen += 1
            yield i, ts, jpg
        if times and seen != len(times):
            raise ValueError('missing processed frames')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stream = replay(args.capture)
    try:
        observe(args.jetson.resolve()/args.config, decoded(selected(stream)), args.output, False)
    finally:
        stream.close()


if __name__ == '__main__':
    main()
