"""Nano 上以真实相机帧间隔回放运动原片，沿用实机解码和丢帧策略；结果属于定时回放。"""
import argparse
import csv
import time
from pathlib import Path
from unittest.mock import patch
import motion_observe as motion

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config',type=Path)
    parser.add_argument('recording',type=Path)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    with (args.recording/'frames.csv').open() as f:
        clock = [float(x['read_s']) for x in csv.DictReader(f)]
    # 回放开始前从磁盘读取完整的压缩 JPEG。预载只属于时钟模拟器；
    # 实际送入实时入口仍为单槽，每帧在记录的采集时间才对解码线程可见。
    cmd = ['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-f','mjpeg',
           '-i',str(args.recording/'raw.mjpg'),'-c:v','copy','-f','mpjpeg','pipe:1']
    data = list(motion.packets(cmd))
    if len(data) != len(clock):
        raise ValueError('JPEG and capture timestamp counts differ')
    def playback(_):
        start = time.monotonic()
        for jpeg,ts in zip(data,clock):
            delay = start + ts - clock[0] - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            yield jpeg

    args.output.mkdir(parents=True,exist_ok=False)
    with patch.object(motion,'packets',playback):
        stream = motion.latest_decoded(motion.live(args.output,clock[-1]-clock[0]+3))
        try:
            motion.observe(args.config.resolve(),stream,args.output/'measurements.csv',True)
        finally:
            stream.close()
