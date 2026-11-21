"""静态摆放用原始 MJPEG 和读帧时间，与运动采样使用同一解包方式。"""
import csv
import sys
import time
from pathlib import Path
from motion_observe import packets

dest = Path(sys.argv[1])
dest.mkdir(exist_ok=False)
src = packets(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-f','v4l2',
               '-input_format','mjpeg','-video_size','1280x720','-framerate','30',
               '-i','/dev/video0','-c:v','copy','-f','mpjpeg','pipe:1'])
try:
    with (dest/'raw.mjpg').open('xb') as raw, (dest/'frames.csv').open('x') as f:
        writer = csv.writer(f)
        writer.writerow(['frame','read_s'])
        for i in range(150):
            jpeg = next(src)
            ts = time.monotonic()
            raw.write(jpeg)
            writer.writerow([i,ts])
finally:
    src.close()
