"""在 Nano 上从原始 MJPEG 预览保存全幅 JPEG；同时只监听滚球遥测。"""
import argparse
import csv
import subprocess
import sys
import time
from contextlib import ExitStack, closing
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'Jetson'))
from position_feedback import LinkJournal, SerialFeedback


def capture(dest, count, url, port):
    dest.mkdir(parents=True, exist_ok=False)
    with ExitStack() as stack:
        # 独立 RX 线程与帧读取时间共用 Nano 单调时钟。此入口没有串口写操作。
        serial = stack.enter_context(closing(SerialFeedback(port)))
        events = stack.enter_context((dest/'telemetry.csv').open('x', newline='', buffering=1))
        journal = LinkJournal(serial, events)
        stack.callback(journal.close)
        stream = stack.enter_context(urlopen(url, timeout=4))
        clock_log = stack.enter_context((dest/'frames.csv').open('x', newline='', buffering=1))
        writer = csv.writer(clock_log, lineterminator='\n')
        writer.writerow(('frame','nano_read_monotonic_ms'))
        # MJPEG 文件由完整 JPEG 顺序组成。回放按帧号关联 frames.csv，封装帧率不参与估时。
        raw = stack.enter_context((dest/'raw.mjpg').open('xb'))
        frame = 0
        while frame < count:
            header = stream.readline()
            if not header:
                raise EOFError('camera preview ended')
            if not header.lower().startswith(b'content-length:'):
                continue
            size = int(header.split(b':',1)[1])
            if stream.readline().strip():
                raise ValueError('unexpected preview header')
            jpeg = stream.read(size)
            stamp = time.perf_counter()*1000
            # 相机 MJPEG 可带填充。按 HTTP Content-Length 确定帧边界，保存完整负载，结束后核对实际可解析帧数。
            if len(jpeg)!=size:
                raise EOFError('multipart frame truncated; partial run retained')
            raw.write(jpeg)
            writer.writerow((frame,stamp))
            frame += 1
    probe = subprocess.check_output(['ffprobe','-v','error','-f','mjpeg','-count_frames',
        '-show_entries','stream=width,height,nb_read_frames','-of','csv=p=0',str(dest/'raw.mjpg')])
    print(probe.decode().strip(),flush=True)
    if probe.decode().strip()!='1280,720,%d'%count:
        raise ValueError('unexpected full-frame video count or resolution')


if __name__=='__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('dest', type=Path)
    p.add_argument('--frames',type=int,default=150)
    p.add_argument('--url',default='http://192.168.55.1:8080/stream.mjpg')
    p.add_argument('--port',default='/dev/serial/by-id/usb-wch.cn_CH32_CMSIS-DAP_B7AD8F061FFECDEFG-if01')
    a=p.parse_args()
    capture(a.dest,a.frames,a.url,a.port)
