# -*- coding: utf-8 -*-
"""原始画面采集工具(ffmpeg, 直接复制摄像头 MJPG 码流, 不重新编码)。

用法:
  python3 capture_raw.py --label no_ball --duration 5
  python3 capture_raw.py --label roll --duration 10 --width 1920 --height 1080
"""
import argparse
import json
import os
import subprocess
import time

BASE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description="保存 USB 摄像头原始视频")
    ap.add_argument("--label", required=True)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--outdir", default=os.path.join(BASE, "recordings"))
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    name = "raw_%s_%s_%dx%d" % (args.label, stamp, args.width, args.height)
    out = os.path.join(args.outdir, name + ".avi")
    still = os.path.join(args.outdir, name + ".png")

    cmd = [
        "ffmpeg", "-y",
        "-f", "v4l2",
        "-input_format", "mjpeg",
        "-video_size", "%dx%d" % (args.width, args.height),
        "-framerate", str(args.fps),
        "-i", args.device,
        "-t", "%.2f" % args.duration,
        "-c:v", "copy",
        out,
    ]
    print("capture:", " ".join(cmd))
    t0 = time.time()
    rc = subprocess.call(cmd)
    wall = time.time() - t0
    if rc != 0:
        raise SystemExit("ffmpeg 采集失败, 返回码 %d" % rc)

    mid = max(0.1, args.duration / 2.0)
    subprocess.call([
        "ffmpeg", "-y", "-ss", "%.2f" % mid, "-i", out,
        "-frames:v", "1", still,
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 用 ffprobe 数实际帧数, 结合墙钟时间得到真实平均帧率。
    frames = None
    header_fps = None
    try:
        probe = subprocess.check_output([
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-count_frames", "-show_entries",
            "stream=nb_read_frames,avg_frame_rate", "-of", "json", out,
        ])
        pdata = json.loads(probe.decode("utf-8"))
        st = pdata.get("streams", [{}])[0]
        frames = int(st.get("nb_read_frames", 0)) or None
        num, den = st.get("avg_frame_rate", "0/1").split("/")
        if float(den) > 0:
            header_fps = float(num) / float(den)
    except Exception:
        pass

    actual_fps = None
    if frames and args.duration > 0:
        # 用 ffmpeg 的输出时长(由 -t 控制)而不是进程墙钟, 后者包含
        # 摄像头打开/关闭的固定开销。
        actual_fps = frames / args.duration

    size = os.path.getsize(out) if os.path.exists(out) else 0
    info = {
        "label": args.label,
        "video": out,
        "still": still if os.path.exists(still) else None,
        "bytes": size,
        "wall_s": round(wall, 3),
        "frames": frames,
        "header_fps": header_fps,
        "actual_fps": round(actual_fps, 3) if actual_fps else None,
        "resolution": "%dx%d" % (args.width, args.height),
    }
    sidecar = os.path.splitext(out)[0] + ".json"
    with open(sidecar, "w") as f:
        json.dump(info, f, indent=2, ensure_ascii=False)
    print(json.dumps(info, indent=2, ensure_ascii=False))
    print("sidecar:", sidecar)


if __name__ == "__main__":
    main()
