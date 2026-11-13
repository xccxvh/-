# -*- coding: utf-8 -*-
"""汇总 main.py 输出的跟踪 CSV, 供 Test A-E 报告使用。

只依赖标准库, 不改动检测/跟踪逻辑。输出的 x 统计全部是像素单位;
未完成标定时不会生成厘米误差。

用法:
  python3 analyze_track.py logs/track_testB_rolling_xxx.csv
  python3 analyze_track.py logs/*.csv --json logs/summary.json
"""
import argparse
import csv
import json
import os
import statistics


def _load(path):
    with open(path, "r", newline="") as f:
        return [row for row in csv.DictReader(f)]


def _pct(values, q):
    if not values:
        return None
    s = sorted(values)
    k = int(round((len(s) - 1) * q))
    return s[max(0, min(k, len(s) - 1))]


def _stat(values):
    if not values:
        return None
    return {
        "n": len(values),
        "mean": statistics.mean(values),
        "std": statistics.pstdev(values),
        "min": min(values),
        "max": max(values),
        "p05": _pct(values, 0.05),
        "p95": _pct(values, 0.95),
    }


def _float(row, key):
    value = row.get(key, "")
    if value in (None, ""):
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _int(row, key):
    value = row.get(key, "")
    if value in (None, ""):
        return None
    try:
        return int(float(value))
    except ValueError:
        return None


def summarize(path):
    rows = _load(path)
    n = len(rows)
    states = {"DETECTED": 0, "UNCERTAIN": 0, "LOST": 0}
    methods = {}
    predicted = 0
    x_cm_n = 0
    state_valid_mismatch = 0
    timestamps = []
    xs = []
    conf_all = []
    conf_detected = []
    algo = []
    wait = []
    pipe = []
    draw = []
    write = []
    frame_ms_values = []
    jumps = []
    uncertain_runs = []
    current_run = 0
    prev = None

    for row in rows:
        state = row.get("state", "")
        if state in states:
            states[state] += 1
        method = row.get("method", "")
        if method:
            methods[method] = methods.get(method, 0) + 1
        if row.get("predicted") == "1":
            predicted += 1
        x_cm = _float(row, "x_cm")
        if x_cm is not None:
            x_cm_n += 1
        valid = row.get("valid") == "1"
        if valid != (state == "DETECTED"):
            state_valid_mismatch += 1

        ts = _float(row, "timestamp")
        if ts is not None:
            timestamps.append(ts)
        conf = _float(row, "confidence")
        if conf is not None:
            conf_all.append(conf)
        for key, target in (("algo_ms", algo), ("wait_ms", wait),
                            ("pipe_ms", pipe), ("draw_ms", draw),
                            ("write_ms", write),
                            ("frame_ms", frame_ms_values)):
            value = _float(row, key)
            if value is not None:
                target.append(value)

        frame = _int(row, "frame")
        x = _float(row, "x_px")
        if state == "DETECTED" and x is not None:
            xs.append(x)
            if conf is not None:
                conf_detected.append(conf)
            if prev is not None and frame is not None and frame == prev[0] + 1:
                jumps.append(abs(x - prev[1]))
            prev = (frame, x)
        else:
            prev = None

        if state == "UNCERTAIN":
            current_run += 1
        elif current_run:
            uncertain_runs.append(current_run)
            current_run = 0
    if current_run:
        uncertain_runs.append(current_run)

    duration = None
    if len(timestamps) >= 2 and timestamps[-1] > timestamps[0]:
        duration = timestamps[-1] - timestamps[0]
    avg_fps = ((n - 1) / duration) if duration else None

    result = {
        "file": path,
        "frames": n,
        "duration_s": duration,
        "avg_fps_from_timestamps": avg_fps,
        "states": states,
        "detected_pct": (100.0 * states["DETECTED"] / n) if n else 0.0,
        "valid_frames": states["DETECTED"],
        "predicted_frames": predicted,
        "state_valid_mismatch": state_valid_mismatch,
        "methods": methods,
        "x_px": _stat(xs),
        "confidence_all": _stat(conf_all),
        "confidence_detected": _stat(conf_detected),
        "jumps_px": _stat(jumps),
        "algo_ms": _stat(algo),
        "queue_wait_ms": _stat(wait),
        "capture_to_result_ms": _stat(pipe),
        "draw_ms": _stat(draw),
        "write_ms": _stat(write),
        "frame_ms": _stat(frame_ms_values),
        "uncertain_runs": uncertain_runs,
        "x_cm_nonempty": x_cm_n,
    }
    return result


def _fmt(value, fmt="%.2f"):
    if value is None:
        return "-"
    return fmt % value


def print_summary(r):
    print("== %s ==" % r["file"])
    print("frames=%d duration=%ss avg_fps=%s" % (
        r["frames"],
        _fmt(r["duration_s"], "%.2f"),
        _fmt(r["avg_fps_from_timestamps"], "%.2f")))
    s = r["states"]
    print("states: DETECTED=%d (%.1f%%) UNCERTAIN=%d LOST=%d  predicted=%d" % (
        s["DETECTED"], r["detected_pct"], s["UNCERTAIN"], s["LOST"],
        r["predicted_frames"]))
    print("methods: %s" % (
        ", ".join("%s=%d" % (k, v) for k, v in sorted(r["methods"].items()))
        or "-"))
    if r["state_valid_mismatch"]:
        print("WARNING: state/valid mismatch=%d" % r["state_valid_mismatch"])
    labels = [
        ("x_px", "x_px"),
        ("confidence_all", "confidence(all)"),
        ("confidence_detected", "confidence(DETECTED)"),
        ("jumps_px", "jump_px"),
        ("algo_ms", "algo_ms"),
        ("queue_wait_ms", "queue_wait_ms"),
        ("capture_to_result_ms", "capture_to_result_ms"),
        ("draw_ms", "draw_ms"),
        ("write_ms", "write_ms"),
        ("frame_ms", "frame_ms"),
    ]
    for key, label in labels:
        st = r[key]
        if not st:
            print("%s: -" % label)
            continue
        print("%s: n=%d mean=%.2f std=%.2f min=%.2f max=%.2f p05=%.2f p95=%.2f" % (
            label, st["n"], st["mean"], st["std"], st["min"], st["max"],
            st["p05"], st["p95"]))
    if r["uncertain_runs"]:
        runs = r["uncertain_runs"]
        print("UNCERTAIN runs=%d lengths=%s" % (len(runs), runs[:20]))
    else:
        print("UNCERTAIN runs=0")
    print("x_cm nonempty=%d" % r["x_cm_nonempty"])
    print("")


def main():
    ap = argparse.ArgumentParser(description="汇总跟踪 CSV")
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--json", default=None, help="把汇总写入 JSON")
    args = ap.parse_args()

    results = []
    for path in args.csv:
        if not os.path.exists(path):
            raise SystemExit("找不到: %s" % path)
        r = summarize(path)
        print_summary(r)
        results.append(r)

    if args.json:
        with open(args.json, "w") as f:
            json.dump(results if len(results) > 1 else results[0],
                      f, indent=2, ensure_ascii=False)
        print("summary json:", args.json)


if __name__ == "__main__":
    main()
