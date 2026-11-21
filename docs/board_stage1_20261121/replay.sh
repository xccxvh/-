#!/bin/bash
# 在单独的 Nano 部署目录运行；两次回放读取同一原片，完整保留无效帧。
set -eu
root=$(cd "$(dirname "$0")/../.." && pwd)
run=$(realpath "$1")
cd "$root/Jetson"
for mode in raw points; do
    cfg=config_topview_circle_20261120.json
    if [ "$mode" = points ]; then cfg=config_board_points.json; fi
    label="$(basename "$run")_$mode"
    OPENBLAS_CORETYPE=ARMV8 python3 -B main.py --config "$cfg" --video "$run/raw.mjpg" --fps 30 --csv --label "$label" > "$run/$mode.log" 2>&1
    log=$(sed -n 's/^csv: //p' "$run/$mode.log")
    test -n "$log"
    cp -n "$log" "$run/$mode.csv"
    cp -n "${log%.csv}.binding.json" "$run/$mode.binding.json"
done
