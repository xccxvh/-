#!/bin/bash
# 在电脑终端逐次确认实物刻度，交替左右两轮。仅按回车后才采集本次原片。
set -eu
host=jetson@192.168.55.1
local_root=$1
mkdir -p "$local_root"
for round in 1 2; do
    for point in '7 m5' '15 p3' '9 m3' '17 p5'; do
        read -r ruler tag <<< "$point"
        run="verify_${tag}_0${round}"
        printf '\n第 %d 轮：请把球心固定在纸尺 %d cm，然后移开双手。\n' "$round" "$ruler"
        read -r -p '摆好后按回车开始录 150 帧，保持到提示录完：' reply </dev/tty
        ssh -n -oBatchMode=yes "$host" "cd ~/board_stage1_20261121 && python3 -B docs/board_stage1_20261121/capture.py runs/$run"
        echo '原片已录完，正在做同帧 A/B 处理。'
        ssh -n -oBatchMode=yes "$host" "cd ~/board_stage1_20261121 && bash docs/board_stage1_20261121/replay.sh runs/$run"
        rsync -a "$host:board_stage1_20261121/runs/$run/" "$local_root/$run/"
    done
done
echo '两轮采样完成。可以回到对话查看结果。'
