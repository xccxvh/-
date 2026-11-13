#!/bin/sh
# 固定当前机位的摄像头参数。
#
# 当前场景背景很暗, 自动曝光会把曝光时间拉到 60 ms, 实际只有 16.8 fps;
# 手动 30 ms + gain 45 可以稳定到 29-30 fps。换机位/光照后需要重新
# 检查曝光和增益, 不要盲目沿用。
#
# 用法: ./set_camera.sh [/dev/video0]
DEV="${1:-/dev/video0}"

v4l2-ctl -d "$DEV" --set-ctrl=exposure_auto=1
v4l2-ctl -d "$DEV" --set-ctrl=exposure_absolute=300
v4l2-ctl -d "$DEV" --set-ctrl=gain=45
# 保持自动白平衡; 若现场光照稳定, 可改为固定色温:
# v4l2-ctl -d "$DEV" --set-ctrl=white_balance_temperature_auto=0
# v4l2-ctl -d "$DEV" --set-ctrl=white_balance_temperature=5000
v4l2-ctl -d "$DEV" --set-ctrl=white_balance_temperature_auto=1

echo "current:"
v4l2-ctl -d "$DEV" --get-ctrl=exposure_auto,exposure_absolute,gain,white_balance_temperature_auto
