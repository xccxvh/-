#!/bin/sh
# 无尺子阶段的像素级测试采集。
#
# 标定需要 -5/0/+5 cm 的已知位置, 当前没有尺子, 先做不依赖厘米的测试:
#   Test A-px: 左/中/右三个位置各静止 10 s, 统计 px 抖动和检出率;
#   Test C-0 : 拿走钢珠 10 s, 统计无钢珠时的误检;
#   Test B   : 钢珠左右滚动 20 s, 统计检出率/跟踪连续性/最大跳变。
#
# 用法: ./run_pixel_tests.sh
# 要求: 相机、轨道、背景人员保持不动; 每次放好球、手离开画面后再按 Enter。
set -e
cd "$(dirname "$0")"

./set_camera.sh

capture() {
    python3 capture_raw.py --label "$1" --duration "$2" \
        --width 1280 --height 720
}

printf '\n[1/5] 把钢珠放在轨道可见段的左侧, 手离开画面, 然后按 Enter...'
read ans
capture testA_left 10

printf '\n[2/5] 把钢珠放在轨道可见段的中间, 手离开画面, 然后按 Enter...'
read ans
capture testA_center 10

printf '\n[3/5] 把钢珠放在轨道可见段的右侧, 手离开画面, 然后按 Enter...'
read ans
capture testA_right 10

printf '\n[4/5] 把钢珠拿走, 手离开画面, 然后按 Enter...'
read ans
capture testC_no_ball 10

printf '\n[5/5] 把钢珠放回轨道, 按 Enter 后立即左右滚动钢珠 20 秒...'
read ans
capture testB_rolling 20

printf '\n采集完成。原始视频在 recordings/ 下, 文件名以 raw_test*.avi 开头。\n'
