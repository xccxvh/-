#!/usr/bin/env sh
#
# 在 PC 上编译并运行滚球控制逻辑的单元测试。
#
# 依赖：gcc（MinGW/MSYS2/Linux 均可），不需要 MSPM0 SDK、Keil 或硬件。
# 用法：在任意目录执行
#   sh Car2/tests/host/run_tests.sh
#
# 产物：
#   tests/host/results/host_test_output.txt
#   tests/host/results/closed_loop_step_response.csv
#
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/../.." && pwd)
cd "$PROJECT_DIR"

mkdir -p tests/host/results

echo "== build: ball control =="
gcc -std=c99 -Wall -Wextra -O2 \
    -I tests/host/stubs -I BSP -I User \
    -o tests/host/ball_test_host.exe \
    tests/host/test_ball_control.c \
    User/App/ball_control.c \
    BSP/SERVO/servo.c \
    tests/host/stubs/system_time_stub.c \
    -lm

echo "== build: vision link =="
gcc -std=c99 -Wall -Wextra -O2 \
    -I tests/host/stubs -I BSP -I User \
    -o tests/host/vision_link_host.exe \
    tests/host/test_vision_link.c \
    BSP/VISION/vision_link.c \
    tests/host/stubs/system_time_stub.c \
    -lm

echo "== run: ball control =="
./tests/host/ball_test_host.exe | tee tests/host/results/host_test_output.txt

echo "== run: vision link =="
./tests/host/vision_link_host.exe | tee tests/host/results/vision_link_test_output.txt

echo "== artifacts =="
ls -l tests/host/results
