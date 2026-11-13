# -
26年h题

## 2026 电赛 H 题：车载平衡滚球运动控制系统

当前进度：已完成题目 **要求 2（循迹一圈 + 计时停车）** 的代码实现并在 Keil 中编译通过。

### 工程结构

| 目录 | 说明 |
|---|---|
| `User/main_h2.c` | **当前唯一编译的应用入口**，实现要求 2 |
| `User/main.c` | 旧版多任务测试程序，保留但未加入 Keil 工程编译 |
| `BSP/` | 板级驱动：电机、八路灰度、OLED、按键、串口、定时器等 |
| `Project/core_test.uvprojx` | Keil MDK 工程文件（目标：`KEIL_M0G3519`） |
| `Source/` | TI MSPM0 SDK / CMSIS 源码，工程编译依赖，必须保留 |
| `User/config.syscfg` | SysConfig 引脚与外设配置，改硬件接线时先改这里 |

### 编译与下载

1. 用 Keil MDK 打开 `Project/core_test.uvprojx`；
2. 目标选择 `KEIL_M0G3519`，直接编译即可（已确认 0 Error / 0 Warning）；
3. 通过板载 SWD 接口或 Type-C 下载。

### 要求 2 的实现要点

- 100 Hz 控制周期，基于 TimerA1 的 100 us 节拍；
- 八路灰度加权偏差 + 增量式 PID 差速循迹，弯道按偏差自动降速；
- 按键启动瞬间开始计时，检测到 A 点 5 cm 横向启停线后锁存时间并主动制动停车；
- OLED 实时显示计时，并显示八路传感器状态（`S:**--**-- N:x`）便于现场排查；
- 超过 20 s 或连续丢线 1 s 自动停车，防止冲出场地。

### 上车前需要标定的参数

全部集中在 `User/main_h2.c` 顶部宏定义区：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `H2_BASE_PWM` | 260 | 直道基础速度，决定能否在 20 s 内完成 |
| `H2_CURVE_MIN_PWM` / `H2_CURVE_SLOW_SLOPE` | 170 / 20 | 弯道降速程度，半圆丢线时继续调低 |
| `H2_FINISH_SENSOR_MIN` | 4 | 启停线判定所需同时压黑路数 |
| `H2_BRAKE_PWM` / `H2_BRAKE_TIME_MS` | 200 / 50 ms | 停车制动强度，配合指定测试点位置保证 ≤2 cm |
| `H2_START_KEY_PIN` | PB31 | 启动按键，使用底板上的独立用户按键（内部上拉，按下接 GND） |

### 注意事项

1. **不要调用 `set_motor_speed(0, 0)` 停车**：底层驱动在 `pwm<=0` 分支会得到 100% 反向占空比，停车必须用 `set_motor_speed(1, 1)`。
2. 灰度模块黑线有效电平由 `BSP/Infrared/infrared.h` 的 `INF_BLACK_LINE_LEVEL` 统一配置，极性相反时改宏，不要交换接线。
3. 引脚以 `User/config.syscfg` 和生成的 `ti_msp_dl_config.h` 为准；后续增加舵机、视觉串口等外设时不要与已占用引脚冲突。
4. 所有源码/头文件统一使用 UTF-8 编码，修改旧文件时一并整理中文注释（见 `AGENTS.md`）。
5. `Output/` 为编译产物，已在 `.gitignore` 中排除，不需要提交。

### 滚球自动稳球（Car2 工程，2026-11 新增）

`Car2/` 是最终上车工程，在循迹之外新增了静态滚球闭环：

- 舵机：PA9 / TIMA0 CCP1，50 Hz，零位 104°；
- 视觉：UART4（PB10/PB11，115200），二进制位置帧 + 遥测帧；
- 控制：PD（Kp=0.60 °/cm，Kd=0.30 °/(cm/s)），±2° 限幅、20°/s 速率限制；
- 安全：IDLE/MANUAL/AUTO/VISION_LOST/FAULT 状态机，视觉 150 ms 超时即退出闭环；
- 循迹与滚球通过 `User/main.c` 的 `CAR_APP_SELECT` 宏二选一，默认仍是循迹。

完整说明、标定表和测试步骤见 [`Car2/docs/BALL_CONTROL.md`](Car2/docs/BALL_CONTROL.md)。
主机端逻辑测试：`sh Car2/tests/host/run_tests.sh`（无硬件，gcc）。


### Jetson Nano 视觉测量

视觉代码、配置及回归测试位于 [`Jetson/`](Jetson/README.md)。当前新机位标定和漏检/误锁修复仍在进行，尚未与 `Car2/` 二进制串口协议对齐，不能直接用于滚球闭环。详见该目录当前状态说明。
