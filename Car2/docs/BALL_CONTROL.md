# G3519 滚球自动稳球控制说明

本文对应 2026 电赛 H 题“车载平衡滚球运动控制系统”中 G3519 端的静态滚球闭环。
范围只包含“钢珠位置反馈 -> 小角度摆杆倾角 -> 舵机 PWM”这一条链路，
不包含小车行驶加速度前馈、Jetson 摄像头识别算法和编码器速度环。

---

## 1. 文件与分层

| 文件 | 职责 |
|---|---|
| `BSP/SERVO/servo.c` / `servo.h` | 舵机 PWM、摆杆倾角映射、机械/斜率限幅 |
| `BSP/VISION/vision_link.c` / `vision_link.h` | UART0 视觉协议、非阻塞收发、超时与链路故障统计 |
| `User/App/ball_control.c` / `ball_control.h` | 状态机、速度估计、PD 控制、安全保护 |
| `User/App/ball_test.c` / `ball_test.h` | 测试应用：手动 / 模拟 / 真实 UART 三种输入源 |
| `User/main.c` | 编译期选择循迹或滚球应用 |
| `tests/host/` | PC 端单元测试与滚球闭环仿真（gcc） |
| `tools/ball_link.py` | Jetson/PC 联调工具：发送位置、解码遥测、半实物闭环仿真 |

分层原则：

- `servo` 只认“摆杆目标倾角”，不包含 PD；
- `ball_control` 只做控制，不直接写 PWM 寄存器；
- `ball_test` 只做数据来源、按键和显示；
- 原有循迹代码 `line_following.c` 未修改。

---

## 2. 硬件资源与接线

| 功能 | 引脚 | 外设 | 说明 |
|---|---|---|---|
| 舵机信号 | PA9 | TIMA0 CCP1（PWM_0 CH1） | 复用 `ServoTest` 已验证的通道 |
| 视觉/调试 | PA10/PA11 | UART0 | 循迹：printf/VOFA；滚球：Jetson 二进制协议（Type-C/DAPLink） |
| 按键 K1/K2 | PA17 / PC5 | GPIO | 已消抖，主循环轮询 |
| LCD | SPI0 | ST7789 | 280×240，用于状态显示 |

供电要求：

- 舵机使用独立 5 V，必须与 G3519 共地；
- 连续摆动时用示波器/万用表确认 V+ 不低于 4.6 V，避免 G3519 掉电复位；
- Jetson 与 G3519 的 UART 电平必须匹配 3.3 V，5 V TTL 需要电平转换。

上电安全：

- `Servo_Init()` 会把舵机直接指令到当前零位（代码中为 104°，约 1324 ticks / 1655 us）；此时舵机内部会以自身速度
  转向零位，无法通过软件斜率限制约束。
- 因上电前请取下钢珠或扶住摆杆，等 LCD 显示 IDLE 后再放球。
- 进入 AUTO 之后的所有动作才受软件斜率限制（默认 180°/s）。

资源说明：

- 视觉链路使用 UART0（PA10/PA11），与 Type-C/DAPLink 复用。
  滚球模式在 `main.c` 中关闭 printf 文本输出，二进制协议独占 UART0；
  循迹模式仍可用 UART0 的 printf/VOFA。
- UART4（PB10/PB11）保持原 WiFi 配置不变，本功能不再使用。
- TIMA0 在原工程中已由 SysConfig 分配但未使用，本功能独占，不与电机
  （TIMG14）、蜂鸣器/RGB（TIMG12）、系统时基（TIMA1）冲突。

---

## 3. 编译与运行模式

`User/main.c` 通过宏选择应用：

```c
#define CAR_APP_LINE_FOLLOW   (0)   /* 原有循迹，默认 */
#define CAR_APP_BALL_TEST     (1)   /* 滚球测试应用 */

#ifndef CAR_APP_SELECT
#define CAR_APP_SELECT        (CAR_APP_LINE_FOLLOW)
#endif
```

运行滚球控制时二选一：

1. 把 `CAR_APP_SELECT` 改成 `CAR_APP_BALL_TEST` 后重新编译；
2. 或在 Keil 的 `C/C++ -> Define` 中加入 `CAR_APP_SELECT=1`。

默认保持循迹模式，滚球模块虽然被编译但不会初始化视觉链路/舵机，因此
原有循迹、灰度、电机和 WiFi 行为不变。

---

## 4. 舵机标定

### 4.1 零位

`ServoTest`（同仓库 `ServoTest/`）曾测得：舵机 **104°** 时摆杆水平。
当前代码零位为 **104°**（约 1324 ticks / 1655 us），是当前机械结构下的标定值；
若钢珠仍持续滚动，需要按本节重新复核：

```c
#define SERVO_NEUTRAL_ANGLE_DEG   (104.0f)
```

必须用 MANUAL 模式找到真正水平位置后写回本宏；再次拆装或配重变化也要复核。

拆装舵盘、摆杆或改变配重后必须重新标定。MANUAL 模式提供 0.5° 步进、±15° 范围：

1. 小车固定、底盘调水平，摆杆不带钢珠先回当前零位 104°；
2. 用 K1+K2 切到 MANUAL，LCD 显示 `MODE: MANUAL`、`Servo: 104.00 deg`；
3. 把钢珠放到轨道中心 O 附近，观察是否持续向某一侧滚动；
4. K1/K2 的步长由 `BALL_TEST_MANUAL_STEP_DEG` 决定（当前 0.5°），
   范围由 `BALL_TEST_MANUAL_LIMIT_DEG` 决定（当前 ±15°）；
   需要精细试探时可临时把步长改小，每次调整后等摆杆稳定再观察；
5. 接近平衡点时可在 `ball_test.h` 把 `BALL_TEST_MANUAL_STEP_DEG`
   改为 0.05 做更细的试探；
6. 找到“钢珠不持续单向滚动”的位置后，读取 LCD/串口的 `servo` 值，
   把 `SERVO_NEUTRAL_ANGLE_DEG` 改成该值并重新编译；
7. 重新上电验证：MANUAL 下不按任何键时，摆杆应停在新平衡点。

注意：

- 不能仅凭钢珠几秒不动就认定绝对水平，需要多次从不同起始位置释放；
- 凹槽摩擦会导致“看似平衡但局部有死区”，应结合摆杆实际水平角度判断；
- 标定完成后必须重新确认正/负倾角对应的钢珠滚动方向。

标定记录表（实测后填写）：

| 项目 | 结果 |
|---|---|
| 原始舵机零位 | 104° |
| 当前代码零位 | 104°（约 1324 ticks / 1655 us） |
| 摆杆实际水平状态 | 待实测 |
| 钢珠静止稳定性 | 待实测 |
| 正倾角对应的滚动方向 | 待实测 |
| 负倾角对应的滚动方向 | 待实测 |
| 摆杆 1° 对应舵机角度（ratio） | 待实测 |

### 4.2 方向与比例

```c
#define SERVO_BEAM_TO_SERVO_RATIO   (1.0f)  /* 摆杆 1° 对应舵机转多少度 */
#define SERVO_BEAM_DIR_SIGN         (1.0f)  /* +1 或 -1 */
```

标定方向：

1. MANUAL 模式用 K1 给 +0.5°～+1.0° 倾角；
2. 观察钢珠是否向 x 坐标正方向滚动；
3. 若相反，把 `SERVO_BEAM_DIR_SIGN` 改为 `-1.0f`
   （或运行时调用 `Servo_SetDirection(-1.0f)`）；
4. 用量角器/手机水平仪测摆杆实际倾角 θ_actual，
   `ratio = 舵机角度增量 / θ_actual`，写回 `SERVO_BEAM_TO_SERVO_RATIO`，
   或在初始化后调用 `Servo_SetBeamRatio(ratio)`。

示例：命令 +1.0°，舵机从 104.0° 转到 105.5°，但摆杆实际只倾斜 0.5°，
则 `ratio = 1.5 / 0.5 = 3.0`。不能默认“舵机转 2° = 摆杆倾 2°”。

### 4.3 角度与 PWM 映射表

TIMA0：80 MHz ÷ 100 = 800 kHz，周期 16000 计数 = 20 ms（50 Hz）。
本板 CC1 高电平时间 = 周期 − 比较值，因此比较值要写 `16000 − 脉宽计数`。

| 舵机角度 | 脉宽计数 | 脉宽 us | CC1 比较值 | 相对零位 104° 的摆杆倾角（比例 1:1） |
|---:|---:|---:|---:|---:|
| 0° | 400 | 500 | 15600 | −104°（机械端点，不进入） |
| 89° | 1191 | 1488 | 14809 | −15° |
| 94° | 1236 | 1545 | 14764 | −10° |
| 99° | 1280 | 1600 | 14720 | −5° |
| **104°** | **1324** | **1655** | **14676** | **0°（当前零位）** |
| 109° | 1369 | 1711 | 14631 | +5° |
| 114° | 1413 | 1766 | 14587 | +10° |
| 119° | 1458 | 1822 | 14542 | +15° |
| 124° | 1502 | 1877 | 14498 | +20°（当前安全上限） |
| 84° | 1147 | 1433 | 14853 | −20°（当前安全下限） |
| 180° | 2000 | 2500 | 14000 | +76°（机械端点，不进入） |

自动控制期间有三层限幅：

1. PD 输出：±1.0°（`BALL_CONTROL_MAX_BEAM_DEG`，首次静态联调值，确认安全后可放宽）；
2. 摆杆输入：默认 ±5.0°（`SERVO_BEAM_LIMIT_DEFAULT_DEG`）；
3. 机械角度：10°~170°，软件斜率默认 180°/s（`SERVO_MAX_SLEW_DPS`，可配置）。

模式例外：
- MANUAL：摆杆输入上限 `BALL_TEST_MANUAL_LIMIT_DEG`（当前 ±15°），
  步长 `BALL_TEST_MANUAL_STEP_DEG`（当前 0.5°）；
- ROLL：K1 启动期间临时放宽到 `BALL_TEST_ROLL_BEAM_LIMIT_DEG`（15.0°），
  停止/结束后恢复默认 5.0°；
- SERVO（历史）：±30°，不在常用切换流程；
- REMOTE：直接 PWM，使用 Servo 安全行程（默认零位 ±20°）。

PD 的 ±1° 和机械角度 10°~170° 始终不变。

---

## 5. UART 通信协议

物理层：115200，8N1，小端。

帧格式：

| 偏移 | 长度 | 内容 |
|---:|---:|---|
| 0 | 1 | `0xA5` |
| 1 | 1 | `0x5A` |
| 2 | 1 | 版本 `0x01` |
| 3 | 1 | 类型：`0x01` 位置帧，`0x02` 远程舵机命令，`0x03` 控制指令，`0x81`/`0x82`/`0x83` 遥测与回执 |
| 4 | 1 | 负载长度 len |
| 5 | 2 | 帧序号 seq |
| 7 | len | 负载 |
| 7+len | 2 | CRC16-CCITT-FALSE（初值 0xFFFF，多项式 0x1021） |

CRC 覆盖偏移 2 到 6+len（版本、类型、长度、序号、负载）。

位置帧（len=12）：

| 偏移 | 类型 | 含义 |
|---:|---|---|
| 0 | uint32 | Jetson 单调时钟时间戳 ms |
| 4 | float32 | 钢珠位置 cm |
| 8 | uint8 | 置信度 0~100 |
| 9 | uint8 | bit0=有效，bit1=已完成厘米标定 |
| 10 | uint16 | 保留，填 0 |

**未完成厘米标定时必须把 bit1 置 0**，G3519 会拒收该帧，绝不用像素坐标
冒充厘米位置。

远程舵机命令（0x02，len=2，Nano → G3519）：

| 偏移 | 类型 | 含义 |
|---:|---|---|
| 0 | uint16 | 目标 PWM 高电平计数 target_pwm_ticks |
| — | — | 400=500us，2000=2500us，当前零位约 1324 ticks |

帧头 seq 作为命令序号；G3519 先按 REMOTE 安全行程（默认零位 ±20°）限幅，
再做 10°~170° 机械限幅和 180°/s 软件斜率限制。
远程命令超过 300 ms 未更新时退出远程控制并回安全零位。

远程回执（0x82，len=12，G3519 → Nano）：

| 偏移 | 类型 | 含义 |
|---:|---|---|
| 0 | uint16 | command_seq（最近处理的 0x02 序号） |
| 2 | uint32 | board_time_ms（G3519 本地毫秒） |
| 6 | uint16 | target_pwm_ticks（限幅后的目标） |
| 8 | uint16 | output_pwm_ticks（当前实际写入的 PWM 计数） |
| 10 | uint8 | remote_enabled（0/1） |
| 11 | uint8 | result（0=OK，1=STOP，2=TIMEOUT，3=NO_CMD，4=CLAMP） |

控制指令（0x03，len=10，Nano → G3519）：

| 偏移 | 类型 | 含义 |
|---:|---|---|
| 0 | uint8 | opcode：1=SET_MODE，2=START_AUTO，3=STOP，4=SET_TARGET，5=SET_PD，6=START_REMOTE，7=GET_PARAMS |
| 1 | uint8 | mode：0=MANUAL，2=UART，5=REMOTE（SET_MODE 使用） |
| 2 | float32 | arg0：SET_TARGET=目标 cm；SET_PD=Kp |
| 6 | float32 | arg1：SET_PD=Kd |

控制回执（0x83，len=6，G3519 → Nano）：

| 偏移 | 类型 | 含义 |
|---:|---|---|
| 0 | uint16 | command_seq（对应 0x03 帧头序号） |
| 2 | uint8 | opcode |
| 3 | uint8 | result：0=OK，1=BAD_MODE，2=VISION_INVALID，3=PARAM_RANGE，4=FAULT，5=REJECTED |
| 4 | uint8 | mode（当前生效模式） |
| 5 | uint8 | control_state（BallControl 状态） |

执行机构参数（0x84，len=12，G3519 → Nano）：

| 偏移 | 类型 | 含义 |
|---:|---|---|
| 0 | uint16 | neutral_ticks（当前软件零位） |
| 2 | uint16 | min_ticks（实际安全行程下限） |
| 4 | uint16 | max_ticks（实际安全行程上限） |
| 6 | uint16 | slew_rate_x10（软件速率 °/s × 10） |
| 8 | uint32 | 保留，填 0 |

Nano 发 0x03 GET_PARAMS（opcode=7）即可查询；START_REMOTE 成功后也会主动发送一次。

遥测帧（len=32）：见 `BSP/VISION/vision_link.h` 顶部注释，字段包括
时间、目标、位置、速度、误差、倾角指令、脉宽、状态、标志和估算倾角。
注意：`beam_actual_deg` 是舵机指令经比例反算的软件估算值，不是摆杆角度
传感器的实测值，只能用于趋势观察和半实物仿真。

安全策略：

- 视觉有效期 150 ms，从最近一次**有效**帧算起；无效帧不会刷新该计时；
- 缺测 80 ms（`BALL_CONTROL_COMMAND_HOLD_MS`）内保持上一次 PD 指令，
  超过 80 ms 先限速回零等待，150 ms 仍无有效帧才退出 AUTO；
- 连续 20 次解码错误置 `link_fault`，控制器进入 FAULT；
- 发送走 512 字节软件环形缓冲 + TX 中断，主循环不会阻塞；
- 接收走 256 字节环形缓冲，中断只搬字节，解析在主循环；
- LCD 采用逐行增量刷新（8 MHz SPI 下一行约 8 ms），不整屏重绘，
  避免一次阻塞约 60 ms 破坏 30 Hz 节拍。

---

## 6. PD 控制与速度估计

控制律：

```
θ = Kp · (target − x) − Kd · v
```

默认参数：

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| Kp | 0.60 | °/cm | 2 cm 误差 -> 1.2° |
| Kd | 0.30 | °/(cm/s) | 阻尼，抑制超调 |
| 最大倾角 | 1.0 | ° | PD 输出硬限幅（首次静态联调） |
| 速度低通系数 | 0.60 | — | 越大越平滑、相位滞后越大 |
| 位置跳变阈值 | 3.0 | cm/帧 | 超过整帧剔除 |
| 速度上限 | 200 | cm/s | 超过判为异常 |
| 控制周期 | 33 ms | ms | 每次收到新有效帧计算一次 |
| 指令保持 | 80 | ms | 缺测超过 80 ms 先限速回零等待 |
| 视觉超时 | 150 | ms | 从最近一次有效帧算起，超时退出 AUTO |

速度估计要点：

- `v = (x_k − x_{k−1}) / Δt`，Δt 使用 **G3519 本地接收时刻**；
- Jetson 的 `frame_time_ms` 只用于排序/诊断，绝不与本地时钟混算；
- 帧间隔 <10 ms 或 >200 ms 时不计算速度（置 0）；
- 速度经过一阶低通后再进入 PD，避免舵机抖动；
- 失效帧、重复帧、跳变帧都不会产生速度测量。

调试输出（遥测帧）字段与题目要求对应：

```
time_ms, target_cm, ball_x_cm, ball_v_cm_s, position_error_cm,
beam_angle_cmd_deg, servo_pwm_us, control_state, vision_valid
```

---

## 7. 状态机与安全

| 状态 | 触发 | 舵机行为 | 退出条件 |
|---|---|---|---|
| IDLE | 上电/关闭自动 | 限速回中 | 0x03 START_AUTO 且视觉新鲜 |
| MANUAL | 手动测试 | 执行手动倾角（限速） | 切换模式 |
| AUTO | 显式使能 + 视觉有效 | 新有效帧算 PD；缺测 80 ms 后先回零等待 | 明确无效帧/超时/故障/关闭 |
| VISION_LOST | 明确无效帧或 >150 ms 无有效帧 | 限速回中，不用旧数据 | 必须重新使能 |
| FAULT | 链路连续错误/舵机异常 | 限速回中，禁止 PD | 显式 `ClearFault` |

关键约束：

- 上电默认 IDLE，不会自动进入 AUTO；
- 视觉恢复后 **不会自动** 回到 AUTO，必须按键重新使能；
- FAULT 只能通过 `BallControl_ClearFault()`（测试应用中为模式切换）解除；
- 所有回中动作都经过软件斜率限制（默认 180°/s），不会突然大幅摆动。

---

## 8. 测试方法与结果

### 8.0 正式模式、按键与 LCD

常用模式循环：

```text
MANUAL -> REMOTE -> UART -> MANUAL
```

SIM、ROLL、SERVO 源码保留，但暂时移出常用切换流程。

| 模式 | K1 | K2 |
|---|---|---|
| MANUAL | +0.5° | −0.5°（范围 ±15°） |
| REMOTE | 本地停止远程控制 | 暂不分配 |
| UART | 本地停止 PD | 暂不分配 |

正常启动由 Nano 发 0x03 完成：`SET_MODE` → `START_AUTO` / `START_REMOTE`。
`K1+K2` 仅用于本地模式切换；REMOTE 与 UART 自动控制互斥。

REMOTE 安全限制：

- Nano 发 START_REMOTE 即进入远程并先保持零位；
  之后若 300 ms 内没有新的 0x02，自动退出远程并回零；K1 只做本地停止；
- 目标 PWM 使用 SERVO 安全行程（默认零位 ±20°，`SERVO_SAFE_NEG_DEG`/
  `SERVO_SAFE_POS_DEG`，允许不对称），不再固定 ±5°；
- 0x02 超过 300 ms 未更新自动退出远程并回零，不会自动恢复；
- 软件斜率默认 180°/s（`SERVO_MAX_SLEW_DPS`），保留 10°~170° 机械限幅。

**MANUAL LCD**

```text
MODE: MANUAL
Servo: 104.00 deg
PWM:    1655 us
Ticks:  1324
Offset:+0.00 deg
K1:+0.5 K2:-0.5
```

**REMOTE LCD**

```text
MODE: REMOTE
State: RUN / STOP
Target: 1368 ticks
Output: 1368 ticks
PWM:    1711 us
Range: 1147~1502
Seq:01234 R:OK
```

Target/Output 都直接显示 PWM 计数；软件 PWM 输出不等于舵机真实机械角度。

**UART LCD**

```text
MODE: UART AUTO
Target: 0.00 cm
X:+0.87 V:+0.00
Beam:-0.52 deg
PWM:1716 us Vis:1
Kp:0.60 Kd:0.30
Stop:NONE Cmd:OK
```

`Stop:` 为 AUTO 退出原因：NONE=未退出，STOP=人工/指令停止，
INV=无效帧/异常数据，TMO=150 ms 超时，FLT=故障。
`Cmd:` 为最近一条 0x03 的结果（OK/BADM/VISN/RNG/FLT/REJ）。



### 8.1 平衡点标定（测试 A）

操作步骤见第 4.1 节。验证要点：

1. 切换到 MANUAL（K1+K2 循环到 MANUAL）；
2. LCD 确认 `MODE: MANUAL`、`Servo: 104.00 deg`、`Offset:+0.00 deg`；
3. 钢珠放 O 点附近，多次从不同起始位置释放，不得持续单向滚动；
4. K1/K2 按 `BALL_TEST_MANUAL_STEP_DEG` 调整（当前 0.5°，范围 ±15°），
   观察增量是否与 `SERVO_MAX_SLEW_DPS` 和机械结构相符；
5. 记录新零位并写回 `SERVO_NEUTRAL_ANGLE_DEG`，重新上电复验。

### 8.1b 自由舵机角度测试（SERVO）

用途：不依赖视觉，直接检查舵机和摆杆在较大角度范围内的动作与限位。

- SERVO 已移出常用模式循环；如需使用，可临时修改 `BALL_TEST_DEFAULT_MODE`
  或调用 `BallTest_SetMode(BALL_TEST_MODE_SERVO)`；
- K1：每次 **+3°**；K2：每次 **-3°**；
- 相对零位范围 **-30° ~ +30°**，进入模式时临时把摆杆软限幅放宽到 30°；
  当前 `SERVO_BEAM_TO_SERVO_RATIO=1.0`，`SrvAng` 等同于舵机角度偏移；
- LCD 第 1 行显示绝对舵机角和估算摆杆角，第 4 行显示 `SrvAng`，
  第 5 行显示 `Range:-30/+30 Step:3`；
- 退出 SERVO 后自动恢复默认 5° 软限幅；
- 舵机机械限幅仍是 10°~170°，不要长时间顶在端部。

### 8.1c REMOTE 执行机构标定

Nano 或 PC 工具先 SET_MODE(REMOTE)，再发 START_REMOTE；之后以 10 Hz 发送 0x02：

```bash
# 切到 REMOTE 模式
python tools/ball_link.py control --port COMx --opcode 1 --mode 5
# 允许远程控制（无需先发 0x02）
python tools/ball_link.py control --port COMx --opcode 6 --mode 5
# 查询执行机构参数（neutral/min/max/slew）
python tools/ball_link.py control --port COMx --opcode 7 --mode 5
# 发送远程 PWM 目标
python tools/ball_link.py remote --port COMx --ticks 1324
python tools/ball_link.py remote --port COMx --ticks 1368
```

- 目标限制在 `Servo_GetSafeMinTicks()`~`Servo_GetSafeMaxTicks()`；
  默认零位 ±20°，需先空载验证实际机械/供电阻尼；
- 用不同 ticks 记录“指令值 → 实际舵机/摆杆角度”，确定：
  - 真实零位（钢珠在 O 点不持续滚动）；
  - 增加 ticks 时摆杆的实际倾斜方向；
  - 最小可重复产生运动的 PWM 步长和机械死区；
  - 传动比例（摆杆角度 / 舵机角度），再写回 PD 配置。
- REMOTE 直接按 PWM 计数驱动舵机，不经过 `SERVO_BEAM_DIR_SIGN` 和
  `SERVO_BEAM_TO_SERVO_RATIO`；PD 的控制方向仍需通过 UART/MANUAL 实验单独确认。
- 0x82 回执中的 target/output 是软件值，不是轨道角度实测值；实际角度必须
  用量角器或钢珠运动验证。

### 8.2 开环 5°/7.5°/10° 自动左右滚动（新增）

#### 运行流程

```text
K1 启动
  -> 回平衡点 0°（等实际到位 + 保持 400 ms）
  -> +θ 保持 POSITIVE_HOLD_MS（默认 500 ms）
  -> 回平衡点 0°（等实际到位 + 保持 400 ms）
  -> -θ 保持 NEGATIVE_HOLD_MS（默认 200 ms）
  -> 回平衡点 0°（等实际到位 + 保持 400 ms）
  -> 再次 +θ ...（循环）
K1 再按一次 -> 立即停止，恢复默认限幅并限速回平衡点
```

- K1 启动后会一直循环，直到再次按 K1；
- 到位超时（3 s）会中止循环并显示 `ABRT`；
- 切换模式也会停止循环。

- θ ∈ {5°, 7.5°, 10°}，通过 K2 选择；
- 循环执行，每次完成 `BACK-` 后直接进入下一轮 `POS`，不再额外插入停顿；
- 每个阶段都先等 `Servo_GetBeamAngleActual()` 进入 ±0.2° 到达窗口，
  再开始保持计时，因此 0→10° 约 0.5 s 的斜率时间不会被算进保持时间；
- 3 s 仍未到位则判定卡滞/异常，显示 `ABRT` 并回平衡点；
- 当前档位 5°/7.5°/10° 比原来的 2°/3°/4°/5° 大，正/负保持时间
  当前配置为 500 ms / 200 ms（原 800 ms），避免钢珠直接撞到轨道端部。

#### 按键

| 场景 | K1 | K2 | K1+K2 |
|---|---|---|---|
| ROLL 未运行 | 启动连续循环滚动 | 切换 5/7.5/10° 档位 | 切换到下一模式 |
| ROLL 运行中 | 立即停止并回平衡点 | 忽略，防止中途变档 | 停止并切换模式 |

#### 可调参数（`ball_test.h`）

| 宏 | 默认 | 说明 |
|---|---:|---|
| `BALL_TEST_ROLL_POSITIVE_HOLD_MS` | 500 | 正倾角保持时间 |
| `BALL_TEST_ROLL_NEGATIVE_HOLD_MS` | 200 | 反倾角保持时间 |
| `BALL_TEST_ROLL_CENTER_HOLD_MS` | 400 | 每次回中后的停留时间 |
| `BALL_TEST_ROLL_BEAM_LIMIT_DEG` | 15.0 | ROLL 运行期间的倾角上限 |
| `BALL_TEST_ROLL_SETTLE_TOL_DEG` | 0.2 | 到达判定窗口 |
| `BALL_TEST_ROLL_SETTLE_TIMEOUT_MS` | 3000 | 到位超时，超时中止 |

#### LCD

- 第 0 行：`M:ROLL S:RUN/STOP/ABRT T:+5.00`；
- 第 1 行：`Servo:104.00 Act:+5.00`（绝对舵机角 / 估算摆杆角）；
- 第 3 行：`Stage:POS/BACK+/NEG/BACK- Run:1 Ab:0`；
- 第 4 行：`Lvl:1/3 +5.00`；
- 第 5 行：正/负保持时间。

#### 测试记录（实测后填写）

| 档位 | +θ 钢珠运动 | -θ 钢珠运动 | 保持时间 | 现象/备注 |
|---:|---|---|---:|---|
| 5° | 待实测 | 待实测 | +500 / -200 ms | 行程余量较大 |
| 7.5° | 待实测 | 待实测 | +500 / -200 ms | 先短时间试跑 |
| 10° | 待实测 | 待实测 | +500 / -200 ms | **必须先空载验证行程**，再放钢珠 |

#### 测试 F：安全退出

1. 选 10°，K1 启动，在 +θ 保持阶段再按 K1；
2. 预期：LCD 立即显示 `STOP`，后续 `BACK+/NEG` 不再执行；
3. 摆杆以软件斜率限速回平衡点，不出现反向大角度动作；
4. 再次按 K1 可重新执行一遍完整流程。

### 8.3 模拟视觉

1. 切换到 SIM；
2. K1 使能 AUTO；
3. K2 依次切换场景：`X=-2`、`X=+2`、`CENTER`、`TGT+5`、`INVALID`、`DYNAMIC`；
4. 观察 LCD 的倾角符号和 INVALID 场景是否立即退出 AUTO；
5. DYNAMIC 场景用滚球动力学模型做闭环，可长期观察是否有界。

### 8.4 首次静态闭环联调（真实钢珠）

当前配置：

```c
CAR_APP_SELECT=1            // main.c 编译期选择滚球应用
0x03 START_AUTO            // Nano 显式启动 PD（无需本地 K1）
BALL_TEST_DEFAULT_MODE=UART
BALL_CONTROL_MAX_BEAM_DEG=1.0
Kp=0.60  Kd=0.30            // 初始值，尚未经真实闭环验证
```

操作顺序：

1. 固定小车，车轮停止，轨道两端放挡球保护；
2. Jetson 完成厘米标定，通过 UART0/Type-C 发送 0x01 位置帧；
3. G3519 上电进入 UART 模式，确认 LCD `Vis=1`、`X` 随钢珠移动；
4. **首次固定目标 0 cm**，不要按 K2 切换目标；
5. 钢珠放在 O 点附近 ±0.5～1 cm；
6. Nano 发 0x03 START_AUTO；观察舵机是否让钢珠向 O 点回归；
7. 分别在 −1 cm、+1 cm 附近释放，确认两个方向都能回中；
8. 若出现持续振荡或超调，先降 Kp，再逐步加 Kd；
9. 出现方向错误、接近轨道端部或 `Vis:0` 时，立即按 K1 停止。

保护保持原样：视觉 150 ms 超时退出 AUTO、无效帧不控制；退出后必须由 Nano 重新发 START_AUTO
使能。当前 PD 限幅 ±1°，舵机软件斜率默认 180°/s；确认机械安全后再调整。

记录 0x81 遥测中的 `target_cm`、`ball_x_cm`、`ball_v_cm_s`、
`position_error_cm`、`beam_cmd_deg`、`servo_pwm_us`、`control_state`、
`vision_valid`，用于判断回中、振荡、超调和舵机饱和。

### 8.5 指定位置保持

K2 在 0 / +5 / −5 cm 之间切换，目标在 5 s 内到位且误差 ≤1 cm。

### 8.6 当前已完成的软件测试

主机端测试（`tests/host/`，gcc，无硬件）：

| 测试程序 | 结果 | 覆盖内容 |
|---|---|---|
| `ball_test_host.exe` | 45 checks, 0 failures | 角度/脉宽映射、斜率限制、机械限幅、PD 符号、当前 PD 限幅、80 ms 指令保持/回零、150 ms 超时、失效帧退出、跳变剔除、速度滤波、重复帧、手动保持、闭环仿真 |
| `vision_link_host.exe` | 50 checks, 0 failures | 0x01/0x02/0x03 帧解析、CRC、字段、重复序号、0x02 超龄、0x03 参数、遥测组包 |
| `roll_mode_host.exe` | 93 checks, 0 failures | 历史模式、MANUAL 0.5°/±15°、REMOTE 安全行程、SET_MODE/START_AUTO/STOP/SET_TARGET/SET_PD/GET_PARAMS |

一键运行：

```bash
sh Car2/tests/host/run_tests.sh
```

滚球动力学仿真结果（Kp=0.60，Kd=0.30，理想模型，无噪声）：

| 场景 | 结果 |
|---|---|
| 5 cm → 0 cm | 1.287 s 进入 ±1 cm，最大偏移 5.00 cm |
| 0 → +5 cm | 1.287 s 进入 ±1 cm，超调约 0.07 cm |
| +5 → −5 cm | 1.716 s 进入 ±1 cm，超调约 0.26 cm |

完整日志：

- `tests/host/results/host_test_output.txt`
- `tests/host/results/vision_link_test_output.txt`
- `tests/host/results/roll_mode_test_output.txt`
- `tests/host/results/closed_loop_step_response.csv`

**注意**：以上是模型仿真结果，不是实车数据；真实钢珠受摩擦、舵机死区、
视觉延迟和机械间隙影响，必须按 8.4/8.5 节重新调参。开环三档的软件验证
（93 项）只覆盖状态机、限幅切换、0x03 控制逻辑和时序，不能替代实车标定。

---

## 9. 联调工具

```bash
# 发送测试 B 的静态场景（每 2 s 切换一次）
python tools/ball_link.py send --port COM7 --mode scenario

# 发送固定位置
python tools/ball_link.py send --port COM7 --mode fixed --x 2

# 解码遥测并保存 CSV
python tools/ball_link.py decode --port COM7 --csv telemetry.csv

# 半实物闭环：PC 用遥测里的实际倾角积分钢珠模型，再发回位置
python tools/ball_link.py sim --port COM7 --start-x 5 --csv sim_loop.csv
```

半实物仿真时 G3519 完全不知道数据来源，因此可以验证
“UART -> 解析 -> 速度估计 -> PD -> 舵机映射”整条链路，
在没有钢珠和摄像头时也能做初步闭环检查。

---

## 10. 调参记录

| 日期 | Kp | Kd | 目标/场景 | 现象 | 结论 |
|---|---:|---:|---|---|---|
| 2026-11-12 | 0.60 | 0.30 | 仿真 0/+5/−5 cm | 1.2~1.5 s 到位，超调 <0.2 cm | 基准参数，待实车验证 |

实车调参建议顺序：

1. Kd=0，从 Kp=0.2 开始，每步 +0.1，直到钢珠能回到目标但略有振荡；
2. 逐步增加 Kd（0.05 起步），观察振荡是否衰减；
3. 若舵机持续抖动，先降低 Kd 或增大速度滤波系数；
4. 若响应太慢，再小幅增加 Kp，同时保证最大倾角不超过当前安全限幅。

---

## 11. 已知问题与下一步

1. **实车标定仍需复核**：代码当前零位为 104°（约 1324 ticks / 1655 us），
   方向 `SERVO_BEAM_DIR_SIGN`、摆杆/舵机比例 `SERVO_BEAM_TO_SERVO_RATIO`
   仍需按第 4 节复核；文档中的标定表和测试记录要按实车填写。
2. **三档滚动只完成软件验证**：5/7.5/10° 的实际钢珠运动、保持时间、
   机械振动和最大档位行程必须按 8.2 节逐档做实车测试并填写记录。
3. **UART0 复用**：滚球模式使用 UART0 跑二进制协议，必须保持 printf 关闭；
   Jetson 通过 Type-C/DAPLink 的 COM 口连接，注意与循迹 VOFA 模式不要同时使用。
4. **舵机供电未做负载测试**：连续摆动时应测量 V+ 压降，必要时加大电容或
   改用独立 BEC；10°/15° 档连续往返时尤其要确认 G3519 不复位。
5. **未做小车运动前馈**：本版本仅静态稳球；小车行驶时需加入加速度前馈
   和 IMU 倾角补偿。
6. **视觉延迟未知**：目前按 30 FPS、150 ms 超时设计，Jetson 实测端到端
   延迟后应重新评估 Kd 和超时阈值。
7. **无编码器/里程计融合**：题目后续若要求运动中的位置保持，需要引入
   小车速度前馈。
8. **演示视频未录制**：实物自动左右滚动的视频需在完成 8.2 节测试后录制。
