# my_arm_control — 手搓机械臂控制（D1 协议层 + D2 运动控制层）

SO-ARM101（幻尔主从双臂，HX 舵机，兼容 Feetech STS3215）的**完全手搓**控制层。

> GitHub: https://github.com/On4offer/my_arm_control（D1/D2 真机验证通过）

> 定位（见 `../learning_roadmap.md` 阶段 B / Demo D1-D2）：
> 不依赖 LeRobot / scservo_sdk，仅用 pyserial 从零实现 Feetech 串口协议与运动控制，
> 对齐 JD 能力：**工业通信协议（串口）、电机/编码器调试、运动控制**。

## 目录结构

```
my_arm_control/
├── my_arm_control/          # 源码包
│   ├── protocol.py          #   手搓 Feetech 协议层（帧构造/解析/校验/扫描）
│   └── motion.py            #   运动控制层：梯形速度规划/缓动 + 限位限幅 + 多关节下发
├── demos/                   # 可执行 Demo（含 sys.path 引导，可直接运行）
│   ├── d1_scan_and_move.py  #   D1：搜索总线 → 识别 ID → 1 号舵机转 30°
│   ├── d2_smooth_move.py    #   D2：多关节梯形速度规划平滑运动，记录轨迹 CSV
│   ├── d2_wave_demo.py      #   D2（录视频用）：多关节交替摆动波浪运动
│   └── d3_servo_dashboard.py#   D3：舵机调试上位机（PySide6/Qt，实时监控+单关节控制）
├── tests/                   # 离线单测（无需硬件）
│   ├── test_protocol.py     #   10 项：帧构造/解析/校验/编解码
│   └── test_motion.py       #   8 项：轨迹规划数学/限幅
├── data/trajectories/       # 运行中间结果（轨迹 CSV，gitignore）
├── pyproject.toml           # 可 pip install -e . 打包安装
└── README.md
```

## 协议要点（对照阅读 LeRobot `src/lerobot/motors/feetech/`）

| 项 | 说明 |
|----|------|
| 帧格式 | 指令 `FF FF ID LEN INST PARAMS.. CHK`；状态 `FF FF ID LEN ERR PARAMS.. CHK` |
| LENGTH | 除头部外字节数 = INST/ERR + PARAMS + CHK |
| 校验和 | `~(ID+LEN+INST/ERR+ΣPARAMS) & 0xFF` |
| 字节序 | 小端（低字节在前） |
| 通信 | 半双工 UART，1M 波特率，8N1；方向切换由 BusLinker(CH343) 硬件完成 |
| 编码 | STS 系列位置/速度类 16 位寄存器为符号-数值编码（bit15 为符号位） |
| 分辨率 | 12 位磁编码器，4096 码 = 360° |

关键寄存器（STS 控制表）：`Torque_Enable(40)` `Goal_Position(42)` `Goal_Time(44)`
`Present_Position(56)` `Min/Max_Position_Limit(9/11)` `Model_Number(3)`。

## 用法

```bash
cd my_arm_control   # 项目根目录（demos/tests 内已做 sys.path 引导，任意目录也可直接运行）

# 1. 离线单测（无需硬件）
python tests/test_protocol.py
python tests/test_motion.py
# 或 pytest tests/

# 2. 真机 Demo D1：搜索总线 + 1 号舵机转 30°
python demos/d1_scan_and_move.py --port COM3

# 3. 真机 Demo D2：多关节梯形速度规划平滑运动（相对角度，度）
python demos/d2_smooth_move.py --port COM3 --target "20,10,-20,15,10,10"
python demos/d2_smooth_move.py --port COM3 --target "20,10,-20,15,10,10" --profile linear --duration-ms 2000
python demos/d2_smooth_move.py --port COM3 --dry-run          # 只读状态

# 3b. D2 波浪运动（录视频用）：多关节交替摆动，--center mid 保证 6 关节全幅
python demos/d2_wave_demo.py --port COM3 --center mid

# 3c. D3 舵机调试上位机（PySide6/Qt）：实时状态面板 + 单关节滑块控制
python demos/d3_servo_dashboard.py                 # 打开窗口，下拉选端口连接
python demos/d3_servo_dashboard.py --port COM3     # 指定端口自动连接
python demos/d3_servo_dashboard.py --smoke         # 冒烟自检（不连硬件）

# 4. 自定义：2 号舵机反向转 45°，2 秒平滑（D1）
python demos/d1_scan_and_move.py --port COM3 --servo 2 --angle -45 --duration-ms 2000
```

> 可选：`pip install -e .` 后 `import my_arm_control` 可在任意目录使用（demos/tests 已内置路径引导，不装也可跑）。

## D2 运动控制层

**实现**（`my_arm_control/motion.py`，对照 LeRobot v0.6.2）：
- `TrapezoidalProfile`：梯形速度规划（加速-匀速-减速三段，距离不足自动退化为三角形剖面）
- `LinearProfile` / `EaseProfile`：线性插值（对照 `control_utils.follower_smooth_move_to`）/ sine 缓动
- `clamp_relative`：逐帧增量限幅（对照 `robots/utils.ensure_safe_goal_position` 的 max_relative_target）
- `ArmController`：按 fps 下发多关节 Goal_Position，目标截断到限位 ± 安全余量，含到位稳定期
- 位置闭环由舵机内部 PID（P/D/I 寄存器 21/22/23）完成

**真机要点**（2026-08-16 主臂 COM22）：
- 6 关节同步梯形规划运动成功，轨迹记录 CSV（time/t_real/goal/present）
- 过载防护：目标默认距限位 ≥150 码（≈13°）安全余量；过载报警位 0x20 需断电清除

## D3 舵机调试上位机

**技术栈**：PySide6（Qt6，工业上位机/示教器主流）；依赖 `pip install PySide6`（2026-08-16 已装）。

**功能**（`demos/d3_servo_dashboard.py`）：
- 实时状态表：6 舵机 位置(码/角度)/电压/温度/错误位/限位，QTimer 10Hz 刷新
- 单关节控制：滑块+数值框写 Goal_Position，范围锁定限位；每关节独立使能/失能
- 安全设计：控制模式门控（默认关闭）、红色急停（全部失能）

**架构（可面试讲解）**：
- 信号槽：滑块 `valueChanged` → 写舵机；按钮 `clicked` → 连接/急停
- 事件循环 + QTimer 周期轮询串口，单线程串行访问串口（无并发竞态）
- UI 状态与串口状态解耦（控制模式 + 每关节使能双门控）

> 真机验证：2026-08-16 连接主臂 COM22 轮询稳定无崩溃；`--smoke` 冒烟自检通过。

## D1 验收状态

- [x] 协议层实现 + 离线单测（10/10 PASS，2026-08-16）
- [x] Demo 脚本就绪（扫描 → 识别 → 平滑转动 → 回读验证）
- [x] 真机验证（2026-08-16）：主臂 COM22 扫出 6 舵机（ID 1-6，型号 777），1 号舵机转 30° 到位（误差 2 码 ≈ 0.18°）
- [x] 从臂 COM24 同步验证：6 舵机全部识别（出厂校准限位 [1663, 4217]）
- [ ] 视频录制（D1 验收标准）— 待用户录制

> 真机发现：HX 固件下**广播 Ping 多舵机应答会在半双工单线上碰撞损坏**（重复扫描结果随机），
> `scan()` 默认改用顺序 Ping（稳定）；广播 Ping 保留为 `scan_broadcast()` 作协议学习对照。

## 对齐 JD

工业通信协议（串口帧解析/校验）、电机总线调试（Ping/寄存器读写/扫描）、
编码器数值处理（角度↔码值、符号-数值编码）。
