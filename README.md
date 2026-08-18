# my_arm_control — 手搓机械臂控制（D1 协议层 + D2 运动控制层 + D4 视觉抓取）

SO-ARM101（幻尔主从双臂，HX 舵机，兼容 Feetech STS3215）的**完全手搓**控制层。

> GitHub: https://github.com/On4offer/my_arm_control（D1/D2 真机验证通过）

> 定位（见 `../learning_roadmap.md` 阶段 B / Demo D1-D2、阶段 C / D4）：
> 不依赖 LeRobot / scservo_sdk，仅用 pyserial 从零实现 Feetech 串口协议与运动控制，
> 对齐 JD 能力：**工业通信协议（串口）、电机/编码器调试、运动控制、相机视觉、手眼标定、抓取闭环**。

## 目录结构

```
my_arm_control/
├── my_arm_control/          # 源码包
│   ├── protocol.py          #   手搓 Feetech 协议层（帧构造/解析/校验/扫描）
│   ├── motion.py            #   运动控制层：梯形速度规划/缓动 + 限位限幅 + 多关节下发
│   ├── kinematics.py        #   D4：SO-101 平面 FK / 垂直抓取 IK + 角度↔码值换算
│   ├── vision.py            #   D4：相机封装（OpenCV）+ 棋盘格内参标定
│   ├── calibration.py       #   D4：手眼标定（像素↔基座 XY 平面单应，eye-to-hand）
│   ├── detect.py            #   D4：目标检测（HSV 颜色分割 + 轮廓筛选）
│   └── grasp.py             #   D4：抓取闭环状态机 + 容错（重试/抓空检测/超时/急停）
├── demos/                   # 可执行 Demo（含 sys.path 引导，可直接运行）
│   ├── d1_scan_and_move.py  #   D1：搜索总线 → 识别 ID → 1 号舵机转 30°
│   ├── d2_smooth_move.py    #   D2：多关节梯形速度规划平滑运动，记录轨迹 CSV
│   ├── d2_wave_demo.py      #   D2（录视频用）：多关节交替摆动波浪运动
│   ├── d3_servo_dashboard.py#   D3：舵机调试上位机（PySide6/Qt，实时监控+单关节控制）
│   ├── dump_joint_limits.py #   关节限位诊断：读取两臂各关节 EEPROM 限位/当前值
│   ├── d4_common.py         #   D4：公共辅助（配置/校准/运动学/相机/串口）
│   ├── d4_calibrate_intrinsics.py # D4：相机内参标定（棋盘格，可选）
│   ├── d4_calibrate_table.py      # D4：手眼标定（像素↔基座 XY，机械臂示教网格+点击）
│   ├── d4_detect_demo.py          # D4：目标检测调试（实时画面 + HSV 滑杆）
│   └── d4_grasp_demo.py           # D4：视觉抓取闭环（定位→接近→下降→夹取→抬起→校验）
├── tests/                   # 离线单测（无需硬件）
│   ├── test_protocol.py     #   10 项：帧构造/解析/校验/编解码
│   ├── test_motion.py       #   8 项：轨迹规划数学/限幅
│   ├── test_kinematics.py   #   8 项：FK/IK 往返/可达性/限位/角度换算
│   ├── test_detect.py       #   6 项：合成图像颜色分割
│   ├── test_calibration.py  #   6 项：单应求解/重投影/存取
│   └── test_grasp.py        #   4 项：抓取状态机（咬住/空抓/检测失败/统计）
├── config/
│   ├── so101_joint_limits.json # 两臂关节限位快照（离线参考）
│   ├── d4_config.json          # D4 配置（相机/端口/HSV/运动学/抓取参数）
│   ├── d4_intrinsics.json      # D4 相机内参（标定产出）
│   └── d4_table_calib.json     # D4 手眼标定单应矩阵（标定产出）
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

## D4 视觉抓取闭环（阶段 C）

**目标**：OpenCV 定位目标 → 手眼标定 → 控制抓取，含失败恢复（对齐 JD：感知-规划-抓取闭环、手眼标定、容错处理）。

**架构**（eye-to-hand 2D 视觉抓取，工业标准做法）：

```
┌─────────┐   像素      ┌──────────────────┐   基座XY   ┌───────────────┐
│ 相机     │──(u,v)──→ │ 手眼标定 单应 H   │─────────→ │ 平面 IK（夹爪竖直）│
│ (固定)   │            │ 像素→基座XY       │            │ + 梯形速度规划   │
└─────────┘            └──────────────────┘            └───────┬───────┘
      ↑                                                         │
      └──────── 目标检测（HSV 颜色分割）← 工作台画面 ←────────────┘
```

**模块**（均为手搓，参考 LeRobot `src/lerobot/cameras/` 与 `model/kinematics.py`）：
- [`kinematics.py`](my_arm_control/kinematics.py)：SO-101 平面 FK / 垂直抓取 IK（肩-肘-腕 2R + 腕部竖直约束 φ2+φ3+φ4=180°），角度↔码值换算（LeRobot DEGREES 约定），可达性/限位检查
- [`vision.py`](my_arm_control/vision.py)：相机封装（OpenCV DSHOW）+ 可选棋盘格内参标定（去畸变）
- [`calibration.py`](my_arm_control/calibration.py)：手眼标定 = 像素↔基座 XY 平面单应（RANSAC 求解 H，含重投影误差）
- [`detect.py`](my_arm_control/detect.py)：目标检测（HSV 分割 + 面积过滤 + 质心）
- [`grasp.py`](my_arm_control/grasp.py)：抓取闭环状态机 + 容错

**为什么用"平面单应"而非通用 AX=XB**：目标与末端都工作在**桌面平面**（z 固定、夹爪竖直），
此时 像素→基座 映射被 2D 单应完整刻画，且把"相机位姿+内参+工作台高度+末端偏移"一次性隐式标定。
通用 AX=XB 手眼标定（Tsai-Lenz）适合任意 6D 抓取姿态；2D 场景用单应是工业视觉的标准简化，
面试可从"为什么平面场景单应足够、何时需要 AX=XB"展开。

**标定/抓取流程**（真机在环）：

```bash
cd my_arm_control
# 0. 环境检查：确认相机与 HSV（可选内参标定提升精度）
python demos/d4_detect_demo.py                       # 调 HSV → 写入 d4_config.json
python demos/d4_calibrate_intrinsics.py --capture    # （可选）棋盘格内参标定

# 1. 手眼标定（一次性）：机械臂依次移到桌面 3x3 网格，点击指尖 → 求解 H
python demos/d4_calibrate_table.py                   # 产出 config/d4_table_calib.json

# 2. 抓取闭环（验收）
python demos/d4_grasp_demo.py --trials 5             # 试跑
python demos/d4_grasp_demo.py --trials 50            # 50 次随机摆放，统计成功率（≥80% 验收）
```

**容错处理**（grasp.py，对齐 JD"失败恢复/节拍工程化"）：
1. 串口读写重试（运动中偶发超时，重试 N 次）
2. 目标检测失败 → 重试/超时 → 记失败跳过，不阻塞后续轮次
3. **抓空检测**：夹爪闭合后读 present 反馈——关到底 = 空抓（松开重试），中途堵转 = 咬住物体
4. 运动异常（不可达/超时/过载 0x20）→ 回安全位后继续，Ctrl+C 优雅退出，`--estop` 失能力矩

**标定质量判定**（`d4_calibrate_table.py` 输出 rms_mm）：
`<8mm` 优秀 / `8~15mm` 可用（目标 ≥4cm）/ `>15mm` 检查相机松动、点击精度、运动学 offset_deg/sign 是否需要调（见 [kinematics.py](my_arm_control/kinematics.py) 头部说明）。

**离线单测**（无需硬件，全部通过）：`test_kinematics`（8 项 FK/IK 往返、可达性、限位）、
`test_detect`（6 项合成图像检测）、`test_calibration`（6 项单应求解/重投影/存取）、
`test_grasp`（4 项状态机：咬住=成功 / 空抓=失败重试 / 检测失败 / 统计）。

> 真机验证状态：代码 + 离线单测就绪（2026-08-17）；标定/抓取真机验收待用户实操
> （先 `d4_detect_demo` 调 HSV → `d4_calibrate_table` 标定 → `d4_grasp_demo` 统计成功率）。

## 关节命名与运动范围（安全参考）

**关节命名**（SO-101，厂商使用文档，从上往下）：`gripper(6) wrist_roll(5) wrist_flex(4) elbow_flex(3) shoulder_lift(2) shoulder_pan(1)`。

**限位以舵机 EEPROM 实测为准**（出厂已校准，读取工具 `demos/dump_joint_limits.py COM22 COM24`）：
2026-08-16 实测（码值，4096 码=360°），快照见 [`config/so101_joint_limits.json`](./config/so101_joint_limits.json)：

| ID | 关节 | 主臂 COM22 [Min,Max] | 从臂 COM24 [Min,Max] | 备注 |
|----|------|---------------------|----------------------|------|
| 1 | shoulder_pan | [0, 4095] | [1663, 4217] | 主臂全量程 |
| 2 | shoulder_lift | [871, 3254] | [2041, 4420] | 大臂重载，易过载 |
| 3 | elbow_flex | [467, 4067] | [0, 4095] | 从臂全量程，物理行程不足 360° 需谨慎 |
| 4 | wrist_flex | [929, 3128] | [140, 2334] | |
| 5 | wrist_roll | [304, 4115] | [0, 4095] | 从臂全量程（真连续旋转） |
| 6 | gripper | [699, 1934] | [1328, 2441] | |

> 更新方法：`conda run -n lerobot python demos/dump_joint_limits.py COM22 COM24 --json config/so101_joint_limits.json`
> （运行时以实时 EEPROM 为准；该 JSON 用于离线参考/规划复用）

**安全位置怎么选**：
- 厂商校准/上电的**初始位置 = 机械臂竖直状态**（使用文档 4.1 校准章节）
- 我们的「回安全位姿」= 各关节**量程中点**（重力最平衡、距限位最远），在限位可信的前提下是保守安全位姿
- **重要**：若某关节当前位置超出其校准限位（如主臂 ID2 曾测到 -823 < 下限 871），说明限位与实际物理状态不符——此时应先跑官方 `lerobot-calibrate` 重新校准（臂放竖直初始位），再操作。GUI 回中时会对此类关节给出预警
- 机械极限由连杆几何决定，**无法程序自动测量**（自动堵转=过载 0x20）；仅换舵机/重装时才需手动校准

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
