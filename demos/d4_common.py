# -*- coding: utf-8 -*-
"""D4 Demos 公共辅助：加载配置/校准/运动学/相机/串口，避免重复代码。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

# 把项目根目录加入 sys.path（demos 目录直接运行时可导入 my_arm_control）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from my_arm_control.calibration import TableCalibration  # noqa: E402
from my_arm_control.detect import ColorTargetDetector  # noqa: E402
from my_arm_control.kinematics import SO101Kinematics, PathUnsafeError, load_calibration  # noqa: E402
from my_arm_control.motion import ArmController  # noqa: E402
from my_arm_control.protocol import DEFAULT_BAUDRATE, FeetechSerialBus  # noqa: E402
from my_arm_control.vision import CameraView, pick_camera  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
D4_CONFIG = PROJECT_ROOT / "config" / "d4_config.json"
DEFAULT_TABLE_CALIB = PROJECT_ROOT / "config" / "d4_table_calib.json"
DEFAULT_INTRINSICS = PROJECT_ROOT / "config" / "d4_intrinsics.json"


def load_d4_config(path: str | Path | None = None) -> dict:
    with open(path or D4_CONFIG, "r", encoding="utf-8") as f:
        return json.load(f)


def make_kinematics(config: dict) -> SO101Kinematics:
    cal = load_calibration(config["hardware"]["calibration_json"])
    return SO101Kinematics(config, cal)


def make_camera(config: dict, use_intrinsics: bool = True, camera_index: int | None = None) -> CameraView:
    """创建相机：优先配置/指定 index，不可用则自动 fallback 到分辨率匹配的相机。"""
    idx = pick_camera(camera_index if camera_index is not None else int(config["hardware"]["camera_index"]))
    cam = CameraView(idx)
    if use_intrinsics and DEFAULT_INTRINSICS.exists():
        cam.load_intrinsics(DEFAULT_INTRINSICS)
    return cam


def make_detector(config: dict) -> ColorTargetDetector:
    t = config["target_detect"]
    return ColorTargetDetector(
        hsv=t["hsv"],
        area_range=tuple(t["area_range"]),
        max_targets=int(t.get("max_targets", 1)),
    )


def make_bus(config: dict) -> FeetechSerialBus:
    return FeetechSerialBus(port=config["hardware"]["port"], baudrate=DEFAULT_BAUDRATE)


GRIPPER_SERVO_ID = 6  # 夹爪舵机：用 open/close 码单独控制，不参与手臂移动


def make_controller(config: dict, bus: FeetechSerialBus) -> ArmController:
    return ArmController(bus, servo_ids=config["hardware"]["servo_ids"])


def _arm_controller(controller: ArmController) -> ArmController:
    """返回只含手臂关节（不含夹爪）的控制器。

    夹爪开到机械挡块时会堵转 → 过载报警 0x20（锁存，该舵机一切读写失败）。
    手臂移动只依赖 1~5 号关节，夹爪由 open/close 码单独控制；若手臂移动路径
    也去读/写夹爪，夹爪一报警整个 move 就会在 20s 重试里卡死（a/d 微调无响应）。
    """
    return ArmController(controller.bus, servo_ids=[sid for sid in controller.servo_ids if sid != GRIPPER_SERVO_ID])


def move_joints_deg_held(controller: ArmController, kin: SO101Kinematics, joints_deg: dict,
                         v_max: float = 150.0, a_max: float = 300.0, settle_s: float = 0.8) -> None:
    """移动指定关节到角度，未指定（wrist_roll 等）保持当前码值；夹爪不参与。

    joints_deg: {关节名: 角度}（shoulder_pan/shoulder_lift/elbow_flex/wrist_flex/wrist_roll）。
    只读写 1~5 号手臂关节（夹爪由 open/close 码单独控制，见 GRIPPER_SERVO_ID），
    因此夹爪堵转/过载报警 0x20 不会卡住手臂移动。
    运动中偶发串口读取超时（阶段 B 已知）→ 自动重试，避免标定中断。
    移动前做【路径级防撞】：机械臂每次起点姿态可能不同，插值路径任何
    一点指尖 z 低于 z_safe_min 都拒绝执行（抛 PathUnsafeError），防止撞桌。
    """
    import time

    from my_arm_control.protocol import ProtocolError  # noqa: PLC0415

    ctrl = _arm_controller(controller)
    _NAME = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll"}
    deadline = time.perf_counter() + 20.0
    while True:
        try:
            present = ctrl.read_positions()
            targets = {sid: float(present[sid]) for sid in ctrl.servo_ids}  # 默认保持当前
            for sid, name in _NAME.items():
                if name in joints_deg:
                    targets[sid] = float(kin.deg_to_code(name, joints_deg[name]))
            # 路径级防撞：从任意当前位姿出发，整条插值路径不得低于安全下限
            cur_deg = {n: kin.code_to_deg(n, present[s]) for s, n in _NAME.items()}
            tgt_deg = {n: kin.code_to_deg(n, targets[s]) for s, n in _NAME.items()}
            kin.check_path_safe(cur_deg, tgt_deg, kin.safe_z_min)
            ctrl.move_to(targets, v_max=v_max, a_max=a_max, settle_s=settle_s)
            return
        except ProtocolError:
            if time.perf_counter() > deadline:
                raise
            time.sleep(0.05)


def safe_move_xyz(controller: ArmController, kin: SO101Kinematics,
                  x: float, y: float, z: float,
                  v_max: float = 120.0, a_max: float = 240.0, settle_s: float = 0.3) -> None:
    """【安全移动】到指尖位置 (x, y, z)：先垂直抬升 → 高位水平移动 → 下降。

    避免从低处直接 move 到目标时，关节插值在低高度横穿（撞桌面/目标物）。
    用于探高/标定/回位等所有"跨工作区移动"。
    """
    from my_arm_control.kinematics import KinematicsError  # noqa: PLC0415

    present = _arm_controller(controller).read_positions()  # 只读手臂关节，夹爪过载 0x20 不影响
    cur_deg = {
        "shoulder_pan": kin.code_to_deg("shoulder_pan", present[1]),
        "shoulder_lift": kin.code_to_deg("shoulder_lift", present[2]),
        "elbow_flex": kin.code_to_deg("elbow_flex", present[3]),
        "wrist_flex": kin.code_to_deg("wrist_flex", present[4]),
    }
    cur_xyz = kin.fk({**cur_deg, "wrist_roll": 0.0, "gripper": 0.0})
    z_high = min(max(cur_xyz[2], z) + 0.05, 0.20)  # 抬升到高于当前和目标（上限 0.20）

    # 1) 垂直抬升（保持当前 XY）
    try:
        lift = kin.ik_vertical(cur_xyz[0], cur_xyz[1], z_high)
        move_joints_deg_held(controller, kin, lift, v_max, a_max, settle_s)
    except PathUnsafeError:
        raise  # 路径撞桌是安全问题：不兜底，必须停下让用户手动处理
    except KinematicsError:
        pass  # 当前点不可达抬升位，直接尝试水平（兜底）
    # 2) 高位水平移动到目标上方
    try:
        move = kin.ik_vertical(x, y, z_high)
        move_joints_deg_held(controller, kin, move, v_max, a_max, settle_s)
    except PathUnsafeError:
        raise
    except KinematicsError:
        pass  # 目标高位不可达，兜底直接下降（有撞风险时会在下降前报错）
    # 3) 下降到目标 z
    down = kin.ik_vertical(x, y, z)
    move_joints_deg_held(controller, kin, down, v_max, a_max, settle_s)


def load_table_calib(path: str | Path | None = None) -> TableCalibration | None:
    p = Path(path or DEFAULT_TABLE_CALIB)
    if p.exists():
        return TableCalibration.load(p)
    return None
