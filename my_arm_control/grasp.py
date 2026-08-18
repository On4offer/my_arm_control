# -*- coding: utf-8 -*-
"""
D4 抓取闭环：状态机 + 容错处理。

流程（每轮）：
    HOME → DETECT（重试/超时）→ APPROACH（目标正上方 z_pre）
         → DESCEND（到 z_grasp）→ CLOSE（夹爪）→ LIFT（z_lift）
         → VERIFY（夹爪到位反馈判断抓空）→ HOME

容错（对齐 JD"失败恢复/节拍工程化"）：
    1. 串口读写重试（运动中偶发超时）
    2. 目标检测失败 → 重试 N 次 / 超时 → 记失败跳过
    3. 抓空检测：夹爪 present 反馈（关到底 = 空抓，中途堵转 = 抓到物体）
    4. 单轮异常 → 安全回位（HOME）后继续，避免卡死流程
    5. Ctrl+C 优雅退出（回 HOME 保持力矩，可选 --estop 断电）
"""

from __future__ import annotations

import math
import signal
import sys
import time
from typing import Any, Callable

import cv2
import numpy as np

from .calibration import TableCalibration
from .detect import ColorTargetDetector, Target
from .kinematics import KinematicsError, SO101Kinematics
from .motion import ArmController
from .protocol import ADDR, ERRBIT_OVERLOAD, FeetechSerialBus, ProtocolError, decode_sign_magnitude


class GraspError(Exception):
    """抓取流程错误（不可恢复，需人工干预）。"""


def _retry(fn: Callable, retries: int, what: str = "串口操作") -> Any:
    """带重试的串口调用：偶发 ProtocolError（读超时）重试，重试耗尽上抛。"""
    last_err: Exception | None = None
    for i in range(retries + 1):
        try:
            return fn()
        except ProtocolError as e:
            last_err = e
            if i < retries:
                time.sleep(0.05)
    raise GraspError(f"{what} 重试 {retries} 次仍失败: {last_err}")


class GraspController:
    """视觉抓取闭环控制器（eye-to-hand 2D 定位 + 平面 IK + 容错状态机）。"""

    def __init__(
        self,
        config: dict[str, Any],
        kinematics: SO101Kinematics,
        calib: TableCalibration,
        detector: ColorTargetDetector,
        bus: FeetechSerialBus,
        controller: ArmController,
    ):
        self.cfg = config
        self.kin = kinematics
        self.calib = calib
        self.detector = detector
        self.bus = bus
        # 手臂移动只用 1~5 号关节；夹爪(6)由 set_gripper 单独控制。
        # 夹爪张到最大顶机械挡块会触发过载报警 0x20（锁存，读写全失败），
        # 若移动路径也去读/写 6 号，夹爪一报警整个 move 就卡 20s（D4 实测）。
        self._estop_ids = list(controller.servo_ids)
        if isinstance(controller, ArmController):
            ids = [sid for sid in controller.servo_ids if sid != 6]
            self.arm = ArmController(controller.bus, servo_ids=ids)
        else:  # 测试注入的 mock 控制器
            self.arm = controller

        t = config["table"]
        self.z_pre = float(t["z_pre"])
        self.z_grasp = float(t["z_grasp"])
        self.z_lift = float(t["z_lift"])
        self.max_reach = float(t["max_reach"])

        m = config["motion"]
        self.v_max = float(m["v_max"])
        self.a_max = float(m["a_max"])
        self.grasp_v = float(m["grasp_v_max"])
        self.grasp_a = float(m["grasp_a_max"])
        self.move_timeout = float(m["move_timeout_s"])
        self.serial_retries = int(m["serial_retries"])

        g = config["gripper"]
        self.gripper_open_code = g["open"]  # 0-100 归一值
        self.gripper_close_code = g["close"]
        self.verify_delta = int(g["verify_delta_codes"])
        # gripper 用 RANGE_0_100 语义：open/close 直接是 0-100 归一值 → 映射到码值
        c = self.kin.cal["gripper"]
        self._gripper_code = lambda v: int(round(v / 100.0 * (c["max"] - c["min"]) + c["min"]))

        # 夹爪"软过载"配置（对照 LeRobot so101_follower.configure()）：
        #   Max_Torque_Limit(0x10)=500   —— 最大扭矩 50%，防烧毁
        #   Protection_Current(0x1C)=250 —— 过载电流阈值 50%
        #   Overload_Torque(0x24)=25     —— 过载时降到 25% 扭矩继续工作
        # 夹住盒子堵转时只降扭矩、不触发 0x20 硬锁存，位置校验才读得到
        # （默认配置一堵转就锁死总线，D4 实测；单写 Torque_Limit(0x30) 无效）。
        for _addr, _val, _size in ((0x10, 500, 2), (0x1C, 250, 2), (0x24, 25, 1)):
            try:
                if _size == 2:
                    self.bus.write_u16(6, _addr, _val)
                else:
                    self.bus.write_u8(6, _addr, _val)
            except ProtocolError:
                pass  # 夹爪已锁存/离线时写失败可接受（本轮快速报错而不是卡死）

        self._camera_read: Callable[[], np.ndarray] | None = None  # 由 demo 注入相机读帧

        gl = config["grasp_loop"]
        self.max_retries = int(gl["max_retries"])
        self.detect_timeout = float(gl["detect_timeout_s"])
        self.n_trials = int(gl["n_trials"])
        self.random_place = bool(gl["random_place"])

        self.stats = {"trials": 0, "success": 0, "fail": 0, "fail_reasons": {}}

    # ------------------------------------------------------------------
    # 基础动作（带重试）
    # ------------------------------------------------------------------
    def read_present_codes(self) -> dict[int, int]:
        return _retry(lambda: self.arm.read_positions(), self.serial_retries, "读取关节位置")

    def move_joints_deg(self, joints_deg: dict[str, float], fast: bool = False) -> None:
        """把 {关节: 角度} 转成码值并平滑移动关节 1-5；夹爪(6)单独控制，绝不被触碰。

        移动前做【路径级防撞】：起点姿态可能变化，插值路径任何一点指尖 z
        低于 z_safe_min 都拒绝执行（抛 PathUnsafeError），防止撞桌。
        """
        present = self.read_present_codes()
        targets: dict[int, float] = {}
        for sid in self.arm.servo_ids:
            if sid == 6:
                continue  # 夹爪由 set_gripper 独立控制（避免抬升时误改夹爪目标）
            jname = _joint_name(sid)
            if jname in joints_deg:
                targets[sid] = self.kin.deg_to_code(jname, joints_deg[jname])
            else:
                targets[sid] = float(present[sid])  # 保持当前
        if not targets:
            return
        # 路径级防撞：从任意当前位姿出发，整条插值路径不得低于安全下限
        cur_deg = {n: self.kin.code_to_deg(n, present[s]) for s, n in _SERVO_DEG.items()}
        tgt_deg = {n: self.kin.code_to_deg(n, targets[s]) for s, n in _SERVO_DEG.items()}
        self.kin.check_path_safe(cur_deg, tgt_deg, self.kin.safe_z_min)
        v = self.grasp_v if fast else self.v_max
        a = self.grasp_a if fast else self.a_max
        deadline = time.perf_counter() + self.move_timeout
        while True:
            try:
                self.arm.move_to(targets, v_max=v, a_max=a, settle_s=self.cfg["motion"]["settle_s"])
                return
            except ProtocolError:
                if time.perf_counter() > deadline:
                    raise GraspError(f"运动超时（>{self.move_timeout}s）")
                time.sleep(0.05)

    def set_gripper(self, value: float) -> None:
        """夹爪开合（0=夹紧 100=张开，RANGE_0_100 归一语义）。"""
        code = self._gripper_code(value)
        _retry(
            lambda: self.bus.write_u16(6, ADDR["goal_position"][0], code),
            self.serial_retries,
            f"夹爪写 Goal({code})",
        )

    def open_gripper(self) -> None:
        self.set_gripper(float(self.gripper_open_code))

    def close_gripper(self) -> None:
        self.set_gripper(float(self.gripper_close_code))

    def gripper_present_code(self) -> int:
        return _retry(lambda: self.bus.read_u16(6, ADDR["present_position"][0]), self.serial_retries, "夹爪读位置")

    def _gripper_state(self, open_present: int) -> str:
        """闭合后判夹爪状态：'grabbed'（咬住）/ 'empty'（空抓）/ 'fail'（夹爪异常）。

        无力度传感器，用"过载错误位 + 位置"近似（配合软过载配置）：
        - 过载位 0x20 且从张开位动过 → 夹爪顶在物体上（堵转）→ 咬住
        - 位置到 close_code 附近 → 关到底 → 空抓
        - 停在半途且动过（薄物未触发过载位）→ 咬住
        - 没动（堵死在张开位）或读失败 → 夹爪故障
        """
        try:
            raw, err = self.bus.read_u16_soft(6, ADDR["present_position"][0])
        except ProtocolError:
            return "fail"
        present = decode_sign_magnitude(raw)
        close_code = self._gripper_code(float(self.gripper_close_code))
        moved = abs(present - open_present) > self.verify_delta
        if err & ERRBIT_OVERLOAD:
            return "grabbed" if moved else "fail"
        if abs(present - close_code) <= self.verify_delta:
            return "empty"
        return "grabbed" if moved else "fail"

    # ------------------------------------------------------------------
    # 检测
    # ------------------------------------------------------------------
    def detect_target(self, frame: np.ndarray) -> Target | None:
        return self.detector.detect_one(frame)

    def pixel_to_center(self, u: float, v: float) -> tuple[float, float]:
        """目标像素 → 夹爪中心基座 XY。

        标定记录的是"活动爪尖点像素 ↔ 尖点基座 XY"（夹爪最大张开、中心被遮挡时点尖点）。
        抓取时目标中心应对准夹爪中心，故把尖点目标位置逆补偿 half_span 得到中心位置。
        """
        bx, by = self.calib.pixel_to_base(u, v)
        g = self.cfg["gripper"]
        half, side = float(g["half_span"]), float(g["jaw_side"])
        pan = math.atan2(by - self.kin.y0, bx - self.kin.x0)
        return bx + side * half * math.sin(pan), by - side * half * math.cos(pan)

    # ------------------------------------------------------------------
    # 抓取一轮
    # ------------------------------------------------------------------
    def _approach_and_grasp(self, bx: float, by: float, wrist_deg: float | None = None) -> bool:
        """移动到 (bx, by) 完成 接近→下降→夹取→抬起→校验，返回是否抓到。

        bx/by: pixel_to_center 输出（夹爪中心目标的近似值）。
        wrist_deg: 夹爪方向（wrist_roll 目标角），None=保持当前；用于长方形目标垂直长边夹取。

        自洽性说明（勿改成"中心→爪尖"）：
          标定记录的是"爪尖像素 ↔ 爪尖基座 G+u"（u=半宽偏移），H 输出=爪尖基座。
          pixel_to_center = H - u（近似中心）；把它当爪尖目标给 ik_vertical，
          中心 = (H-u) + u = H ≈ 盒子基座。半宽/爪侧符号在标定与抓取中抵消，
          不影响中心落点。改成"中心→爪尖"会让中心偏一个半宽（D4 实测教训）。
        """
        def _with_wrist(joints: dict[str, float]) -> dict[str, float]:
            if wrist_deg is not None:
                joints = dict(joints)
                joints["wrist_roll"] = self.kin.clamp_joint("wrist_roll", wrist_deg)
            return joints

        # 夹爪先张开到最大（每次抓取前必做，否则闭合状态下降夹不到）
        self.open_gripper()
        time.sleep(0.4)

        # APPROACH：目标正上方（保持夹爪竖直）
        pre = _with_wrist(self.kin.ik_vertical(bx, by, self.z_pre))
        self.move_joints_deg(pre, fast=True)
        time.sleep(0.3)

        # DESCEND：降到夹取高度
        down = _with_wrist(self.kin.ik_vertical(bx, by, self.z_grasp))
        self.move_joints_deg(down, fast=True)
        time.sleep(0.3)

        # 闭合前记录夹爪张开位（VERIFY 需要：区分"咬住"与"压根没动"）
        open_present = self.gripper_present_code()

        # CLOSE：夹爪闭合（等待稳定）
        self.close_gripper()
        time.sleep(0.6)

        # LIFT：抬起
        lift = _with_wrist(self.kin.ik_vertical(bx, by, self.z_lift))
        self.move_joints_deg(lift, fast=True)
        time.sleep(0.3)

        # VERIFY：夹爪状态 → 判断咬合/空抓（无力/力矩传感器，用"堵转+位置"近似）
        # 1) 过载错误位 0x20 = 夹爪顶在东西上（堵转）且确实从张开位动过 → 咬住物体
        # 2) 关到底（离 close_code 近）→ 空抓
        # 3) 停在半途且动过 → 咬住（薄物没触发过载位）
        # 4) 压根没动（堵死在张开位）→ 夹爪故障，不算成功
        state = self._gripper_state(open_present)
        if state == "empty":
            return False  # 关到底 = 没抓到物体
        if state == "fail":
            return False  # 夹爪没动/读不了 → 按失败重试
        return True  # 咬住物体

    def _target_long_axis_base(self, target: Target) -> float:
        """目标长边在基座系的方位角（rad）。用单应把图像长边方向映射到基座系。"""
        theta_img = math.radians(target.angle)
        scale = 60.0  # 图像采样长度（像素）
        bx0, by0 = self.calib.pixel_to_base(target.x, target.y)
        bx1, by1 = self.calib.pixel_to_base(
            target.x + scale * math.cos(theta_img), target.y + scale * math.sin(theta_img)
        )
        return math.atan2(by1 - by0, bx1 - bx0)

    def _wrist_for_target(self, target: Target, bx: float, by: float) -> float:
        """让夹爪两片连线垂直于目标长边 → wrist_roll 目标角（度，wrap 到 [-180,180]）。

        物理模型：夹爪连线基座方位角 α = pan + α0 + s·ψ（s=wrist_roll_sign 方向，
        α0=零位偏置，由 d4_calib_wrist.py 标定写入 grip_offset_deg）。
        令 α=θ_grip → ψ = s·(θ_grip - pan) + grip_off。
        """
        theta_long = self._target_long_axis_base(target)
        theta_grip = theta_long + math.pi / 2  # 夹爪开口方向 ⊥ 长边
        pan = math.atan2(by - self.kin.y0, bx - self.kin.x0)
        wrist_sign = float(self.cfg["kinematics"].get("wrist_roll_sign", 1.0))
        grip_off = float(self.cfg["gripper"].get("grip_offset_deg", 0.0))
        wrist_deg = wrist_sign * math.degrees(theta_grip - pan) + grip_off
        wrist_deg = (wrist_deg + 180.0) % 360.0 - 180.0  # wrap 到 [-180,180]（避免跨边界跳变）
        return self.kin.clamp_joint("wrist_roll", wrist_deg)

    def grasp_once(self, frame: np.ndarray, log: Callable[[str], None] = print) -> bool:
        """单轮抓取（含检测/夹取重试）。返回是否成功。

        Args:
            frame: 当前相机画面（含目标）。
            log: 日志回调。
        """
        self.stats["trials"] += 1

        # 检测（重试 + 超时）
        target = None
        deadline = time.perf_counter() + self.detect_timeout
        for i in range(self.max_retries + 1):
            target = self.detect_target(frame)
            if target is not None:
                break
            if time.perf_counter() > deadline:
                break
            if i < self.max_retries:
                log(f"[抓取] 目标检测失败，{i + 1}/{self.max_retries} 次重试 ...")
                time.sleep(0.5)
                frame = self.camera_read()
        if target is None:
            self.stats["fail"] += 1
            self.stats["fail_reasons"]["detect_fail"] = self.stats["fail_reasons"].get("detect_fail", 0) + 1
            log("[抓取] 目标检测失败（超时/未找到目标色块）")
            return False

        # 像素 → 基座 XY（平面单应）
        bx, by = self.pixel_to_center(target.x, target.y)
        if np.hypot(bx - self.kin.x0, by - self.kin.y0) > self.max_reach:
            self.stats["fail"] += 1
            self.stats["fail_reasons"]["out_of_reach"] = self.stats["fail_reasons"].get("out_of_reach", 0) + 1
            log(f"[抓取] 目标超出工作半径 {self.max_reach:.2f}m（基座系 ({bx:.3f}, {by:.3f})）")
            return False
        wrist_deg = self._wrist_for_target(target, bx, by) if target.angle is not None else None
        log(f"[抓取] 检测到目标 @px({target.x},{target.y}) → 基座 ({bx:.3f}, {by:.3f})"
            f"{'  目标长边角=' + str(round(wrist_deg, 1)) + '°(腕部)' if wrist_deg is not None else ''}")

        # 重试循环：夹取失败可再次下降（目标可能被碰歪）
        for attempt in range(self.max_retries + 1):
            if attempt:
                log(f"[抓取] 第 {attempt} 次重试（重新检测）")
                frame = self.camera_read()
                target = self.detect_target(frame)
                if target is None:
                    break
                bx, by = self.pixel_to_center(target.x, target.y)
                wrist_deg = self._wrist_for_target(target, bx, by) if target.angle is not None else None
            try:
                ok = self._approach_and_grasp(bx, by, wrist_deg)
            except (GraspError, KinematicsError) as e:
                log(f"[抓取] 运动异常: {e} → 回安全位")
                self.safe_home()
                break
            if ok:
                self.stats["success"] += 1
                log("[抓取] ✅ 成功（夹爪咬合）")
                self.place_item()  # 移到放置点并松开放下
                return True
            log("[抓取] ❌ 抓空（夹爪关到底）→ 复位重试")
            # 空抓：松开 → 回到 z_pre → 继续重试
            self.open_gripper()
            pre = self.kin.ik_vertical(bx, by, self.z_pre)
            self.move_joints_deg(pre, fast=True)

        self.stats["fail"] += 1
        self.stats["fail_reasons"]["grasp_fail"] = self.stats["fail_reasons"].get("grasp_fail", 0) + 1
        log("[抓取] 重试耗尽，本轮失败")
        return False

    def camera_read(self) -> np.ndarray:
        """读取相机画面（由 demo 注入的读取函数；未注入时抛错）。"""
        if self._camera_read is None:
            raise GraspError("未注入 camera_read 回调")
        return self._camera_read()

    # ------------------------------------------------------------------
    # 流程编排
    # ------------------------------------------------------------------
    def place_item(self) -> None:
        """抓取成功后：移到放置点 → 松开夹爪放下物品（不阻塞流程）。"""
        px, py = self.cfg.get("grasp_loop", {}).get("place_xy", [0.06, 0.10])
        try:
            pre = self.kin.ik_vertical(float(px), float(py), self.z_pre)
            self.move_joints_deg(pre, fast=True)
            self.open_gripper()
            time.sleep(0.6)
            print(f"[放置] 已移到放置点 ({px:.3f}, {py:.3f}) 并松开夹爪")
        except Exception as e:  # noqa: BLE001
            print(f"[放置] 放置失败: {e}（夹爪保持闭合）")

    def safe_home(self) -> None:
        """安全回位（三段式，避免低高度水平移动撞物）：
        1) 垂直抬升到高位（保持当前 XY）
        2) 高位水平移到安全位姿正上方
        3) 下降/姿态还原到安全位姿（safe_pose 或量程中点）
        """
        pose = self.cfg.get("safe_pose") or self.kin.home_pose()
        try:
            # 当前指尖位置（读编码器 → FK）
            present = self.read_present_codes()
            cur_deg = {
                "shoulder_pan": self.kin.code_to_deg("shoulder_pan", present[1]),
                "shoulder_lift": self.kin.code_to_deg("shoulder_lift", present[2]),
                "elbow_flex": self.kin.code_to_deg("elbow_flex", present[3]),
                "wrist_flex": self.kin.code_to_deg("wrist_flex", present[4]),
            }
            cur_xyz = self.kin.fk({**cur_deg, "wrist_roll": 0.0, "gripper": 0.0})
            safe_xyz = self.kin.fk({**pose, "wrist_roll": pose.get("wrist_roll", 0.0), "gripper": 0.0})
            z_high = min(max(cur_xyz[2], safe_xyz[2]) + 0.02, 0.20)  # 抬升高度（上限 0.20）

            # 1) 垂直抬升（保持当前 XY，只动 z）
            lift = self.kin.ik_vertical(cur_xyz[0], cur_xyz[1], z_high)
            self.move_joints_deg(lift, fast=True)

            # 2) 高位水平移动（安全 XY 正上方，保持 z_high）
            try:
                move = self.kin.ik_vertical(safe_xyz[0], safe_xyz[1], z_high)
                self.move_joints_deg(move, fast=True)
            except KinematicsError:
                print("[安全] 高位水平移动不可达，从当前位置直接回落安全位姿")

            # 3) 下降到安全位姿（姿态还原，XY 已对齐）
            self.move_joints_deg(pose, fast=True)
        except Exception as e:  # noqa: BLE001
            print(f"[安全] 回安全位失败: {e}（保持当前力矩）")

    def run_trials(self, n: int | None = None, log: Callable[[str], None] = print,
                   preview: Callable[[int, int], None] | None = None) -> dict[str, Any]:
        """连续执行 n 轮抓取（默认取配置），输出成功率统计。

        preview: 每轮检测前的回调(i, n)，可用来显示相机画面确认目标可见。
        """
        n = n or self.n_trials
        log(f"===== D4 抓取闭环：{n} 轮 =====")
        self.stats = {"trials": 0, "success": 0, "fail": 0, "fail_reasons": {}}
        self.safe_home()
        for i in range(1, n + 1):
            log(f"--- 第 {i}/{n} 轮 ---")
            if preview is not None:
                preview(i, n)
            frame = self.camera_read()
            self.grasp_once(frame, log=log)
            self.safe_home()
            time.sleep(1.0)
        s = self.stats
        rate = s["success"] / s["trials"] * 100 if s["trials"] else 0.0
        log(f"===== 结果: {s['success']}/{s['trials']} 成功（{rate:.1f}%） 失败原因: {s['fail_reasons']} =====")
        return {**s, "success_rate": rate}

    def install_sigint_handler(self, estop: bool = False) -> None:
        """Ctrl+C 优雅退出：回 HOME；estop=True 时退出前失能力矩。"""

        def _handler(sig, frame):  # noqa: ANN001
            print("\n[中断] Ctrl+C 收到 → 回安全位")
            try:
                self.safe_home()
            finally:
                if estop:
                    try:
                        for sid in self._estop_ids:
                            self.bus.write_u8(sid, ADDR["torque_enable"][0], 0)
                        print("[中断] 已失能力矩（急停）")
                    except Exception:  # noqa: BLE001
                        pass
                sys.exit(130)

        signal.signal(signal.SIGINT, _handler)


def _joint_name(servo_id: int) -> str:
    """舵机 ID → 关节名（SO-101 从臂）。"""
    return {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}[servo_id]


# 参与路径防撞验算的关节（FK 需要；夹爪 6 不参与）
_SERVO_DEG = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll"}
