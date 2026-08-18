# -*- coding: utf-8 -*-
"""
D4 视觉抓取：SO-101 平面运动学（FK / 垂直抓取 IK）与角度换算。

模型（对照 `../lerobot` URDF `so101_new_calib.urdf` 提取）：
  肩轴（shoulder_pan）在基座 (x0, y0, h0)；臂在垂直平面内转动，平面方位角 φ1=atan2(y,x)。
  平面内三连杆（上臂/前臂/腕部）：
    φ2 = s2*θ2 + o2   # 上臂相对竖直方向的夹角
    φ3 = s3*θ3 + o3   # 前臂相对上臂的夹角（肘关节弯曲量）
    φ4 = s4*θ4 + o4   # 腕部相对前臂的夹角
  夹爪竖直朝下 ⟺ φ2 + φ3 + φ4 = 180°
  L1 = 肩→肘, L2 = 肘→腕, D_tip = 腕→指尖（夹爪竖直时指尖在腕正下方）

角度约定（与 LeRobot `MotorsBus._normalize` 的 DEGREES 分支一致）：
  angle_deg = (raw - mid) * 360 / 4095，mid = (range_min + range_max) / 2
  raw = angle_deg * 4095 / 360 + mid

说明：H 由"真实手臂末端示教点"标定，IK 是同一 FK 模型的逆解——两者自洽，
模型误差（连杆长度/偏移）主要影响手腕竖直的物理性，可经真机校验（重投影误差）调参。
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

RESOLUTION = 4095  # 12 位编码器：4096 码 = 360°，LeRobot 归一化用 4095

JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class KinematicsError(Exception):
    """运动学错误（不可达 / 参数缺失等）。"""


class PathUnsafeError(KinematicsError):
    """路径级防撞拒绝：关节插值路径中指尖 z 低于安全下限。

    区别于 KinematicsError（目标本身不可达）：这是"当前/目标位姿过低、
    插值路径会低处横穿/下坠撞桌"，属于安全问题，调用方必须停止而不是兜底。
    """


def load_calibration(json_path: str | Path) -> dict[str, dict[str, float]]:
    """读取 LeRobot 校准 JSON，转成 {关节: {"mid":.., "min":.., "max":..}}。"""
    with open(json_path, "r", encoding="utf-8") as f:
        cal = json.load(f)
    out = {}
    for name, v in cal.items():
        rmin, rmax = float(v["range_min"]), float(v["range_max"])
        out[name] = {"mid": (rmin + rmax) / 2.0, "min": rmin, "max": rmax}
    return out


def deg_to_code(cal: dict[str, dict[str, float]], joint: str, deg: float) -> int:
    """角度（度，零在量程中点）→ 原始码值。"""
    return int(round(deg * RESOLUTION / 360.0 + cal[joint]["mid"]))


def code_to_deg(cal: dict[str, dict[str, float]], joint: str, code: int) -> float:
    """原始码值 → 角度（度）。"""
    return (code - cal[joint]["mid"]) * 360.0 / RESOLUTION


class SO101Kinematics:
    """SO-101 平面运动学：FK + 垂直抓取 IK（含限位/可达性检查）。"""

    def __init__(self, config: dict[str, Any], calibration: dict[str, dict[str, float]] | None = None):
        k = config["kinematics"]
        self.x0 = float(k["shoulder_x"])
        self.y0 = float(k["shoulder_y"])
        self.h0 = float(k["shoulder_h"])
        self.l1 = float(k["link1"])
        self.l2 = float(k["link2"])
        self.d_tip = float(k["wrist_to_tip"])
        self.sign = {j: float(k["sign"][j]) for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")}
        self.offset = {j: float(k["offset_deg"][j]) for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")}
        self.elbow_branch = k.get("elbow_branch", "down")

        # 防撞桌面安全下限（由 d4_probe_z.py 探高校准后写入；未配置则跳过校验）
        try:
            self.safe_z_min = float(config["table"]["z_safe_min"])
        except (KeyError, TypeError, ValueError):
            self.safe_z_min = None

        self.cal = calibration or {}
        # 关节角度限位（度），来自校准 JSON
        self.limits_deg: dict[str, tuple[float, float]] = {}
        for j in JOINT_ORDER:
            if j in self.cal:
                c = self.cal[j]
                lo = (c["min"] - c["mid"]) * 360.0 / RESOLUTION
                hi = (c["max"] - c["mid"]) * 360.0 / RESOLUTION
                self.limits_deg[j] = (min(lo, hi), max(lo, hi))

    # ---- 角度换算 ----
    def deg_to_code(self, joint: str, deg: float) -> int:
        if joint not in self.cal:
            raise KinematicsError(f"无 {joint} 校准数据")
        return deg_to_code(self.cal, joint, deg)

    def code_to_deg(self, joint: str, code: int) -> float:
        if joint not in self.cal:
            raise KinematicsError(f"无 {joint} 校准数据")
        return code_to_deg(self.cal, joint, code)

    def clamp_joint(self, joint: str, deg: float) -> float:
        """把关节角度截断到校准限位内。"""
        if joint in self.limits_deg:
            lo, hi = self.limits_deg[joint]
            return min(max(deg, lo), hi)
        return deg

    # ---- 平面内几何（φ 为模型角，单位弧度） ----
    def _phi2(self, deg: float) -> float:
        return math.radians(self.sign["shoulder_lift"] * deg + self.offset["shoulder_lift"])

    def _phi3(self, deg: float) -> float:
        return math.radians(self.sign["elbow_flex"] * deg + self.offset["elbow_flex"])

    def _phi4(self, deg: float) -> float:
        return math.radians(self.sign["wrist_flex"] * deg + self.offset["wrist_flex"])

    def _phi_from_joints(self, joints_deg: dict[str, float]) -> tuple[float, float, float]:
        return self._phi2(joints_deg["shoulder_lift"]), self._phi3(joints_deg["elbow_flex"]), self._phi4(joints_deg["wrist_flex"])

    # ---- 正运动学 ----
    def fk(self, joints_deg: dict[str, float]) -> tuple[float, float, float]:
        """末端（指尖）在基座系的 (x, y, z)，单位米。夹爪姿态任意，按模型链推算。"""
        phi2, phi3, phi4 = self._phi_from_joints(joints_deg)
        # 腕关节在平面内位置（r 为离肩轴水平距离，z 为相对肩轴高度）
        r_w = self.l1 * math.sin(phi2) + self.l2 * math.sin(phi2 + phi3)
        z_w = self.l1 * math.cos(phi2) + self.l2 * math.cos(phi2 + phi3)
        # 指尖沿夹爪方向（模型角 φ2+φ3+φ4）延伸 D_tip
        phi_tip = phi2 + phi3 + phi4
        r_t = r_w + self.d_tip * math.sin(phi_tip)
        z_t = z_w + self.d_tip * math.cos(phi_tip)
        # 平面方位角 → 基座 XYZ
        phi1 = math.radians(self.sign["shoulder_pan"] * joints_deg["shoulder_pan"] + self.offset["shoulder_pan"])
        x = self.x0 + r_t * math.cos(phi1)
        y = self.y0 + r_t * math.sin(phi1)
        z = self.h0 + z_t
        return x, y, z

    def check_path_safe(self, cur_deg: dict[str, float], tgt_deg: dict[str, float],
                        z_min: float | None = None, n_samples: int = 40) -> None:
        """路径级防撞护栏：采样"当前→目标"的关节插值整条路径，逐点 FK 验算。

        机械臂每次运行起点姿态都可能不同，直接 move 时关节线性插值可能
        在低处横穿/下坠撞桌——仅校验目标点 z 不够。这里把插值路径均匀
        采样 n_samples 段，任一采样点指尖（模型）z 低于 z_min 即抛
        PathUnsafeError 拒绝执行（不动臂）。

        cur_deg/tgt_deg: 各关节当前/目标角度（度），未涉及的关节保持不变。
        z_min: 安全下限；None 表示受控模式（如探高下降段）跳过校验。
        """
        if z_min is None:
            return
        joints = list(tgt_deg.keys())
        full_cur = {j: cur_deg.get(j, tgt_deg[j]) for j in joints}
        tol = 1e-3  # FK/关节码值量化误差容差（约 1mm），避免目标恰好贴在下限时误拒
        for i in range(n_samples + 1):
            t = i / n_samples
            deg = {j: full_cur[j] + (tgt_deg[j] - full_cur[j]) * t for j in joints}
            xyz = self.fk({**deg, "wrist_roll": deg.get("wrist_roll", 0.0), "gripper": 0.0})
            if xyz[2] < z_min - tol:
                if t > 0.95:
                    hint = "目标高度接近/低于安全下限 z_safe_min（标定/抓取高度必须明显高于它，请检查 z 配置）"
                else:
                    hint = "当前位姿过低，请先手动把机械臂抬高"
                raise PathUnsafeError(
                    f"路径防撞：插值 t={t:.2f} 处指尖 z={xyz[2]:.3f}m 低于安全下限 "
                    f"{z_min:.3f}m，已拒绝执行（{hint}）"
                )

    # ---- 垂直抓取逆运动学 ----
    def ik_vertical(self, x: float, y: float, z: float) -> dict[str, float]:
        """求夹爪竖直朝下、指尖到达基座 (x, y, z) 的关节角。

        返回 {shoulder_pan, shoulder_lift, elbow_flex, wrist_flex}（度，已夹到限位），
        不可达抛 KinematicsError。
        """
        # 防撞桌面硬约束：低于安全下限的 z 目标直接拒绝（探高校准后数值可信）
        if self.safe_z_min is not None and z < self.safe_z_min:
            raise KinematicsError(
                f"目标 z={z:.3f}m 低于安全下限 z_safe_min={self.safe_z_min:.3f}m，拒绝执行"
                f"（先运行 demos/d4_probe_z.py 校准桌面高度）"
            )
        # 1) 平面方位角
        rx, ry = x - self.x0, y - self.y0
        r = math.hypot(rx, ry)
        if r > self.l1 + self.l2 + 1e-6:
            raise KinematicsError(f"目标水平距离 {r:.3f}m 超过最大臂展 {self.l1 + self.l2:.3f}m")
        phi1 = math.atan2(ry, rx)
        shoulder_pan = (math.degrees(phi1) - self.offset["shoulder_pan"]) / self.sign["shoulder_pan"]

        # 2) 平面内 2R 解（腕关节位置）
        z_w = z - self.h0 + self.d_tip  # 指尖在腕下方 D_tip
        r_w = r
        d = math.hypot(r_w, z_w)
        if d < 1e-4:
            raise KinematicsError("目标点与肩轴重合，无法确定臂平面")
        if d < abs(self.l1 - self.l2) - 1e-6 or d > self.l1 + self.l2 + 1e-6:
            raise KinematicsError(f"腕关节距离 {d:.3f}m 超出可达范围 [{abs(self.l1 - self.l2):.3f}, {self.l1 + self.l2:.3f}]m")

        cos_a = (d**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        a = math.acos(max(-1.0, min(1.0, cos_a)))  # 肘关节弯曲量（无符号，[0, π]）
        if self.elbow_branch == "up":
            phi3 = a  # 肘"上"：上臂近竖直、前臂前下折叠（桌面抓取默认分支）
        else:
            phi3 = -a
        beta = math.atan2(r_w, z_w)  # 腕方向（相对竖直）
        gamma = math.atan2(self.l2 * math.sin(phi3), self.l1 + self.l2 * math.cos(phi3))
        phi2 = beta - gamma

        # 3) 夹爪竖直：φ2+φ3+φ4 = π
        phi4 = math.pi - phi2 - phi3

        shoulder_lift = (math.degrees(phi2) - self.offset["shoulder_lift"]) / self.sign["shoulder_lift"]
        elbow_flex = (math.degrees(phi3) - self.offset["elbow_flex"]) / self.sign["elbow_flex"]
        wrist_flex = (math.degrees(phi4) - self.offset["wrist_flex"]) / self.sign["wrist_flex"]

        out = {
            "shoulder_pan": self.clamp_joint("shoulder_pan", shoulder_pan),
            "shoulder_lift": self.clamp_joint("shoulder_lift", shoulder_lift),
            "elbow_flex": self.clamp_joint("elbow_flex", elbow_flex),
            "wrist_flex": self.clamp_joint("wrist_flex", wrist_flex),
        }
        # 限位截断后校验可达性（被截断的关节可能导致指尖偏差，提示用户）
        raw = [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex]
        if any(abs(out[j] - v) > 1e-6 for j, v in zip(out, raw, strict=True)):
            raise KinematicsError("IK 结果被关节限位截断，目标不可达（调近工作区或检查参数）")
        return out

    def home_pose(self) -> dict[str, float]:
        """回安全位姿：全部关节 0°（量程中点，竖直站立夹爪朝前）。"""
        return {j: 0.0 for j in JOINT_ORDER if j in self.limits_deg}
