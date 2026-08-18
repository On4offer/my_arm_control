# -*- coding: utf-8 -*-
"""
手搓运动控制层（D2）：轨迹规划 + 限位/限速 + 多关节同步下发。

对照 LeRobot（v0.6.2）：
- common/control_utils.py  `teleop_smooth_move_to` / `follower_smooth_move_to`
    —— 线性插值 + 固定 fps 步进（平滑过渡用）
- robots/so_follower/so_follower.py `send_action` + robots/utils.py `ensure_safe_goal_position`
    —— 每帧写 Goal_Position，并用 max_relative_target 把"本帧增量"限制在 ±max_diff（隐式限速）
- 位置闭环本身在舵机内部 PID（P/D/I 寄存器 21/22/23）完成
+
本模块的三层结构（对应 D2 任务）：
1. 轨迹规划：TrapezoidalProfile（梯形速度：加速-匀速-减速）/ LinearProfile / EaseProfile
2. 安全限幅：目标截断到 Min/Max_Position_Limit + 逐帧增量限幅 clamp_relative（对照 ensure_safe_goal_position）
3. 控制下发：ArmController 按 fps 把规划位置写入各舵机 Goal_Position，可记录轨迹
"""

from __future__ import annotations

import csv
import math
import time

from .protocol import ADDR, FeetechSerialBus, ProtocolError, decode_sign_magnitude, encode_sign_magnitude


# ---------------------------------------------------------------------------
# 1. 轨迹规划
# ---------------------------------------------------------------------------
class TrapezoidalProfile:
    """梯形速度规划（加速-匀速-减速三段）。

    标准梯形速度曲线：
      加速段  q(t) = q0 + 0.5*a*t²                    0 ≤ t <  t_a
      匀速段  q(t) = q0 + d_a + v*(t - t_a)           t_a ≤ t <  t_a + t_c
      减速段  q(t) = qf - 0.5*a*(T - t)²              t_a + t_c ≤ t < T
    t_a = t_d = v/a（满速时）；若距离不够则退化为三角形剖面（无匀速段）。
    """

    def __init__(self, q0: float, qf: float, v_max: float, a_max: float):
        if v_max <= 0 or a_max <= 0:
            raise ValueError(f"v_max/a_max 必须为正: {v_max=} {a_max=}")
        self.q0 = float(q0)
        self.qf = float(qf)
        self.v_max = float(v_max)
        self.a_max = float(a_max)

        d_abs = abs(self.qf - self.q0)
        sign = 1.0 if self.qf >= self.q0 else -1.0
        d_a_full = self.v_max**2 / (2 * self.a_max)  # 加速段位移（达到满速时）
        if d_abs <= 2 * d_a_full:
            # 三角形剖面：达不到满速，峰值速度 v_p = sqrt(a * d)
            v_p = math.sqrt(self.a_max * d_abs)
            self.v_peak = sign * v_p
            self.t_a = v_p / self.a_max
            self.t_c = 0.0
            self.T = 2 * self.t_a
        else:
            self.v_peak = sign * self.v_max
            self.t_a = self.v_max / self.a_max
            self.t_c = (d_abs - 2 * d_a_full) / self.v_max
            self.T = 2 * self.t_a + self.t_c
        self.d_a = sign * 0.5 * self.a_max * self.t_a**2  # 加速段位移（带符号）

    def position(self, t: float) -> float:
        t = float(t)
        if t <= 0.0:
            return self.q0
        if t >= self.T:
            return self.qf
        a = self.v_peak / self.t_a  # 带符号加速度
        if t < self.t_a:
            return self.q0 + 0.5 * a * t**2
        if t < self.t_a + self.t_c:
            return self.q0 + self.d_a + self.v_peak * (t - self.t_a)
        t_dec = t - self.t_a - self.t_c  # 减速段内部时间
        return self.qf - 0.5 * a * (self.t_a - t_dec) ** 2

    def velocity(self, t: float) -> float:
        """解析速度（用于单测验证 v ≤ v_max、|a| ≤ a_max）。"""
        t = float(t)
        if t <= 0.0:
            return 0.0
        if t >= self.T:
            return 0.0
        a = self.v_peak / self.t_a
        if t < self.t_a:
            return a * t
        if t < self.t_a + self.t_c:
            return self.v_peak
        t_dec = t - self.t_a - self.t_c
        return a * (self.t_a - t_dec)


class LinearProfile:
    """匀速线性插值（对照 LeRobot `follower_smooth_move_to` 的线性平滑）。"""

    def __init__(self, q0: float, qf: float, duration: float):
        if duration <= 0:
            raise ValueError(f"duration 必须为正: {duration}")
        self.q0 = float(q0)
        self.qf = float(qf)
        self.T = float(duration)

    def position(self, t: float) -> float:
        t = min(max(float(t), 0.0), self.T)
        return self.q0 + (self.qf - self.q0) * (t / self.T)


def ease_in_out_sine(t: float) -> float:
    """缓动函数（sine ease-in-out，起点/终点速度为零，无瞬时跳变）。"""
    return -(math.cos(math.pi * t) - 1.0) / 2.0


class EaseProfile:
    """ease-in-out-sine 缓动（起点/终点速度为零，比线性插值更平滑）。"""

    def __init__(self, q0: float, qf: float, duration: float):
        if duration <= 0:
            raise ValueError(f"duration 必须为正: {duration}")
        self.q0 = float(q0)
        self.qf = float(qf)
        self.T = float(duration)

    def position(self, t: float) -> float:
        t = min(max(float(t), 0.0), self.T)
        return self.q0 + (self.qf - self.q0) * ease_in_out_sine(t / self.T)


# ---------------------------------------------------------------------------
# 2. 安全限幅（对照 LeRobot `ensure_safe_goal_position`）
# ---------------------------------------------------------------------------
def clamp_relative(goal: float, present: float, max_diff: float) -> float:
    """把单帧增量限制在 ±max_diff 内（对照 LeRobot 的 max_relative_target 限幅）。

    作用：即使轨迹/指令瞬间跳变，每帧最多移动 max_diff 码 → 隐式限速，防冲击。
    """
    diff = goal - present
    diff = min(max(diff, -max_diff), max_diff)
    return present + diff


# ---------------------------------------------------------------------------
# 3. 控制下发
# ---------------------------------------------------------------------------
class ArmController:
    """按 fps 把轨迹规划位置写入多关节舵机，带限位截断与逐帧限幅。"""

    def __init__(
        self,
        bus: FeetechSerialBus,
        servo_ids: list[int],
        fps: int = 50,
        max_step: float | None = None,
    ):
        self.bus = bus
        self.servo_ids = list(servo_ids)
        self.fps = fps
        self.dt = 1.0 / fps
        self.max_step = max_step  # 每帧最大步长（码），None=不限制

    # ---- 状态读取 ----
    def read_positions(self) -> dict[int, int]:
        """读各关节 Present_Position（原始码值）。"""
        positions = {}
        for sid in self.servo_ids:
            positions[sid] = decode_sign_magnitude(
                self.bus.read_u16(sid, ADDR["present_position"][0])
            )
        return positions

    def get_limits(self) -> dict[int, tuple[int, int]]:
        """读各关节 Min/Max_Position_Limit。"""
        limits = {}
        for sid in self.servo_ids:
            lo = self.bus.read_u16(sid, ADDR["min_position_limit"][0])
            hi = self.bus.read_u16(sid, ADDR["max_position_limit"][0])
            limits[sid] = (lo, hi)
        return limits

    def enable_torque(self) -> None:
        for sid in self.servo_ids:
            self.bus.write_u8(sid, ADDR["torque_enable"][0], 1)

    def disable_torque(self) -> None:
        for sid in self.servo_ids:
            self.bus.write_u8(sid, ADDR["torque_enable"][0], 0)

    # ---- 轨迹下发 ----
    def move_to(
        self,
        targets: dict[int, float],
        profile: str = "trapezoid",
        v_max: float = 200.0,
        a_max: float = 400.0,
        duration_s: float | None = None,
        log_path: str | None = None,
        settle_s: float = 1.5,
        settle_tol: float = 10.0,
    ) -> dict:
        """把各关节平滑运动到目标码值。

        Args:
            targets: {关节ID: 目标码值}。目标会被截断到该关节限位内。
            profile: "trapezoid"（梯形速度，推荐）/ "linear"（线性插值，LeRobot 对照）/ "ease"（缓动）。
            v_max: 梯形规划最大速度（码/秒）。
            a_max: 梯形规划最大加速度（码/秒²）。
            duration_s: linear/ease 的固定时长；trapezoid 忽略（由 v_max/a_max 决定）。
            log_path: 非空则把轨迹写入 CSV（time, t_real, servo_id, goal, present）。
            settle_s: 轨迹结束后保持最终目标的稳定期（秒），让舵机内部 PID 跟上。
            settle_tol: 稳定期到位判定容差（码）。

        Returns:
            {"profiles": ..., "duration_s": T, "start": ..., "end": ..., "rows": [...]}
        """
        limits = self.get_limits()
        present = self.read_positions()

        # 1) 目标截断到限位
        clipped = {}
        for sid in self.servo_ids:
            lo, hi = limits[sid]
            clipped[sid] = min(max(float(targets.get(sid, present[sid])), lo), hi)
        targets = clipped

        # 2) 构造各关节轨迹
        if profile == "trapezoid":
            trajs = {
                sid: TrapezoidalProfile(present[sid], targets.get(sid, present[sid]), v_max, a_max) for sid in self.servo_ids
            }
            T = max(trajs[sid].T for sid in self.servo_ids)
        elif profile == "linear":
            if duration_s is None:
                duration_s = max(abs(targets.get(sid, present[sid]) - present[sid]) for sid in self.servo_ids) / v_max
            trajs = {sid: LinearProfile(present[sid], targets.get(sid, present[sid]), duration_s) for sid in self.servo_ids}
            T = duration_s
        elif profile == "ease":
            if duration_s is None:
                duration_s = max(abs(targets.get(sid, present[sid]) - present[sid]) for sid in self.servo_ids) / v_max
            trajs = {sid: EaseProfile(present[sid], targets.get(sid, present[sid]), duration_s) for sid in self.servo_ids}
            T = duration_s
        else:
            raise ValueError(f"未知 profile: {profile}（可选 trapezoid/linear/ease）")

        # 3) 使能力矩并进入控制循环（按墙钟时间对齐节拍，保证实际 fps ≈ 设定 fps）
        self.enable_torque()
        t = 0.0
        t_wall0 = time.perf_counter()
        rows: list[dict] = []
        cur = present
        while t < T + self.dt:
            goals = {}
            for sid in self.servo_ids:
                q = trajs[sid].position(t)
                if self.max_step is not None:
                    q = clamp_relative(q, cur[sid], self.max_step)
                lo, hi = limits[sid]
                q = min(max(q, lo), hi)  # 目标截断到限位：越限目标会被舵机拒绝(错误位 0x10)
                goals[sid] = int(round(q))
                self.bus.write_u16(sid, ADDR["goal_position"][0], encode_sign_magnitude(goals[sid]))
            # 回读实际位置（记录轨迹）
            try:
                cur = self.read_positions()
            except ProtocolError:
                pass  # 偶发失败不中断
            t_real = time.perf_counter() - t_wall0
            for sid in self.servo_ids:
                rows.append(
                    {"time": round(t, 3), "t_real": round(t_real, 3), "servo_id": sid, "goal": goals[sid], "present": cur[sid]}
                )
            # 节拍补偿：睡到下一帧目标墙钟时间
            sleep = (t_wall0 + t + self.dt) - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            t += self.dt

        # 4) 稳定期：保持最终目标，等待舵机内部 PID 到位（避免测量过早的假性误差）
        end = cur  # 最后已知位置（若稳定期读到更准的位置会被覆盖）
        if settle_s > 0:
            deadline = time.perf_counter() + settle_s
            while time.perf_counter() < deadline:
                try:
                    cur = self.read_positions()
                    end = cur
                except ProtocolError:
                    pass  # 舵机错误位（如过载报警）不中断，位置数据仍可后续重读
                if all(abs(cur[sid] - targets.get(sid, present[sid])) <= settle_tol for sid in self.servo_ids):
                    break
                time.sleep(0.05)

        if log_path:
            _write_csv(log_path, rows)
        return {
            "profile": profile,
            "duration_s": T,
            "start": present,
            "end": end,
            "rows": rows,
        }


def _write_csv(path: str, rows: list[dict]) -> None:
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["time", "t_real", "servo_id", "goal", "present"])
        writer.writeheader()
        writer.writerows(rows)
