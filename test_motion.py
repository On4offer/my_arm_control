#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D2 运动控制层离线单测：无需硬件，验证轨迹规划数学与限幅逻辑。

运行：
  python test_motion.py
  pytest test_motion.py
"""

from motion import (
    EaseProfile,
    LinearProfile,
    TrapezoidalProfile,
    clamp_relative,
    ease_in_out_sine,
)


# ---- 梯形速度规划 ----
def test_trapezoid_endpoints():
    p = TrapezoidalProfile(0.0, 1000.0, v_max=200.0, a_max=400.0)
    assert p.position(0.0) == 0.0
    assert p.position(p.T) == 1000.0
    assert p.position(p.T + 10.0) == 1000.0  # 超时保持终点


def test_trapezoid_speed_accel_bounds():
    """任意时刻 |v| ≤ v_max，|a| ≤ a_max。"""
    p = TrapezoidalProfile(0.0, 1000.0, v_max=200.0, a_max=400.0)
    dt = 1e-3
    t = 0.0
    v_prev = 0.0
    while t <= p.T + dt:
        v = p.velocity(t)
        assert abs(v) <= p.v_max + 1e-6, f"超速: t={t} v={v}"
        # 数值加速度
        v_next = p.velocity(min(t + dt, p.T))
        a = (v_next - v) / dt if t < p.T else 0.0
        assert abs(a) <= p.a_max + 1e-3, f"超加速度: t={t} a={a}"
        # 单调性（正向运动）
        assert v >= -1e-6
        t += dt


def test_trapezoid_symmetric_and_triangle():
    """短距离退化为三角形剖面（无匀速段），且对称性正确。"""
    p_full = TrapezoidalProfile(0.0, 1000.0, v_max=200.0, a_max=400.0)
    assert p_full.t_c > 0.0  # 长距离有匀速段
    assert abs(p_full.T - (2 * 0.5 + (1000 - 2 * 50) / 200.0)) < 1e-6  # 2*50+900/200=5.5s

    p_tri = TrapezoidalProfile(0.0, 100.0, v_max=200.0, a_max=400.0)
    assert p_tri.t_c == 0.0  # 短距离无匀速段
    v_p = p_tri.v_peak
    assert abs(v_p - 200.0) < 1e-6  # sqrt(400*100)=200，恰达满速
    # 对称：中点时刻位置应为半程
    assert abs(p_tri.position(p_tri.T / 2) - 50.0) < 1e-3


def test_trapezoid_negative_direction():
    p = TrapezoidalProfile(1000.0, 200.0, v_max=200.0, a_max=400.0)
    assert p.position(0.0) == 1000.0
    assert p.position(p.T) == 200.0
    v = p.velocity(p.t_a)  # 匀速段速度应为 -v_max
    assert abs(v + 200.0) < 1e-6


def test_trapezoid_zero_distance():
    p = TrapezoidalProfile(500.0, 500.0, v_max=200.0, a_max=400.0)
    assert p.position(0.0) == 500.0
    assert p.position(1.0) == 500.0


# ---- 线性 / 缓动 ----
def test_linear_profile():
    p = LinearProfile(0.0, 100.0, duration=2.0)
    assert p.position(0.0) == 0.0
    assert p.position(1.0) == 50.0
    assert p.position(2.0) == 100.0
    assert p.position(5.0) == 100.0  # 超时保持终点


def test_ease_profile_endpoints_and_smoothness():
    p = EaseProfile(0.0, 100.0, duration=2.0)
    assert p.position(0.0) == 0.0
    assert p.position(2.0) == 100.0
    assert p.position(1.0) == 50.0 or abs(p.position(1.0) - 50.0) < 1e-9  # sine ease 中点为半程（对称）
    # 起点/终点速度接近零（有限差分在边界处残差为 O(dt)，与中段速度对比验证平滑）
    dt = 1e-3
    v0 = (p.position(dt) - p.position(0.0)) / dt
    v1 = (p.position(2.0) - p.position(2.0 - dt)) / dt
    v_mid = (p.position(1.0 + dt) - p.position(1.0 - dt)) / (2 * dt)
    assert v_mid > 50.0  # 中段速度显著（sine ease 峰值速度 ≈ 78.5 码/s）
    assert abs(v0) < v_mid * 1e-3 and abs(v1) < v_mid * 1e-3
    assert ease_in_out_sine(0.0) == 0.0 and ease_in_out_sine(1.0) == 1.0


# ---- 限幅（对照 LeRobot ensure_safe_goal_position） ----
def test_clamp_relative():
    assert clamp_relative(goal=1000.0, present=0.0, max_diff=100.0) == 100.0
    assert clamp_relative(goal=-1000.0, present=0.0, max_diff=100.0) == -100.0
    assert clamp_relative(goal=50.0, present=0.0, max_diff=100.0) == 50.0  # 小增量不限
    assert clamp_relative(goal=0.0, present=0.0, max_diff=100.0) == 0.0


if __name__ == "__main__":
    import sys
    import traceback

    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    passed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
            passed += 1
        except Exception:
            print(f"FAIL  {fn.__name__}")
            traceback.print_exc()
    print(f"\n汇总: {passed} PASS / {len(fns) - passed} FAIL / {len(fns)} 总检查项")
    sys.exit(0 if passed == len(fns) else 1)
