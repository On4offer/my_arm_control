#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 手眼标定（eye-to-hand 平面单应）：像素 ↔ 机械臂基座 XY
============================================================

流程（机械臂 + 相机在环，一次性标定，结果存 config/d4_table_calib.json）：
  1. 机械臂移到桌面上方标定网格（默认 3x3）每个点，夹爪竖直朝下
  2. 相机画面中【鼠标左键点击指尖位置】（指尖应贴近桌面，点=桌面平面像素）
  3. 全部点收集完 → 求解单应矩阵 H（像素→基座 XY）→ 打印重投影误差 → 保存

用法：
  python d4_calibrate_table.py                # 全流程（移动+点击）
  python d4_calibrate_table.py --dry-run      # 不移动机械臂，仅预览相机+练点击
  python d4_calibrate_table.py --only-solve   # 用已存点重新求解/查看误差
  python d4_calibrate_table.py --port COM24 --grid 3x3

标定质量判定（重投影误差 rms_mm）：
  < 8mm  优秀；8~15mm 可用（目标≥4cm）；>15mm 检查：相机是否松动、
  指尖点击是否准确、运动学 offset_deg/sign 是否需调（见 kinematics.py 说明）。
"""

import argparse
import math
import sys
import time
from pathlib import Path

# 把项目根目录加入 sys.path（demos 目录直接运行时可导入 my_arm_control）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from d4_common import (  # noqa: E402
    DEFAULT_TABLE_CALIB,
    load_d4_config,
    load_table_calib,
    make_bus,
    make_camera,
    make_controller,
    make_kinematics,
    safe_move_xyz,
)
from my_arm_control.calibration import TableCalibration, grid_xy  # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D4：手眼标定（像素↔基座 XY 平面单应）")
    p.add_argument("--port", type=str, default=None, help="舵机总线串口（默认取 d4_config.json）")
    p.add_argument("--camera", type=int, default=None, help="相机索引（默认取 d4_config.json，不可用自动选择）")
    p.add_argument("--grid", type=str, default="3x3", help="标定网格 列x行（默认 3x3）")
    p.add_argument("--dry-run", action="store_true", help="不移动机械臂，仅预览相机")
    p.add_argument("--only-solve", action="store_true", help="仅用已存标定点重新求解")
    p.add_argument("--out", type=str, default=str(DEFAULT_TABLE_CALIB), help="标定 JSON 输出路径")
    p.add_argument("--z", type=float, default=None, help="指尖接近工作台高度（米，默认取配置 z_touch）")
    return p.parse_args(argv)


def _wheel_direction(flags: int) -> int:
    """滚轮方向：返回 +1(放大) / -1(缩小) / 0(无法判断)。

    兼容不同 OpenCV 后端的编码：
    - Win32 后端：上滚=EVENT_FLAG_CTRLKEY，下滚=EVENT_FLAG_SHIFTKEY
    - Qt/Cocoa 后端：delta 编码在高 16 位（低 16 位为 0），或直接低 16 位带符号
    """
    up = bool(flags & cv2.EVENT_FLAG_CTRLKEY)
    down = bool(flags & cv2.EVENT_FLAG_SHIFTKEY)
    if up and not down:
        return 1
    if down and not up:
        return -1
    raw = flags & 0xFFFF
    if raw == 0:
        raw = (flags >> 16) & 0xFFFF
    if raw == 0:
        return 0
    delta = raw if raw <= 32767 else raw - 65536
    return 1 if delta > 0 else -1


class ZoomPanView:
    """可缩放/平移的图像视图（滚轮缩放、右键拖动平移、左键选点）。

    所有外部坐标（选点/文字）都换算到【原始图像坐标】，与未缩放时一致，
    因此 H 标定使用的像素坐标不受缩放影响。
    """

    def __init__(self, width: int, height: int, max_scale: float = 10.0, step: float = 1.25):
        self.W, self.H = int(width), int(height)
        self.min_scale, self.max_scale = 1.0, max_scale
        self.step = step
        self.scale = 1.0
        self.cx, self.cy = self.W / 2.0, self.H / 2.0  # 视图中心（原始图像坐标）

    # ---- 坐标换算 ----
    def window_to_orig(self, x: float, y: float) -> tuple[float, float]:
        """窗口像素 → 原始图像像素。"""
        return self.cx + (x - self.W / 2.0) / self.scale, self.cy + (y - self.H / 2.0) / self.scale

    def orig_to_window(self, x: float, y: float) -> tuple[float, float]:
        """原始图像像素 → 窗口像素（用于在缩放图上画点/文字）。"""
        return (x - self.cx) * self.scale + self.W / 2.0, (y - self.cy) * self.scale + self.H / 2.0

    # ---- 视图控制 ----
    def _clamp_center(self) -> None:
        """限制视图中心，保证视口不越出图像边界（避免黑边/越界裁剪）。"""
        half_w = self.W / (2.0 * self.scale)
        half_h = self.H / (2.0 * self.scale)
        self.cx = min(max(self.cx, half_w), self.W - half_w)
        self.cy = min(max(self.cy, half_h), self.H - half_h)

    def zoom_at(self, x: float, y: float, factor: float) -> None:
        """以光标所在窗口位置 (x, y) 为中心缩放，缩放前后该点对应的原始坐标不动。"""
        new_scale = min(max(self.scale * factor, self.min_scale), self.max_scale)
        if abs(new_scale - self.scale) < 1e-9:
            return
        ox, oy = self.window_to_orig(x, y)
        self.scale = new_scale
        self.cx = ox - (x - self.W / 2.0) / self.scale
        self.cy = oy - (y - self.H / 2.0) / self.scale
        self._clamp_center()

    def pan_by(self, dx: float, dy: float) -> None:
        """按窗口像素增量平移（右键拖动）。"""
        self.cx -= dx / self.scale
        self.cy -= dy / self.scale
        self._clamp_center()

    def render(self, frame) -> "cv2.Mat":  # noqa: ANN001
        """把原始帧按当前视图裁剪+放大成窗口大小；未缩放时原样返回。"""
        if self.scale <= 1.0 + 1e-9:
            return frame.copy()
        half_w = self.W / (2.0 * self.scale)
        half_h = self.H / (2.0 * self.scale)
        x0 = int(round(self.cx - half_w))
        y0 = int(round(self.cy - half_h))
        x1 = x0 + int(round(self.W / self.scale))
        y1 = y0 + int(round(self.H / self.scale))
        x0, y0 = max(x0, 0), max(y0, 0)
        x1, y1 = min(x1, self.W), min(y1, self.H)
        roi = frame[y0:y1, x0:x1]
        return cv2.resize(roi, (self.W, self.H), interpolation=cv2.INTER_LINEAR)


def click_callback(event, x, y, flags, param):  # noqa: ANN001
    """滚轮缩放 / 左键选点 / 右键拖动平移。点坐标一律换算为原始图像坐标。"""
    view: ZoomPanView = param["view"]
    if event == cv2.EVENT_MOUSEWHEEL:
        direction = _wheel_direction(flags)
        if direction:
            view.zoom_at(x, y, view.step if direction > 0 else 1.0 / view.step)
    elif event == cv2.EVENT_LBUTTONDOWN:
        param["last_click"] = tuple(round(v) for v in view.window_to_orig(x, y))
        param["new_click"] = True
    elif event == cv2.EVENT_RBUTTONDOWN:
        param["dragging"] = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE and param["dragging"] is not None and (flags & cv2.EVENT_FLAG_RBUTTON):
        lx, ly = param["dragging"]
        view.pan_by(x - lx, y - ly)
        param["dragging"] = (x, y)
    elif event == cv2.EVENT_RBUTTONUP:
        param["dragging"] = None


def collect_point(cam, kin, controller, config, bx, by, z, dry_run: bool, idx: int, total: int) -> tuple | None:
    """【安全移动】到 (bx, by, z)（先抬升→高位水平→下降），用户点击活动爪尖点像素。

    返回 (像素, 尖点基座XY)。夹爪最大张开、中心被腕部遮挡时点活动爪尖点，
    用 half_span/jaw_side 把"网格中心 XY"换算成"活动爪尖的实际基座 XY"（自洽标定）。
    """
    if not dry_run:
        print(f"  [{(idx)}/{total}] 安全移动到基座 ({bx:.3f}, {by:.3f}) @ z={z:.3f} ...")
        safe_move_xyz(controller, kin, bx, by, z,
                      v_max=float(config["motion"]["grasp_v_max"]),
                      a_max=float(config["motion"]["grasp_a_max"]), settle_s=0.5)
        time.sleep(0.8)

    # 活动爪尖点的实际基座 XY = 网格中心 XY + 半宽偏移（沿机械臂局部 y，随 pan 旋转）
    g = config["gripper"]
    half, side = float(g["half_span"]), float(g["jaw_side"])
    pan = math.atan2(by - kin.y0, bx - kin.x0)
    tip_x = bx - side * half * math.sin(pan)
    tip_y = by + side * half * math.cos(pan)

    print("  请点击画面中【活动爪尖点】（夹爪最大张开时能看到的那片爪的尖端）")
    print("  滚轮放大/缩小，右键拖动平移，左键选点（记录的是未缩放坐标）。Enter 确认 / Esc 跳过")
    frame0 = cam.read_undistorted() if hasattr(cam, "read_undistorted") else cam.read()
    view = ZoomPanView(frame0.shape[1], frame0.shape[0])
    ctx = {"view": view, "last_click": None, "new_click": False, "dragging": None}
    cv2.namedWindow("calib")  # 先创建窗口再绑定鼠标回调（否则 setMouseCallback 报 NULL window）
    cv2.setMouseCallback("calib", click_callback, ctx)
    last_click = None
    while True:
        frame = cam.read_undistorted() if hasattr(cam, "read_undistorted") else cam.read()
        display = view.render(frame)
        cv2.putText(display, f"point {idx}/{total}  base=({bx:.3f},{by:.3f})  click jaw tip -> ENTER",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(display, f"zoom x{view.scale:.1f}  [wheel]zoom [R-drag]pan [L]pick",
                    (10, display.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
        if ctx["new_click"]:
            last_click = ctx["last_click"]
            ctx["new_click"] = False
        if last_click:
            wx, wy = view.orig_to_window(*last_click)
            cv2.circle(display, (int(wx), int(wy)), 6, (0, 0, 255), -1)
            cv2.putText(display, f"({last_click[0]},{last_click[1]})", (int(wx) + 10, int(wy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.imshow("calib", display)
        key = cv2.waitKey(30) & 0xFF
        if key in (13, 10) and last_click:  # ENTER
            return (last_click, (tip_x, tip_y))  # 记录"尖点像素 ↔ 尖点基座XY"（自洽）
        if key == 27:  # ESC 跳过
            return None


def main(argv=None) -> int:
    args = parse_args(argv)
    config = load_d4_config()
    if args.port:
        config["hardware"]["port"] = args.port
    nx, ny = (int(v) for v in args.grid.lower().split("x"))

    calib = load_table_calib() or TableCalibration()
    if args.only_solve:
        if calib.n_points() < 4:
            print("!! 已存标定点不足，请先完整跑一遍标定")
            return 1
        err = calib.solve()
        print(f"重投影误差: {err}")
        print(f"已有标定点: {list(zip(calib.base_pts, calib.pixel_pts, strict=True))}")
        return 0
    # 完整标定：从零开始（避免旧标定点与新点混叠导致单应失真）
    if DEFAULT_TABLE_CALIB.exists():
        print(f"!! 检测到旧标定文件 {DEFAULT_TABLE_CALIB}，本次将覆盖（从零标定 9 点）")
    calib = TableCalibration()

    kin = make_kinematics(config)
    cam = make_camera(config, camera_index=args.camera)
    z = args.z if args.z is not None else float(config["table"]["z_touch"])

    grid = grid_xy(
        config["table"]["grid"]["x_start"], config["table"]["grid"]["x_stop"], nx,
        config["table"]["grid"]["y_start"], config["table"]["grid"]["y_stop"], ny,
    )
    print(f"标定网格: {nx}x{ny}，共 {len(grid)} 点（z={z:.3f}m）")

    bus = None
    controller = None
    if not args.dry_run:
        bus = make_bus(config)
        controller = make_controller(config, bus)
        print(f"已连接 {config['hardware']['port']}")

    try:
        for i, (bx, by) in enumerate(grid):
            pt = collect_point(cam, kin, controller, config, bx, by, z, args.dry_run, i + 1, len(grid))
            if pt is not None:
                calib.add_point(*pt)
                print(f"  记录点 {calib.n_points()}: 像素{pt[0]} ↔ 基座{pt[1]}")
        cv2.destroyAllWindows()

        if calib.n_points() < 4:
            print("!! 有效标定点不足 4，未求解")
            return 1
        err = calib.solve()
        print(f"\n求解完成，重投影误差: rms_mm={err['rms_mm']:.2f}  rms_px={err['rms_px']:.2f}")
        calib.save(args.out)
        print(f"已保存 → {args.out}")
        print("下一步：python d4_grasp_demo.py --trials 5")
    finally:
        if bus:
            bus.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
