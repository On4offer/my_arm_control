#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 一键 HSV 自动标定：框选目标 → 自动统计 HSV 范围 → 写入 d4_config.json → 实时验证
===============================================================================

解决手动滑杆"黑盒"问题：不需要理解 HSV 参数，只需要在画面里框住目标物体。

两种框选模式（--mode）：
  quad（默认）：鼠标【左键点击】物体的 4 个角点（顺序随意，自动凸包排序）
               → 只分析四边形内像素，倾斜摆放的目标也能精准采样
  rect（旧）：鼠标【按住左键拖框】框住目标

操作（quad 模式）：
  1. 摆好目标物体（颜色单一、≥4cm、能被夹爪握住）
  2. 运行本脚本，依次点击目标物体的 4 个角（画面会画出连线）
  3. 按 Enter → 自动分析 → 写入 d4_config.json → 实时显示检测效果
  4. 绿色框稳稳定住目标 → 按 q 退出；框偏了 → 按 r 重选
  5. 完成后直接跑 python demos\\d4_calibrate_table.py

原理：对选中区域做 HSV 直方图，取主峰两侧覆盖 95% 的区间（H 通道处理
0/179 环绕，S/V 用 2%~98% 百分位数并留余量），对少量背景干扰鲁棒。
"""

import argparse
import json
import sys
from pathlib import Path

# 把项目根目录加入 sys.path（demos 目录直接运行时可导入 my_arm_control）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from d4_common import D4_CONFIG, load_d4_config, make_camera  # noqa: E402
from my_arm_control.detect import ColorTargetDetector  # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D4：一键 HSV 自动标定（框选目标）")
    p.add_argument("--mode", choices=["quad", "rect"], default="quad",
                   help="框选方式：quad=点击物体4角点（默认，支持倾斜目标）；rect=拖框")
    p.add_argument("--camera", type=int, default=None, help="相机索引（默认取 d4_config.json）")
    p.add_argument("--config", type=str, default=str(D4_CONFIG), help="写入的配置文件路径")
    return p.parse_args(argv)


def wrap_h_range(vals: np.ndarray, coverage: float = 0.95) -> tuple[int, int]:
    """H 通道环形直方图：从主峰向两侧扩展直到覆盖 coverage 比例。

    返回 (lo, hi)：hi 可为 >179（表示跨 0° 环绕，实际用 [lo, hi-180] 触发环绕分支）。
    """
    hist = np.bincount(vals, minlength=180).astype(float)
    total = hist.sum()
    if total == 0:
        return 0, 179
    peak = int(np.argmax(hist))
    lo = hi = peak
    cum = hist[peak]
    while cum < total * coverage and (hi - lo) < 179:
        left = hist[(lo - 1) % 180]
        right = hist[(hi + 1) % 180]
        if right >= left:
            hi = (hi + 1) % 180
            cum += right
        else:
            lo = (lo - 1) % 180
            cum += left
    if lo > hi:
        hi += 180  # 跨 0° 环绕
    return lo, hi


def _finalize_hsv(h_lo: int, h_hi: int, s_vals: np.ndarray, v_vals: np.ndarray) -> list[int]:
    """H 最小宽度 + S/V 余量，输出 [H_min,H_max,S_min,S_max,V_min,V_max]。"""
    min_span = 8
    if h_hi >= 180:  # 环绕
        hb = h_hi - 180
        span = (179 - h_lo) + hb + 1
        while span < min_span and span < 180:
            if hb > 0:
                hb -= 1
                span += 1
            elif h_lo < 179:
                h_lo += 1
                span += 1
            else:
                break
        h_lo, h_hi = h_lo, hb
    elif h_hi - h_lo < min_span:
        ext = (min_span - (h_hi - h_lo)) // 2
        h_lo = max(0, h_lo - ext)
        h_hi = min(179, h_hi + ext)

    s_lo, s_hi = np.percentile(s_vals, [2, 98])
    v_lo, v_hi = np.percentile(v_vals, [2, 98])
    s_lo = max(20.0, s_lo - 15.0)
    s_hi = min(255.0, s_hi + 15.0)
    v_lo = max(20.0, v_lo - 15.0)
    v_hi = min(255.0, v_hi + 15.0)
    return [int(round(v)) for v in (h_lo, h_hi, s_lo, s_hi, v_lo, v_hi)]


def auto_hsv_from_mask(frame: np.ndarray, mask: np.ndarray) -> list[int]:
    """只统计二值 mask 内的像素，自动推断 HSV 范围。

    过滤低饱和度/极低亮度像素（白色字体、黑色阴影等无彩色成分），
    使主色统计更纯（适用于"蓝底白字"这类目标）。
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    vals = hsv[mask > 0]
    # 无彩色成分：S<25（白/灰）或 V<25（黑/暗影）
    sel = (vals[:, 1] >= 25) & (vals[:, 2] >= 25)
    vals = vals[sel]
    if len(vals) < 30:
        raise ValueError("选中区域颜色过杂/过灰（无主色），请尽量框住纯色区域")
    return _finalize_hsv(*wrap_h_range(vals[:, 0]), vals[:, 1], vals[:, 2])


def auto_hsv_from_quad(frame: np.ndarray, pts: list[tuple[int, int]]) -> list[int]:
    """四角点 → 凸包多边形 mask → 分析。点顺序任意。"""
    if len(pts) != 4:
        raise ValueError(f"需要 4 个角点，当前 {len(pts)} 个")
    mask = np.zeros(frame.shape[:2], np.uint8)
    hull = cv2.convexHull(np.array(pts, np.int32).reshape(-1, 1, 2))
    cv2.fillConvexPoly(mask, hull, 255)
    return auto_hsv_from_mask(frame, mask)


def auto_hsv_from_roi(frame: np.ndarray, roi: tuple[int, int, int, int]) -> list[int]:
    """轴对齐矩形 ROI（rect 模式，保留兼容）。"""
    x, y, w, h = roi
    x, y, w, h = max(x, 0), max(y, 0), max(w, 1), max(h, 1)
    x = min(x, frame.shape[1] - 1)
    y = min(y, frame.shape[0] - 1)
    w = min(w, frame.shape[1] - x)
    h = min(h, frame.shape[0] - y)
    roi_img = frame[y : y + h, x : x + w]
    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
    return _finalize_hsv(*wrap_h_range(hsv[:, :, 0].ravel()),
                         hsv[:, :, 1].ravel(), hsv[:, :, 2].ravel())


def main(argv=None) -> int:
    args = parse_args(argv)
    config = load_d4_config(args.config)
    if args.camera is not None:
        config["hardware"]["camera_index"] = args.camera
    cam = make_camera(config)

    win = "d4_auto_hsv"
    cv2.namedWindow(win)
    state = {"mode": args.mode, "pts": [], "roi0": None, "roi": None, "hsv": None}

    def on_mouse(event, x, y, flags, param):  # noqa: ANN001
        if param["mode"] == "quad":
            if event == cv2.EVENT_LBUTTONDOWN and len(param["pts"]) < 4:
                param["pts"].append((x, y))
        else:  # rect：拖框
            if event == cv2.EVENT_LBUTTONDOWN:
                param["roi0"] = (x, y)
            elif event == cv2.EVENT_LBUTTONUP and param["roi0"]:
                x0, y0 = param["roi0"]
                param["roi"] = (min(x0, x), min(y0, y), abs(x - x0), abs(y - y0))
                param["roi0"] = None

    cv2.setMouseCallback(win, on_mouse, state)
    if args.mode == "quad":
        print("操作：依次【点击】目标物体的 4 个角点（顺序随意）→ Enter 分析；r=重选；q=退出")
    else:
        print("操作：按住左键拖框框住目标 → 松开 → Enter 分析；r=重选；q=退出")

    result_hsv = None
    while True:
        frame = cam.read()
        display = frame.copy()

        if state["mode"] == "quad":
            pts = state["pts"]
            if len(pts) >= 2:
                arr = np.array(pts, np.int32).reshape(-1, 1, 2)
                cv2.polylines(display, [arr], isClosed=True, color=(255, 0, 0), thickness=2)
            for i, (px, py) in enumerate(pts):
                cv2.circle(display, (px, py), 4, (0, 0, 255), -1)
                cv2.putText(display, str(i + 1), (px + 6, py - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if len(pts) == 4:
                over = display.copy()
                hull = cv2.convexHull(np.array(pts, np.int32).reshape(-1, 1, 2))
                cv2.fillConvexPoly(over, hull, (255, 0, 0))
                display = cv2.addWeighted(over, 0.25, display, 0.75, 0)
                cv2.putText(display, "4 点已选：按 Enter 分析 / r 重选", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            else:
                cv2.putText(display, f"点击第 {len(pts) + 1}/4 个角点（顺序随意）", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            if state["hsv"]:
                det = ColorTargetDetector(hsv=state["hsv"])
                display = det.draw(display, det.detect(frame))
                cv2.putText(display, f"HSV={state['hsv']}  r=重选 q=退出", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("mask", det.mask(frame))
        else:  # rect
            if state["roi"]:
                x, y, w, h = state["roi"]
                cv2.rectangle(display, (x, y), (x + w, y + h), (255, 0, 0), 2)
                if state["hsv"]:
                    det = ColorTargetDetector(hsv=state["hsv"])
                    display = det.draw(display, det.detect(frame))
                    cv2.putText(display, f"HSV={state['hsv']}  r=重选 q=退出", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.imshow("mask", det.mask(frame))
                else:
                    cv2.putText(display, "按 Enter 分析 / r 重框", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            else:
                cv2.putText(display, "按住左键拖框框住目标物体 ...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.imshow(win, display)

        key = cv2.waitKey(30) & 0xFF
        if key in (13, 10):  # ENTER：分析
            try:
                if state["mode"] == "quad":
                    if len(state["pts"]) == 4:
                        result_hsv = auto_hsv_from_quad(frame, state["pts"])
                        state["hsv"] = result_hsv
                        print(f"自动分析 HSV={result_hsv}")
                    else:
                        print(f"还需点击 {4 - len(state['pts'])} 个角点")
                elif state["roi"]:
                    result_hsv = auto_hsv_from_roi(frame, state["roi"])
                    state["hsv"] = result_hsv
                    print(f"自动分析 HSV={result_hsv}")
            except ValueError as e:
                print(f"分析失败: {e}")
        elif key == ord("r"):
            state["pts"].clear()
            state["roi"] = None
            state["hsv"] = None
            try:
                cv2.destroyWindow("mask")
            except cv2.error:
                pass
        elif key == ord("q"):
            break
    cv2.destroyAllWindows()

    if result_hsv is None:
        print("未完成框选/分析，未写入配置")
        return 1

    cfg_path = Path(args.config)
    cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
    cfg["target_detect"]["hsv"] = result_hsv
    cfg_path.write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"已写入 {cfg_path}: target_detect.hsv={result_hsv}")
    print("下一步：python demos\\d4_calibrate_table.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
