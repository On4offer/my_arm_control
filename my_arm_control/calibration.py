# -*- coding: utf-8 -*-
"""
D4 手眼标定（eye-to-hand 平面单应）：像素坐标 ↔ 机械臂基座 XY。

原理：相机固定在工作台上方（eye-to-hand），目标与机械臂末端都工作在同一
"桌面平面"上。把机械臂末端（指尖）移到桌面上 N(≥4) 个已知基座坐标的网格点，
记录每个点对应的像素坐标 → 求解单应矩阵 H（像素→基座 XY）：
    [x_base, y_base, 1]ᵀ ≈ H · [u, v, 1]ᵀ     （齐次坐标）
H 用 `cv2.findHomography`（RANSAC）求解。抓取时：目标像素 p → H → 基座 XY → IK。

相比通用 AX=XB 手眼标定：本场景机械臂只在桌面平面上移动（z 固定、夹爪竖直），
2D 单应已完整刻画 像素→基座 的映射，且把"相机位姿 + 相机内参 + 工作台高度 +
末端安装偏移"一次性隐式标定——工业 2D 视觉抓取的标准做法，无需测距/量角。
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np


class CalibrationError(Exception):
    """标定错误（点数不足 / 退化配置）。"""


def grid_xy(
    x_start: float, x_stop: float, nx: int, y_start: float, y_stop: float, ny: int
) -> list[tuple[float, float]]:
    """生成基座 XY 标定网格点（米）。"""
    xs = np.linspace(x_start, x_stop, nx)
    ys = np.linspace(y_start, y_stop, ny)
    return [(float(x), float(y)) for x in xs for y in ys]


class TableCalibration:
    """像素 ↔ 基座 XY 平面单应标定（eye-to-hand，2D 抓取）。"""

    def __init__(self):
        self.pixel_pts: list[tuple[float, float]] = []  # (u, v)
        self.base_pts: list[tuple[float, float]] = []  # (x, y) 米
        self.H: np.ndarray | None = None  # 像素 → 基座 单应矩阵（3x3）

    # ---- 数据收集 ----
    def add_point(self, pixel: tuple[float, float], base: tuple[float, float]) -> None:
        self.pixel_pts.append((float(pixel[0]), float(pixel[1])))
        self.base_pts.append((float(base[0]), float(base[1])))

    def n_points(self) -> int:
        return len(self.pixel_pts)

    # ---- 求解 ----
    def solve(self, method: int = cv2.RANSAC) -> dict[str, float]:
        """求解单应矩阵，返回重投影误差（基座系毫米）。至少 4 点。"""
        n = len(self.pixel_pts)
        if n < 4:
            raise CalibrationError(f"标定点不足（{n}/4）：需要至少 4 组 像素↔基座 对应点")
        src = np.array(self.pixel_pts, dtype=np.float32).reshape(-1, 1, 2)
        dst = np.array(self.base_pts, dtype=np.float32).reshape(-1, 1, 2)
        H, _ = cv2.findHomography(src, dst, method, 5.0)
        if H is None:
            raise CalibrationError("单应矩阵求解失败（点共线或退化配置）")
        self.H = H
        return self.reprojection_error()

    def reprojection_error(self) -> dict[str, float]:
        """重投影误差：像素 → 基座 XY 后与实际基座点的 RMS 偏差（毫米）。"""
        if self.H is None or not self.pixel_pts:
            return {"rms_px": float("nan"), "rms_mm": float("nan")}
        errs_px: list[float] = []
        errs_mm: list[float] = []
        for (u, v), (bx, by) in zip(self.pixel_pts, self.base_pts, strict=True):
            px, py = self.pixel_to_base(u, v)
            errs_mm.append(math.hypot(px - bx, py - by) * 1000.0)
            # 反投影到像素比较
            bu, bv = self.base_to_pixel(bx, by)
            errs_px.append(math.hypot(bu - u, bv - v))
        return {"rms_px": float(np.sqrt(np.mean(np.square(errs_px)))),
                "rms_mm": float(np.sqrt(np.mean(np.square(errs_mm))))}

    # ---- 变换 ----
    def pixel_to_base(self, u: float, v: float) -> tuple[float, float]:
        """像素 (u, v) → 基座 XY（米）。"""
        if self.H is None:
            raise CalibrationError("未求解：先调用 solve()")
        p = self.H @ np.array([u, v, 1.0])
        return float(p[0] / p[2]), float(p[1] / p[2])

    def base_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        """基座 XY（米）→ 像素 (u, v)。"""
        if self.H is None:
            raise CalibrationError("未求解：先调用 solve()")
        Hi = np.linalg.inv(self.H)
        p = Hi @ np.array([x, y, 1.0])
        return float(p[0] / p[2]), float(p[1] / p[2])

    # ---- 持久化 ----
    def save(self, path: str | Path) -> None:
        if self.H is None:
            raise CalibrationError("未求解，无法保存")
        data = {
            "type": "table_homography_pixel_to_base",
            "H_pixel_to_base": self.H.tolist(),
            "pixel_pts": self.pixel_pts,
            "base_pts": self.base_pts,
            "reprojection": self.reprojection_error(),
        }
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    @classmethod
    def load(cls, path: str | Path) -> "TableCalibration":
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        obj = cls()
        obj.H = np.array(data["H_pixel_to_base"], dtype=np.float64)
        obj.pixel_pts = [tuple(p) for p in data["pixel_pts"]]
        obj.base_pts = [tuple(p) for p in data["base_pts"]]
        return obj

    # ---- 工具 ----
    def draw_points(self, frame: np.ndarray, radius: int = 6) -> np.ndarray:
        """在画面绘制标定点（像素），调试用。"""
        out = frame.copy()
        for i, (u, v) in enumerate(self.pixel_pts):
            cv2.circle(out, (int(u), int(v)), radius, (0, 200, 255), -1)
            cv2.putText(out, str(i), (int(u) + 8, int(v) + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        return out
