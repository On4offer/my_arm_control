# -*- coding: utf-8 -*-
"""
D4 目标检测：HSV 颜色分割 + 轮廓筛选（面积/圆度）→ 目标中心像素。

典型流程：
    det = ColorTargetDetector(hsv=[H_lo,H_hi,S_lo,S_hi,V_lo,V_hi], area_range=(800, 120000))
    target = det.detect(frame)   # Target(x_px, y_px, area_px, bbox) or None
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Target:
    """检测到的目标：中心像素坐标 + 面积 + 外接框。

    box: 旋转矩形（minAreaRect）的 4 个角点（Nx2 float），贴合倾斜目标；
         None 表示用轴对齐 bbox。
    angle: 长边方位角（图像系，度，x 轴为 0；方向感知抓取用），None 表示无。
    """

    x: int
    y: int
    area: float
    bbox: tuple[int, int, int, int]  # x, y, w, h（轴对齐，像素）
    box: np.ndarray | None = None  # 旋转矩形角点
    angle: float | None = None  # 长边方位角（图像系，度）

    @property
    def cx(self) -> int:
        return self.x

    @property
    def cy(self) -> int:
        return self.y


def _long_axis_angle(box: np.ndarray) -> float:
    """旋转矩形长边方位角（图像系，度）。长边无方向，范围 [-90, 90)。"""
    p = box.astype(np.float64)
    d1 = p[1] - p[0]
    d2 = p[2] - p[1]
    long_vec = d1 if np.linalg.norm(d1) >= np.linalg.norm(d2) else d2
    return float(math.degrees(math.atan2(long_vec[1], long_vec[0])))


def _pca_long_axis_angle(contour) -> float:
    """轮廓主方向（PCA 第一主轴）作为长边角（图像系，度）。

    对"矩形 + 侧立面/噪点"的轮廓比 minAreaRect 抗偏：主方向由全部轮廓点
    的方差决定，不受少数极值点带偏（D4 实测 minAreaRect 可偏几十度）。
    长边无方向，范围 [-90, 90)。
    """
    pts = contour.reshape(-1, 2).astype(np.float64)
    mean = pts.mean(axis=0)
    cov = (pts - mean).T @ (pts - mean) / len(pts)
    eigvals, eigvecs = np.linalg.eigh(cov)
    v = eigvecs[:, int(np.argmax(eigvals))]
    return float(math.degrees(math.atan2(v[1], v[0])))


class ColorTargetDetector:
    """颜色目标检测器（HSV 分割 + 面积/圆度过滤）。"""

    def __init__(
        self,
        hsv: list[int] | tuple[int, int, int, int, int, int],
        area_range: tuple[int, int] = (600, 120000),
        max_targets: int = 1,
        morph_open: int = 3,
    ):
        self.hsv_min = np.array([hsv[0], hsv[2], hsv[4]], np.uint8)
        self.hsv_max = np.array([hsv[1], hsv[3], hsv[5]], np.uint8)
        self.area_range = area_range
        self.max_targets = max_targets
        self.morph_open = morph_open
        self._hsv_hist: tuple[np.ndarray, np.ndarray] | None = None  # 调试用

    def mask(self, frame: np.ndarray) -> np.ndarray:
        """HSV 阈值掩膜。H 通道注意红色在 0/180 环绕的情况（调用方应合并两份掩膜）。"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower, upper = self.hsv_min.copy(), self.hsv_max.copy()
        if lower[0] > upper[0]:
            # H 跨越 0°：拆成两段（0~upper 和 lower~179）
            mask = cv2.inRange(hsv, np.array([0, lower[1], lower[2]], np.uint8), np.array([upper[0], upper[1], upper[2]], np.uint8))
            mask |= cv2.inRange(hsv, np.array([lower[0], lower[1], lower[2]], np.uint8), np.array([179, upper[1], upper[2]], np.uint8))
        else:
            mask = cv2.inRange(hsv, lower, upper)
        if self.morph_open > 0:
            kernel = np.ones((self.morph_open, self.morph_open), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def detect(self, frame: np.ndarray) -> list[Target]:
        """返回按面积降序的目标列表（最多 max_targets）。"""
        mask = self.mask(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        targets: list[Target] = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.area_range[0] or area > self.area_range[1]:
                continue
            moments = cv2.moments(c)
            if moments["m00"] == 0:
                continue
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            x, y, w, h = cv2.boundingRect(c)
            # 旋转矩形：贴合倾斜目标（小目标 minAreaRect 噪声大，仅当面积够大时使用）
            box = None
            angle = None
            if area >= 400:
                # 用凸包再拟合：相机略偏正上方时盒子侧立面会被分割进来，
                # 轮廓非干净矩形会把 minAreaRect 长边带偏几十度（D4 实测）。
                hull = cv2.convexHull(c)
                rot = cv2.minAreaRect(hull)
                box = cv2.boxPoints(rot)
                angle = _pca_long_axis_angle(c)  # 长边角用 PCA 主方向（抗噪）
                lw, lh = rot[1]
                aspect = max(lw, lh) / max(min(lw, lh), 1e-6)
                if aspect < 1.2:  # 近方形：长边方向不可靠 → 不做方向感知
                    angle = None
            targets.append(Target(x=cx, y=cy, area=float(area), bbox=(x, y, w, h), box=box, angle=angle))
        targets.sort(key=lambda t: t.area, reverse=True)
        return targets[: self.max_targets]

    def detect_one(self, frame: np.ndarray) -> Target | None:
        """只取最大目标；没有返回 None。"""
        targets = self.detect(frame)
        return targets[0] if targets else None

    def draw(self, frame: np.ndarray, targets: list[Target]) -> np.ndarray:
        """在画面绘制目标（优先旋转矩形贴合倾斜目标）与外接框（调试/录视频用）。"""
        out = frame.copy()
        for t in targets:
            if t.box is not None:
                pts = t.box.astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(out, [pts], True, (0, 255, 0), 2)
            else:
                x, y, w, h = t.bbox
                cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(out, (t.x, t.y), 5, (0, 0, 255), -1)
            cv2.putText(out, f"{t.area:.0f}", (t.bbox[0], max(t.bbox[1] - 8, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return out
