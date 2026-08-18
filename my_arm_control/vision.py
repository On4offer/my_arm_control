# -*- coding: utf-8 -*-
"""
D4 视觉层：相机封装（OpenCV VideoCapture）+ 相机内参标定（棋盘格）。

用法：
    cam = CameraView(index=0, width=1280, height=720)
    frame = cam.read()
    cam.save_snapshot("shot.jpg", frame)

内参标定（可选，用于去畸变提升检测精度）：
    calibrate_intrinsics(images, pattern_size=(9,6), square_mm=25)
    → {"camera_matrix":..., "dist_coeffs":..., "rms":...}
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

# Windows MSMF 硬件变换会与 OpenCV 冲突（LeRobot camera_opencv.py 同样处理）
try:
    import os

    if os.name == "nt" and "OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS" not in os.environ:
        os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
except Exception:  # pragma: no cover
    pass


class CameraError(Exception):
    """相机错误（打开失败 / 读帧失败）。"""


def scan_cameras(max_index: int = 8) -> list[dict]:
    """探测可用相机，返回 [{index, width, height, backend}]（Windows 用 DSHOW 后端）。"""
    found = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        try:
            if cap.isOpened():
                found.append({
                    "index": i,
                    "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                    "backend": cap.getBackendName(),
                })
        finally:
            cap.release()
    return found


def find_cameras(max_index: int = 8) -> list[int]:
    """兼容旧接口：返回可用相机 index 列表。"""
    return [c["index"] for c in scan_cameras(max_index)]


def pick_camera(preferred: int, prefer_width: int = 1280) -> int:
    """选择相机：优先用配置的 index；不可用时自动选分辨率最接近 prefer_width 的可用相机。

    Windows 上 USB 相机换插口后 index 会变，自动 fallback 避免硬编码 index 失效。
    """
    cams = scan_cameras()
    if not cams:
        raise CameraError("未检测到任何可用相机（检查 USB 连接/驱动）")
    indexes = [c["index"] for c in cams]
    if preferred in indexes:
        return preferred
    cams.sort(key=lambda c: abs(c["width"] - prefer_width))
    best = cams[0]
    print(f"[相机] 配置 index {preferred} 不可用（可用 {indexes}），自动选择 index {best['index']} "
          f"({best['width']}x{best['height']})。若不对请用 --camera 指定。")
    return best["index"]


class CameraView:
    """基于 OpenCV 的相机封装：打开/读帧/快照/去畸变。"""

    def __init__(self, index: int = 0, width: int | None = None, height: int | None = None):
        self.index = index
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise CameraError(f"相机 {index} 打开失败。可用: {find_cameras()}")
        if width and height:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # 读一帧确认可用
        ret, frame = self.cap.read()
        if not ret or frame is None:
            raise CameraError(f"相机 {index} 读帧失败")
        self.width = frame.shape[1]
        self.height = frame.shape[0]
        self._intrinsics: dict[str, Any] | None = None

    def __del__(self):
        try:
            self.cap.release()
        except Exception:
            pass

    def release(self) -> None:
        """释放相机（显式调用，替代依赖 __del__）。"""
        try:
            self.cap.release()
        except Exception:
            pass

    def read(self) -> np.ndarray:
        ret, frame = self.cap.read()
        if not ret or frame is None:
            raise CameraError(f"相机 {self.index} 读帧失败")
        return frame

    def read_undistorted(self) -> np.ndarray:
        """读帧并去畸变（若已加载内参）。"""
        frame = self.read()
        if self._intrinsics:
            mtx = np.array(self._intrinsics["camera_matrix"], dtype=np.float64)
            dist = np.array(self._intrinsics["dist_coeffs"], dtype=np.float64)
            h, w = frame.shape[:2]
            new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0, (w, h))
            return cv2.undistort(frame, mtx, dist, None, new_mtx)
        return frame

    def load_intrinsics(self, json_path: str | Path) -> None:
        with open(json_path, "r", encoding="utf-8") as f:
            self._intrinsics = json.load(f)

    def save_snapshot(self, path: str | Path, frame: np.ndarray | None = None) -> str:
        if frame is None:
            frame = self.read()
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), frame)
        return str(path)


# ---- 相机内参标定（棋盘格） ----
def calibrate_intrinsics(
    images: list[np.ndarray],
    pattern_size: tuple[int, int] = (9, 6),
    square_mm: float = 25.0,
) -> dict[str, Any]:
    """用棋盘格图片标定相机内参。

    Args:
        images: 不同视角/距离的棋盘格 BGR 图像（建议 ≥10 张，棋盘格完全在画面内）。
        pattern_size: 内角点数 (列, 行)，默认 9×6。
        square_mm: 棋盘格单个方格边长（毫米）。

    Returns:
        {"camera_matrix", "dist_coeffs", "rms", "n_used"}
    """
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : pattern_size[0], 0 : pattern_size[1]].T.reshape(-1, 2) * square_mm

    obj_points, img_points = [], []
    used = 0
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if not found:
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        obj_points.append(objp)
        img_points.append(corners)
        used += 1
    if used < 3:
        raise ValueError(f"有效棋盘格图片不足（{used}/3），请拍摄更清晰完整的棋盘格")

    h, w = images[0].shape[:2]
    rms, mtx, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
    return {
        "camera_matrix": mtx.tolist(),
        "dist_coeffs": dist.tolist(),
        "rms": float(rms),
        "n_used": used,
    }
