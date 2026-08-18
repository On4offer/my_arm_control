#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 Demo（可选）：相机内参标定（棋盘格）→ 保存 config/d4_intrinsics.json
=========================================================================

用于图像去畸变，提升目标检测像素精度（不标定也可抓取，标定更稳）。

用法：
  # 方式一：从已有图片文件夹标定（每张需完整拍到棋盘格，建议 ≥10 张、多角度多距离）
  python d4_calibrate_intrinsics.py --source D:/calib_imgs

  # 方式二：实时拍摄标定（空格=抓取当前帧，c=清空，Enter=完成）
  python d4_calibrate_intrinsics.py --capture --pattern 9x6 --square-mm 25

说明：
  - 棋盘格（黑白格）打印后贴在硬纸板上，避免弯曲
  - 标定结果：camera_matrix / dist_coeffs / rms（重投影误差，越小越好，<0.5 为佳）
"""

import argparse
import sys
from pathlib import Path

# 把项目根目录加入 sys.path（demos 目录直接运行时可导入 my_arm_control）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2  # noqa: E402

from d4_common import DEFAULT_INTRINSICS, load_d4_config  # noqa: E402
from my_arm_control.vision import CameraView, calibrate_intrinsics  # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D4：相机内参标定（棋盘格）")
    p.add_argument("--source", type=str, default=None, help="棋盘格图片文件夹；不填则用 --capture 实时拍摄")
    p.add_argument("--capture", action="store_true", help="实时拍摄模式")
    p.add_argument("--pattern", type=str, default="9x6", help="棋盘格内角点数 列x行（默认 9x6）")
    p.add_argument("--square-mm", type=float, default=25.0, help="棋盘格单格边长毫米（默认 25）")
    p.add_argument("--out", type=str, default=str(DEFAULT_INTRINSICS), help="输出 JSON 路径")
    return p.parse_args(argv)


def capture_images(config: dict, pattern: tuple[int, int]) -> list:
    """实时拍摄棋盘格：空格抓帧，c 清空，Enter 完成。"""
    cam = CameraView(int(config["hardware"]["camera_index"]))
    images = []
    print("实时标定：把棋盘格完整放入画面（多角度/多距离）→ 空格抓帧；c=清空；Enter=完成；Esc=取消")
    while True:
        frame = cam.read()
        display = frame.copy()
        cv2.putText(display, f"captured={len(images)}  SPACE=grab  C=clear  ENTER=done", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern, None)
        if found:
            cv2.drawChessboardCorners(display, pattern, corners, found)
        cv2.imshow("calibrate", display)
        key = cv2.waitKey(30) & 0xFF
        if key == 32:  # SPACE
            if found:
                images.append(frame.copy())
                print(f"  抓取第 {len(images)} 张（检测到棋盘格）")
            else:
                print("  !! 未检测到棋盘格，请调整角度/距离/光照")
        elif key == ord("c"):
            images.clear()
            print("  已清空")
        elif key in (13, 10):  # ENTER
            break
        elif key == 27:
            images.clear()
            break
    cv2.destroyAllWindows()
    cam.cap.release()
    return images


def main(argv=None) -> int:
    args = parse_args(argv)
    config = load_d4_config()
    nx, ny = (int(v) for v in args.pattern.lower().split("x"))
    pattern = (nx, ny)

    if args.source:
        files = sorted(Path(args.source).glob("*.jpg")) + sorted(Path(args.source).glob("*.png"))
        if not files:
            print(f"!! {args.source} 下没有 jpg/png 图片")
            return 1
        images = [cv2.imread(str(f)) for f in files]
        print(f"载入 {len(images)} 张图片（{args.source}）")
    elif args.capture:
        images = capture_images(config, pattern)
    else:
        print("用法：--source <图片文件夹> 或 --capture（实时拍摄）")
        return 1

    if len(images) < 3:
        print(f"!! 有效图片不足（{len(images)}/3）")
        return 1

    try:
        result = calibrate_intrinsics(images, pattern, args.square_mm)
    except ValueError as e:
        print(f"!! {e}")
        return 1

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        import json

        json.dump(result, f, ensure_ascii=False, indent=2)

    print("\n标定结果：")
    print(f"  有效图片: {result['n_used']}")
    print(f"  RMS 重投影误差: {result['rms']:.4f} px（<0.5 为佳）")
    print(f"  camera_matrix:\n{result['camera_matrix']}")
    print(f"  已保存 → {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
