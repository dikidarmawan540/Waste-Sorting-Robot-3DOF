"""
homography.py
=============
Transformasi pixel kamera ke koordinat kerja dalam mm.

Versi ini dibuat untuk alur paling sederhana:
    pixel kamera -> X_robot_mm, Y_robot_mm -> kirim ke ESP32

Tidak ada inverse kinematics di Python. ESP32/firmware yang menghitung IK.
"""

from __future__ import annotations

import json
import os
from typing import Iterable, Tuple

import cv2
import numpy as np

import config

_DEFAULT_PATH = getattr(
    config,
    "HOMOGRAPHY_PATH",
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "homography_robot.json"),
)

_H = None
_H_INV = None
_SRC_POINTS = None
_DST_POINTS_MM = None
_COORDINATE_SPACE = "robot"
_loaded = False


def load(path: str | None = None) -> None:
    """Muat homography JSON hasil Calibration_Homography.py."""
    global _H, _H_INV, _SRC_POINTS, _DST_POINTS_MM, _COORDINATE_SPACE, _loaded

    path = path or _DEFAULT_PATH
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"[homography] File tidak ditemukan: {path}\n"
            "Jalankan src/Calibration_Homography.py terlebih dahulu."
        )

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    _H = np.array(data["H"], dtype=np.float64)
    _H_INV = np.linalg.inv(_H)
    _SRC_POINTS = np.array(data.get("src_points", []), dtype=np.float64)
    _DST_POINTS_MM = np.array(data.get("dst_points_mm", []), dtype=np.float64)
    _COORDINATE_SPACE = data.get("coordinate_space", getattr(config, "HOMOGRAPHY_MODE", "robot"))
    _loaded = True

    print(f"[homography] Loaded: {path}")
    print(f"[homography] Space : {_COORDINATE_SPACE}")
    if _DST_POINTS_MM.size:
        xs = _DST_POINTS_MM[:, 0]
        ys = _DST_POINTS_MM[:, 1]
        print(f"[homography] Range : X {xs.min():.1f}..{xs.max():.1f} mm | Y {ys.min():.1f}..{ys.max():.1f} mm")


def initialize(path: str | None = None) -> None:
    load(path)


def _check_loaded() -> None:
    if not _loaded:
        raise RuntimeError("[homography] Belum load. Panggil homography.initialize() dulu.")


def pixel_to_mm(px: float, py: float) -> tuple[float, float]:
    """Konversi 1 titik pixel ke koordinat mm sesuai file kalibrasi."""
    _check_loaded()
    pt = np.array([[[float(px), float(py)]]], dtype=np.float64)
    out = cv2.perspectiveTransform(pt, _H)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def pixel_to_robot(px: float, py: float) -> tuple[float, float]:
    """
    Alias eksplisit untuk versi robot-base.
    Return langsung X_robot_mm, Y_robot_mm.
    """
    return pixel_to_mm(px, py)


def mm_to_pixel(x_mm: float, y_mm: float) -> tuple[float, float]:
    """Konversi koordinat mm kembali ke pixel untuk debug overlay."""
    _check_loaded()
    pt = np.array([[[float(x_mm), float(y_mm)]]], dtype=np.float64)
    out = cv2.perspectiveTransform(pt, _H_INV)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def pixels_to_mm(points_px: np.ndarray) -> np.ndarray:
    _check_loaded()
    pts = np.array(points_px, dtype=np.float64).reshape(-1, 1, 2)
    out = cv2.perspectiveTransform(pts, _H)
    return out.reshape(-1, 2)


def is_inside_workspace(x_mm: float, y_mm: float, margin: float | None = None) -> bool:
    """Cek apakah target robot berada dalam batas aman workspace."""
    _check_loaded()
    if margin is None:
        margin = float(getattr(config, "WORKSPACE_MARGIN_MM", 0.0))

    x_min = float(getattr(config, "ROBOT_WORKSPACE_X_MIN_MM", -999999.0)) - margin
    x_max = float(getattr(config, "ROBOT_WORKSPACE_X_MAX_MM",  999999.0)) + margin
    y_min = float(getattr(config, "ROBOT_WORKSPACE_Y_MIN_MM", -999999.0)) - margin
    y_max = float(getattr(config, "ROBOT_WORKSPACE_Y_MAX_MM",  999999.0)) + margin

    return x_min <= float(x_mm) <= x_max and y_min <= float(y_mm) <= y_max


# Nama lama dipertahankan supaya modul lama tidak error.
def is_inside_conveyor(x_mm: float, y_mm: float, margin: float = 0.0) -> bool:
    return is_inside_workspace(x_mm, y_mm, margin)


def source_points_px() -> np.ndarray:
    _check_loaded()
    return np.array(_SRC_POINTS, dtype=np.float64).copy()


def destination_points_mm() -> np.ndarray:
    _check_loaded()
    return np.array(_DST_POINTS_MM, dtype=np.float64).copy()


def coordinate_space() -> str:
    _check_loaded()
    return str(_COORDINATE_SPACE)


def draw_mm_grid(frame: np.ndarray, step_mm: float = 50.0, color=(40, 40, 40), thickness: int = 1) -> np.ndarray:
    """Overlay grid robot-space di frame kamera untuk debug visual."""
    _check_loaded()
    out = frame.copy()

    x_min = float(getattr(config, "ROBOT_WORKSPACE_X_MIN_MM", -250.0))
    x_max = float(getattr(config, "ROBOT_WORKSPACE_X_MAX_MM",  250.0))
    y_min = float(getattr(config, "ROBOT_WORKSPACE_Y_MIN_MM",  130.0))
    y_max = float(getattr(config, "ROBOT_WORKSPACE_Y_MAX_MM",  260.0))

    x = np.ceil(x_min / step_mm) * step_mm
    while x <= x_max:
        p1 = mm_to_pixel(x, y_min)
        p2 = mm_to_pixel(x, y_max)
        cv2.line(out, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), color, thickness, cv2.LINE_AA)
        cv2.putText(out, f"X{int(x)}", (int(p1[0]) + 2, int(p1[1]) + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
        x += step_mm

    y = np.ceil(y_min / step_mm) * step_mm
    while y <= y_max:
        p1 = mm_to_pixel(x_min, y)
        p2 = mm_to_pixel(x_max, y)
        cv2.line(out, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), color, thickness, cv2.LINE_AA)
        cv2.putText(out, f"Y{int(y)}", (int(p1[0]) + 2, int(p1[1]) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
        y += step_mm

    return out


if __name__ == "__main__":
    initialize()
    print("=== SELF TEST HOMOGRAPHY ===")
    for i, (x_mm, y_mm) in enumerate(destination_points_mm(), start=1):
        px, py = mm_to_pixel(x_mm, y_mm)
        x2, y2 = pixel_to_mm(px, py)
        print(f"P{i}: robot=({x_mm:.2f},{y_mm:.2f}) -> px=({px:.1f},{py:.1f}) -> robot=({x2:.2f},{y2:.2f})")
