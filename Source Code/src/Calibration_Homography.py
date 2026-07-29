"""
Calibration_Homography.py
=========================
Kalibrasi pixel kamera langsung ke koordinat robot base.

Hasil akhirnya:
    pixel kamera -> X_robot_mm, Y_robot_mm

Python TIDAK menghitung inverse kinematics. Python hanya mengirim:
    G1 X.. Y.. Z..
ke ESP32. IK tetap di firmware ESP32.

1. Tentukan 4 titik robot di config.ROBOT_CALIBRATION_POINTS_MM.
2. Buat tanda/marker fisik pada 4 titik tersebut di bidang pick.
   Cara paling akurat: gerakkan robot ke tiap titik dengan ESP32, lalu tandai posisi ujung tool/suction.
3. Jalankan file ini.
4. Klik marker di kamera sesuai urutan titik di config.
5. Tekan ENTER untuk simpan homography_robot.json.
"""

from __future__ import annotations

import json
import os
import sys

import cv2
import numpy as np

_SRC_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SRC_DIR)

import config
from camera_source import open_camera
from undistortion import initialize, undistort_frame

OUTPUT_FILE = getattr(
    config,
    "HOMOGRAPHY_PATH",
    os.path.join(os.path.dirname(_SRC_DIR), "config", "homography_robot.json"),
)

DST_POINTS_MM = np.array(getattr(config, "ROBOT_CALIBRATION_POINTS_MM"), dtype=np.float32)
if DST_POINTS_MM.shape != (4, 2):
    raise ValueError("config.ROBOT_CALIBRATION_POINTS_MM harus berisi tepat 4 titik [[X,Y], ...].")

POINT_RADIUS = 4
POINT_RING_RADIUS = 8
POINT_THICKNESS = 1
LINE_THICKNESS = 1

COLORS = [
    (0, 255, 0),
    (0, 200, 255),
    (0, 100, 255),
    (255, 100, 0),
]

points_px: list[tuple[int, int]] = []


def label_for(i: int) -> str:
    x, y = DST_POINTS_MM[i]
    return f"P{i + 1}: X{x:.0f} Y{y:.0f}"


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points_px) < 4:
        points_px.append((int(x), int(y)))
        print(f"  [+] Klik {label_for(len(points_px)-1)} pada pixel ({x}, {y})")


def draw_guide(frame):
    h, w = frame.shape[:2]

    # garis bantu tengah frame
    overlay = frame.copy()
    cv2.line(overlay, (w // 2, 0), (w // 2, h), (60, 60, 60), 1)
    cv2.line(overlay, (0, h // 2), (w, h // 2), (60, 60, 60), 1)
    cv2.addWeighted(overlay, 0.35, frame, 0.65, 0, frame)

    if len(points_px) >= 2:
        for i in range(len(points_px) - 1):
            cv2.line(frame, points_px[i], points_px[i + 1], (255, 255, 0), LINE_THICKNESS, cv2.LINE_AA)
    if len(points_px) == 4:
        cv2.line(frame, points_px[3], points_px[0], (255, 255, 0), LINE_THICKNESS, cv2.LINE_AA)

    for i, (px, py) in enumerate(points_px):
        color = COLORS[i]
        cv2.circle(frame, (px, py), POINT_RADIUS, color, -1)
        cv2.circle(frame, (px, py), POINT_RING_RADIUS, (255, 255, 255), POINT_THICKNESS)
        cv2.putText(frame, label_for(i), (px + 10, py - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)

    if len(points_px) < 4:
        instruction = f"Klik {label_for(len(points_px))} | Z=undo | R=reset | ESC=batal"
    else:
        instruction = "ENTER=simpan | Z=undo | R=reset | ESC=batal"

    cv2.rectangle(frame, (0, h - 42), (w, h), (0, 0, 0), -1)
    cv2.putText(frame, instruction, (10, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 1, cv2.LINE_AA)


def compute_and_save() -> bool:
    src = np.array(points_px, dtype=np.float32)
    dst = DST_POINTS_MM.astype(np.float32)

    H, mask = cv2.findHomography(src, dst, method=0)
    if H is None:
        print("ERROR: Homography gagal. Pastikan 4 titik tidak segaris dan urutannya benar.")
        return False

    os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)
    data = {
        "coordinate_space": "robot_base",
        "unit": "mm",
        "H": H.tolist(),
        "src_points": [[int(x), int(y)] for x, y in points_px],
        "dst_points_mm": dst.astype(float).tolist(),
        "notes": "pixel kamera -> X_robot_mm,Y_robot_mm. IK dihitung di ESP32.",
    }
    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    print(f"\n[OK] Homography robot disimpan ke: {OUTPUT_FILE}")
    print("\nCek hasil titik kalibrasi:")
    for i, (px, py) in enumerate(points_px):
        pt = np.array([[[px, py]]], dtype=np.float64)
        out = cv2.perspectiveTransform(pt, H)
        xr, yr = out[0, 0]
        ex, ey = dst[i]
        print(f"  P{i+1}: px=({px},{py}) -> robot=({xr:.2f},{yr:.2f}) target=({ex:.2f},{ey:.2f})")

    return True


def main():
    try:
        cap = open_camera(config.CAM_WIDTH, config.CAM_HEIGHT)
    except RuntimeError as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1)

    ok, frame = cap.read()
    if not ok:
        print("ERROR: Gagal membaca frame kamera.")
        raise SystemExit(1)

    initialize(frame.shape[1], frame.shape[0])

    print("\n=== KALIBRASI HOMOGRAPHY ROBOT ===")
    print("Klik marker pada kamera sesuai urutan koordinat robot berikut:")
    for i in range(4):
        print(f"  {label_for(i)}")
    print("\nKontrol: klik kiri=tambah titik | Z=undo | R=reset | ENTER=simpan | ESC=batal\n")

    win = "Kalibrasi Homography Robot"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, mouse_callback)

    while True:
        ok, raw = cap.read()
        if not ok:
            continue

        frame = undistort_frame(raw)
        display = frame.copy()
        draw_guide(display)
        cv2.imshow(win, display)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("Kalibrasi dibatalkan.")
            break
        if key in (ord("z"), ord("Z")):
            if points_px:
                removed = points_px.pop()
                print(f"  [Z] Undo titik: {removed}")
        if key in (ord("r"), ord("R")):
            points_px.clear()
            print("  [R] Reset titik.")
        if key == 13 and len(points_px) == 4:
            compute_and_save()
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
