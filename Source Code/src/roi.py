"""
roi.py
======
Modul ROI (Region of Interest) untuk CROP frame yang sudah di-undistort
ke area kerja conveyor, sebelum dikirim ke YOLO.

ROI di sini murni RECTANGLE CROP dari bounding box 4 titik yang dipilih
manual (klik di stream kamera live). Tidak ada polygon masking, tidak ada
filter deteksi tambahan -- ROI cuma memotong frame ke area kerja.

Cara pilih titik:
    1) Jalankan langsung:  python roi.py
       -> tahap 1 pilih 4 titik ROI deteksi, tahap 2 pilih 4 titik PICK ZONE.
    2) Saat main.py berjalan:
       - tekan "R" untuk pilih ulang ROI lalu PICK ZONE secara berurutan.
       - tekan "T" untuk pilih ulang PICK ZONE saja.

File hasil:
    - config.ROI_POINTS_PATH untuk ROI deteksi/crop.
    - config.PICK_ZONE_POINTS_PATH untuk polygon tengah trigger pick.
Keduanya memakai koordinat frame YANG SUDAH DI-UNDISTORT.
"""

import cv2
import numpy as np
import json
import os
from typing import Iterable

import config

# ================================================================
# STATE INTERNAL
# ================================================================

_points : list[list[float]] | None = None   # 4 titik [x, y] koordinat frame penuh (untuk digambar ulang saja)
_x      : int  = 0
_y      : int  = 0
_w      : int  = 0
_h      : int  = 0
_loaded : bool = False

_DEFAULT_POINTS_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "roi_points.json",
)


def _points_path() -> str:
    return getattr(config, "ROI_POINTS_PATH", _DEFAULT_POINTS_PATH)


# ================================================================
# LOAD / SAVE
# ================================================================

def _load_points(path: str) -> list[list[float]] | None:
    if not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        pts = data.get("points")
        if not pts or len(pts) != 4:
            print(f"[ROI] File {path} tidak berisi 4 titik yang valid.")
            return None
        return [[float(p[0]), float(p[1])] for p in pts]
    except Exception as e:
        print(f"[ROI] Gagal baca {path}: {e}")
        return None


def _save_points(path: str, points: list[list[float]], frame_w: int, frame_h: int) -> None:
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(
                {"points": points, "frame_w": frame_w, "frame_h": frame_h},
                f, indent=2,
            )
        print(f"[ROI] Titik ROI disimpan: {path}")
    except Exception as e:
        print(f"[ROI] Gagal simpan {path}: {e}")


# ================================================================
# BBOX DARI 4 TITIK (untuk crop rectangle)
# ================================================================

def _compute_bbox(points: list[list[float]], frame_w: int, frame_h: int):
    pts = np.array(points, dtype=np.float64)

    x1, y1 = pts.min(axis=0)
    x2, y2 = pts.max(axis=0)

    x = int(max(0, np.floor(x1)))
    y = int(max(0, np.floor(y1)))
    x2c = int(min(frame_w, np.ceil(x2)))
    y2c = int(min(frame_h, np.ceil(y2)))

    w = max(1, x2c - x)
    h = max(1, y2c - y)

    return x, y, w, h


# ================================================================
# INISIALISASI / SET POINTS
# ================================================================

def initialize(frame_w: int, frame_h: int) -> None:
    """Muat titik ROI tersimpan. Kalau belum ada file, fallback ke seluruh
    frame dan minta user memilih ROI (python roi.py atau tombol R di main.py)."""
    path = _points_path()
    points = _load_points(path)

    if points is None:
        print("=" * 70)
        print(f"[ROI] PERINGATAN: File titik ROI tidak ditemukan/valid: {path}")
        print("[ROI] ROI SAAT INI = SELURUH FRAME (tidak ada crop!)")
        print("[ROI] Jalankan 'python roi.py' atau tekan 'R' di main.py untuk pilih 4 titik ROI.")
        print("=" * 70)
        points = [
            [0.0, 0.0],
            [float(frame_w - 1), 0.0],
            [float(frame_w - 1), float(frame_h - 1)],
            [0.0, float(frame_h - 1)],
        ]

    set_points(points, frame_w, frame_h, save=False)


def set_points(points: list[list[float]], frame_w: int, frame_h: int, save: bool = True) -> None:
    """Set 4 titik ROI baru (koordinat frame penuh yang sudah di-undistort).
    Cuma bounding box (min/max) dari 4 titik ini yang dipakai untuk crop."""
    global _points, _x, _y, _w, _h, _loaded

    if len(points) != 4:
        raise ValueError("ROI harus tepat 4 titik.")

    pts = [[float(p[0]), float(p[1])] for p in points]
    x, y, w, h = _compute_bbox(pts, frame_w, frame_h)

    _points = pts
    _x, _y, _w, _h = x, y, w, h
    _loaded = True

    print(f"[ROI] Titik: {pts}")
    print(f"[ROI] Crop rectangle: x={_x} y={_y} w={_w} h={_h}")

    if save:
        _save_points(_points_path(), _points, frame_w, frame_h)


# ================================================================
# PILIH 4 TITIK LEWAT STREAM (LIVE)
# ================================================================

def select_points_interactive(
    read_frame_fn,
    window_name: str = "Pilih 4 Titik ROI (klik urut | R=reset | ENTER=simpan | ESC=batal)",
    destroy_window: bool = True,
):
    """
    Tampilkan stream live dan biarkan user KLIK 4 TITIK. Bounding box
    (kotak lurus) dari 4 titik ini yang akan dipakai untuk crop area
    kerja -- jadi klik saja di sudut kiri-atas dan kanan-bawah area
    kerja (2 titik sudah cukup membentuk kotak, tapi tetap klik 4 kali
    supaya format file konsisten).

    Return
    ------
    list [[x,y], x4] kalau user menekan ENTER/SPACE setelah 4 titik,
    atau None kalau dibatalkan (ESC).
    """
    clicked: list[list[float]] = []

    def _on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicked) < 4:
            clicked.append([float(x), float(y)])

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, _on_mouse)

    print("[ROI] Klik 4 titik di stream untuk membentuk area kerja (kotak).")
    print("[ROI] R=reset titik | ENTER/SPACE=simpan (setelah 4 titik) | ESC=batal")

    result = None

    while True:
        frame = read_frame_fn()
        if frame is None:
            if (cv2.waitKey(30) & 0xFF) == 27:
                break
            continue

        display = frame.copy()

        for i, pt in enumerate(clicked):
            p = (int(pt[0]), int(pt[1]))
            cv2.circle(display, p, 6, (0, 0, 255), -1)
            cv2.putText(
                display, str(i + 1), (p[0] + 8, p[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA,
            )

        if len(clicked) >= 2:
            xs = [p[0] for p in clicked]
            ys = [p[1] for p in clicked]
            x1, y1, x2, y2 = int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys))
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)

        status = f"Titik: {len(clicked)}/4"
        # Teks status tanpa background hitam. Outline hitam menjaga teks tetap terbaca.
        cv2.putText(
            display, status, (6, 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA,
        )
        cv2.putText(
            display, status, (6, 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA,
        )

        cv2.imshow(window_name, display)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            print("[ROI] Pemilihan dibatalkan.")
            result = None
            break
        elif key in (ord("r"), ord("R")):
            clicked.clear()
            print("[ROI] Titik direset.")
        elif key in (13, 32) and len(clicked) == 4:  # ENTER / SPACE
            result = [list(p) for p in clicked]
            print(f"[ROI] 4 titik dipilih: {result}")
            break

    if destroy_window:
        cv2.destroyWindow(window_name)
    return result


# ================================================================
# CROP
# ================================================================

def crop(frame: np.ndarray) -> tuple[np.ndarray, int, int]:
    """Crop frame ke rectangle ROI (bounding box dari 4 titik). Tidak ada
    masking polygon -- hanya potong kotak lurus."""
    if not _loaded:
        return frame, 0, 0
    return frame[_y: _y + _h, _x: _x + _w], _x, _y


def offset() -> tuple[int, int]:
    return _x, _y


def rect() -> tuple[int, int, int, int]:
    """Rectangle crop (x, y, w, h) dari bounding box 4 titik ROI."""
    return _x, _y, _w, _h


def points() -> list[list[float]]:
    """Salinan 4 titik ROI (koordinat frame penuh)."""
    return [] if _points is None else [list(p) for p in _points]


# ================================================================
# LETTERBOX UNTUK INPUT YOLO
# ================================================================

def letterbox_for_yolo(
    frame_roi: np.ndarray,
    target_w: int,
    target_h: int,
    color: int | tuple = 114,
) -> tuple[np.ndarray, dict]:
    """
    Resize ROI ke input YOLO tanpa merusak aspect ratio (letterbox).
    Menghindari objek gepeng/tertarik kalau ROI hasil crop tidak persegi.
    """
    src_h, src_w = frame_roi.shape[:2]
    if src_w <= 0 or src_h <= 0:
        out = np.full((target_h, target_w, 3), color, dtype=np.uint8)
        return out, {
            "enabled": True, "src_w": src_w, "src_h": src_h,
            "target_w": target_w, "target_h": target_h,
            "scale": 1.0, "new_w": target_w, "new_h": target_h,
            "pad_x": 0, "pad_y": 0,
        }

    scale = min(target_w / src_w, target_h / src_h)
    new_w = max(1, int(round(src_w * scale)))
    new_h = max(1, int(round(src_h * scale)))

    resized = cv2.resize(frame_roi, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    canvas_color = color if isinstance(color, tuple) else (int(color), int(color), int(color))
    out = np.full((target_h, target_w, 3), canvas_color, dtype=frame_roi.dtype)
    pad_x = (target_w - new_w) // 2
    pad_y = (target_h - new_h) // 2
    out[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized

    return out, {
        "enabled": True, "src_w": src_w, "src_h": src_h,
        "target_w": target_w, "target_h": target_h,
        "scale": scale, "new_w": new_w, "new_h": new_h,
        "pad_x": pad_x, "pad_y": pad_y,
    }


def no_letterbox_meta(src_w: int, src_h: int, target_w: int, target_h: int) -> dict:
    """Meta fallback untuk mode resize langsung (tanpa letterbox)."""
    return {
        "enabled": False, "src_w": src_w, "src_h": src_h,
        "target_w": target_w, "target_h": target_h,
        "scale": None, "new_w": target_w, "new_h": target_h,
        "pad_x": 0, "pad_y": 0,
    }


# ================================================================
# DRAW ROI BORDER
# ================================================================

def draw_roi_border(
    frame: np.ndarray,
    color: tuple = (0, 255, 0),
    thickness: int | None = None,
    label: bool = True,
) -> None:
    if not _loaded:
        return

    if thickness is None:
        thickness = int(getattr(config, "ROI_BORDER_THICKNESS", 1))
    thickness = max(1, int(thickness))

    cv2.rectangle(frame, (_x, _y), (_x + _w, _y + _h), color, thickness)

    if label:
        cv2.putText(
            frame, f"ROI {_w}x{_h}", (_x, max(18, _y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA,
        )




# ================================================================
# PICK ZONE (TAHAP 2) - berada di file yang sama dengan ROI
# ================================================================
# ROI = area crop/deteksi YOLO yang lebih luas.
# PICK ZONE = area pick penuh; STOP dipicu garis kuning di sisi akhir/paling kiri.

_pick_zone_points: list[list[float]] | None = None
_pick_zone_polygon_np: np.ndarray | None = None
_pick_zone_loaded: bool = False

_DEFAULT_PICK_ZONE_POINTS_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config",
    "pick_zone_points.json",
)


def _pick_zone_points_path() -> str:
    return getattr(config, "PICK_ZONE_POINTS_PATH", _DEFAULT_PICK_ZONE_POINTS_PATH)


def _load_pick_zone_points(path: str) -> list[list[float]] | None:
    if not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        pts = data.get("points")
        if not pts or len(pts) < 3:
            print(f"[PICK_ZONE] File {path} tidak berisi minimal 3 titik yang valid.")
            return None
        return [[float(p[0]), float(p[1])] for p in pts]
    except Exception as e:
        print(f"[PICK_ZONE] Gagal baca {path}: {e}")
        return None


def _save_pick_zone_points(path: str, points: list[list[float]], frame_w: int, frame_h: int) -> None:
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"points": points, "frame_w": frame_w, "frame_h": frame_h}, f, indent=2)
        print(f"[PICK_ZONE] Titik PICK ZONE disimpan: {path}")
    except Exception as e:
        print(f"[PICK_ZONE] Gagal simpan {path}: {e}")

def _default_pick_zone_points(frame_w: int, frame_h: int, roi_rect: tuple[int, int, int, int] | None = None) -> list[list[float]]:
    """Fallback polygon di tengah ROI/frame bila file pick_zone_points.json belum ada."""
    if roi_rect is not None:
        rx, ry, rw, rh = roi_rect
    else:
        rx, ry, rw, rh = 0, 0, frame_w, frame_h

    width_ratio = float(getattr(config, "PICK_ZONE_DEFAULT_WIDTH_RATIO", 0.35))
    height_ratio = float(getattr(config, "PICK_ZONE_DEFAULT_HEIGHT_RATIO", 0.35))
    width_ratio = min(max(width_ratio, 0.05), 1.0)
    height_ratio = min(max(height_ratio, 0.05), 1.0)

    zw = max(8.0, rw * width_ratio)
    zh = max(8.0, rh * height_ratio)
    cx = rx + rw / 2.0
    cy = ry + rh / 2.0

    x1 = max(0.0, cx - zw / 2.0)
    x2 = min(float(frame_w - 1), cx + zw / 2.0)
    y1 = max(0.0, cy - zh / 2.0)
    y2 = min(float(frame_h - 1), cy + zh / 2.0)

    return [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]


def initialize_pick_zone(
    frame_w: int,
    frame_h: int,
    roi_rect: tuple[int, int, int, int] | None = None,
) -> None:
    """Muat polygon PICK ZONE. Kalau belum ada, fallback ke tengah ROI/frame."""
    path = _pick_zone_points_path()
    pts = _load_pick_zone_points(path)

    if pts is None:
        print("=" * 70)
        print(f"[PICK_ZONE] PERINGATAN: File PICK ZONE tidak ditemukan/valid: {path}")
        print("[PICK_ZONE] Fallback sementara = polygon kecil di tengah ROI/frame.")
        print("[PICK_ZONE] Tekan T di main.py atau jalankan 'python roi.py' untuk memilih titik yang tepat.")
        print("=" * 70)
        pts = _default_pick_zone_points(frame_w, frame_h, roi_rect)

    set_pick_zone_points(pts, frame_w, frame_h, save=False)


def set_pick_zone_points(points: list[list[float]], frame_w: int, frame_h: int, save: bool = True) -> None:
    """Set polygon PICK ZONE dalam koordinat frame penuh yang sudah di-undistort."""
    global _pick_zone_points, _pick_zone_polygon_np, _pick_zone_loaded

    if len(points) < 3:
        raise ValueError("PICK ZONE minimal harus 3 titik polygon.")

    pts: list[list[float]] = []
    for p in points:
        x = min(max(float(p[0]), 0.0), float(frame_w - 1))
        y = min(max(float(p[1]), 0.0), float(frame_h - 1))
        pts.append([x, y])

    _pick_zone_points = pts
    _pick_zone_polygon_np = np.array(pts, dtype=np.float32).reshape((-1, 1, 2))
    _pick_zone_loaded = True

    print(f"[PICK_ZONE] Titik: {pts}")

    if save:
        _save_pick_zone_points(_pick_zone_points_path(), pts, frame_w, frame_h)


def pick_zone_is_enabled() -> bool:
    return bool(getattr(config, "PICK_ZONE_ENABLE", True))


def get_pick_zone_points() -> list[list[float]]:
    return [] if _pick_zone_points is None else [list(p) for p in _pick_zone_points]


def _pick_zone_flat_points() -> np.ndarray | None:
    if _pick_zone_polygon_np is None:
        return None
    try:
        return _pick_zone_polygon_np.reshape(-1, 2).astype(np.float32)
    except Exception:
        return None


def pick_zone_center() -> tuple[float, float] | None:
    """Titik tengah/cross PICK ZONE.

    Polygon PICK ZONE tetap dipakai sebagai batas visual dan batas aman,
    tetapi trigger pick-and-place memakai titik tengah ini dengan toleransi kecil.
    """
    pts = _pick_zone_flat_points()
    if pts is None or len(pts) < 3:
        return None

    try:
        m = cv2.moments(pts.reshape((-1, 1, 2)))
        if abs(float(m.get("m00", 0.0))) > 1e-6:
            return float(m["m10"] / m["m00"]), float(m["m01"] / m["m00"])
    except Exception:
        pass

    # Fallback aman: rata-rata titik polygon.
    return float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))


def _pick_zone_bbox_size() -> tuple[float, float]:
    pts = _pick_zone_flat_points()
    if pts is None or len(pts) < 3:
        return 0.0, 0.0
    x, y, w, h = cv2.boundingRect(pts.astype(np.int32).reshape((-1, 1, 2)))
    return float(max(w, 1)), float(max(h, 1))


def _pick_zone_bbox_rect() -> tuple[int, int, int, int] | None:
    pts = _pick_zone_flat_points()
    if pts is None or len(pts) < 3:
        return None
    x, y, w, h = cv2.boundingRect(pts.astype(np.int32).reshape((-1, 1, 2)))
    return int(x), int(y), int(x + w), int(y + h)


def pick_zone_center_gate_rect() -> tuple[int, int, int, int] | None:
    """Kompatibilitas versi lama.

    Pada revisi stop-line, center gate tidak lagi dipakai sebagai syarat conveyor
    STOP maupun target pick. Fungsi ini tetap ada agar import/kode lama tidak error.
    """
    if not bool(getattr(config, "PICK_ZONE_CENTER_GATE_ENABLE", False)):
        return None

    center = pick_zone_center()
    if center is None:
        return None

    bbox_w, bbox_h = _pick_zone_bbox_size()
    width_ratio = float(getattr(config, "PICK_ZONE_CENTER_GATE_WIDTH_RATIO", 0.10))
    height_ratio = float(getattr(config, "PICK_ZONE_CENTER_GATE_HEIGHT_RATIO", 0.10))
    min_size = float(getattr(config, "PICK_ZONE_CENTER_GATE_MIN_SIZE_PX", 18))
    max_size = float(getattr(config, "PICK_ZONE_CENTER_GATE_MAX_SIZE_PX", 45))

    gate_w = max(min_size, bbox_w * max(0.01, width_ratio))
    gate_h = max(min_size, bbox_h * max(0.01, height_ratio))
    gate_w = min(gate_w, max_size, max(1.0, bbox_w))
    gate_h = min(gate_h, max_size, max(1.0, bbox_h))

    cx, cy = center
    x1 = int(round(cx - gate_w / 2.0))
    y1 = int(round(cy - gate_h / 2.0))
    x2 = int(round(cx + gate_w / 2.0))
    y2 = int(round(cy + gate_h / 2.0))
    return x1, y1, x2, y2


def pick_zone_center_gate_contains_point(x: float, y: float) -> bool:
    rect = pick_zone_center_gate_rect()
    if rect is None:
        return False
    x1, y1, x2, y2 = rect
    return x1 <= float(x) <= x2 and y1 <= float(y) <= y2


def pick_zone_polygon_contains_point(x: float, y: float) -> bool:
    """True jika titik/centroid berada di dalam polygon PICK ZONE penuh."""
    if not pick_zone_is_enabled():
        return True
    if not _pick_zone_loaded or _pick_zone_polygon_np is None:
        return True
    return cv2.pointPolygonTest(_pick_zone_polygon_np, (float(x), float(y)), False) >= 0


def pick_zone_contains_point(x: float, y: float) -> bool:
    """Kompatibilitas nama lama: sekarang berarti SELURUH polygon PICK ZONE.

    Revisi ini sengaja tidak memakai center gate untuk target pick, supaya robot
    tetap bisa pick objek di seluruh area PICK ZONE setelah conveyor berhenti.
    """
    return pick_zone_polygon_contains_point(x, y)


def pick_zone_stop_line_info() -> dict | None:
    """Info garis trigger STOP conveyor di PICK ZONE.

    Garis dihitung dari bounding box polygon PICK ZONE. Default-nya sisi LEFT,
    sesuai kebutuhan conveyor berhenti saat objek menyentuh garis paling kiri.
    """
    if not pick_zone_is_enabled() or not bool(getattr(config, "PICK_ZONE_STOP_LINE_ENABLE", True)):
        return None
    rect = _pick_zone_bbox_rect()
    if rect is None:
        return None
    x1, y1, x2, y2 = rect
    side = str(getattr(config, "PICK_ZONE_STOP_LINE_SIDE", "LEFT")).strip().upper()
    if side not in {"LEFT", "RIGHT", "TOP", "BOTTOM"}:
        side = "LEFT"
    if side == "LEFT":
        p1, p2 = (x1, y1), (x1, y2)
        axis, value = "x", float(x1)
    elif side == "RIGHT":
        p1, p2 = (x2, y1), (x2, y2)
        axis, value = "x", float(x2)
    elif side == "TOP":
        p1, p2 = (x1, y1), (x2, y1)
        axis, value = "y", float(y1)
    else:
        p1, p2 = (x1, y2), (x2, y2)
        axis, value = "y", float(y2)
    return {"side": side, "axis": axis, "value": value, "p1": p1, "p2": p2, "rect": rect}


def pick_zone_object_touches_stop_line(obj: dict) -> bool:
    """True jika bbox/mask objek menyentuh garis akhir STOP LINE.

    Untuk side LEFT: sisi kiri objek <= garis kiri + tolerance.
    Untuk side RIGHT: sisi kanan objek >= garis kanan - tolerance.
    Untuk TOP/BOTTOM analog pada sumbu Y.
    Centroid default tetap wajib di dalam polygon PICK ZONE agar deteksi luar
    tidak keliru memicu STOP.
    """
    info = pick_zone_stop_line_info()
    if info is None:
        return False

    cx = float(obj.get("cx", 0.0))
    cy = float(obj.get("cy", 0.0))
    if bool(getattr(config, "PICK_ZONE_STOP_REQUIRE_CENTROID_IN_ZONE", True)):
        if not pick_zone_polygon_contains_point(cx, cy):
            return False

    # bbox berasal dari mask segmentasi di visualization.py. Jika belum ada,
    # fallback ke centroid agar program lama tetap jalan, meski kurang presisi.
    try:
        x1 = float(obj.get("x1", obj.get("bbox", [cx, cy, cx, cy])[0]))
        y1 = float(obj.get("y1", obj.get("bbox", [cx, cy, cx, cy])[1]))
        x2 = float(obj.get("x2", obj.get("bbox", [cx, cy, cx, cy])[2]))
        y2 = float(obj.get("y2", obj.get("bbox", [cx, cy, cx, cy])[3]))
    except Exception:
        x1 = x2 = cx
        y1 = y2 = cy

    side = str(info.get("side", "LEFT")).upper()
    value = float(info.get("value", 0.0))
    tol = float(getattr(config, "PICK_ZONE_STOP_LINE_TOLERANCE_PX", 6))

    if side == "LEFT":
        return x1 <= value + tol
    if side == "RIGHT":
        return x2 >= value - tol
    if side == "TOP":
        return y1 <= value + tol
    if side == "BOTTOM":
        return y2 >= value - tol
    return False


def pick_stop_line_trigger_track_ids(
    tracks: dict[int, dict],
    skipped_track_ids: set[int],
    sent_track_ids: set[int],
    frame_index: int,
) -> list[int]:
    """ID yang baru memicu STOP karena menyentuh stop line PICK ZONE."""
    if not pick_zone_is_enabled() or not bool(getattr(config, "PICK_ZONE_STOP_LINE_ENABLE", True)):
        return []

    ignore_skipped = bool(getattr(config, "CONVEYOR_IGNORE_SKIPPED_IDS_FOR_BLOCK", False))
    hold_frames = int(getattr(config, "PICK_ZONE_STOP_LINE_HOLD_FRAMES", 3))

    triggered: list[int] = []
    for track_id, tr in tracks.items():
        track_id = int(track_id)
        if track_id <= 0:
            continue
        if ignore_skipped and track_id in skipped_track_ids:
            continue
        if track_id in sent_track_ids:
            continue

        visible = bool(tr.get("visible", False))
        touches = bool(tr.get("touches_pick_stop_line", False))
        if visible and touches:
            triggered.append(track_id)
            continue

        last_touch = tr.get("last_pick_stop_line_frame", None)
        if last_touch is not None:
            try:
                frames_since = int(frame_index) - int(last_touch)
            except Exception:
                frames_since = hold_frames + 1
            if 0 < frames_since <= hold_frames:
                triggered.append(track_id)

    return sorted(set(triggered))

def annotate_pick_zone_objects(centroids: list[dict]) -> None:
    """Tambahkan status PICK ZONE dan STOP LINE pada setiap objek."""
    for obj in centroids:
        inside = pick_zone_polygon_contains_point(float(obj.get("cx", 0.0)), float(obj.get("cy", 0.0)))
        touches_stop = pick_zone_object_touches_stop_line(obj) if inside else False
        obj["in_pick_zone"] = inside
        obj["touches_pick_stop_line"] = touches_stop


def visible_pick_zone_objects(centroids: Iterable[dict]) -> list[dict]:
    return [obj for obj in centroids if bool(obj.get("in_pick_zone", False))]


def blocking_pick_zone_track_ids(
    tracks: dict[int, dict],
    skipped_track_ids: set[int],
    sent_track_ids: set[int],
    frame_index: int,
) -> list[int]:
    """Return ID yang membuat conveyor wajib STOP berdasarkan PICK ZONE.

    Bedanya dengan ROI lama: ID yang berada di ROI tapi belum masuk PICK ZONE
    tidak menahan conveyor. ID yang baru hilang setelah masuk PICK ZONE tetap
    ditahan beberapa frame untuk kasus tertutup robot/gripper.
    """
    if not pick_zone_is_enabled():
        return []
    if not bool(getattr(config, "CONVEYOR_BLOCK_WHILE_PICK_ZONE_OCCUPIED", True)):
        return []

    hold_frames = int(getattr(config, "PICK_ZONE_OCCLUSION_HOLD_FRAMES", 12))
    ignore_skipped = bool(getattr(config, "CONVEYOR_IGNORE_SKIPPED_IDS_FOR_BLOCK", False))
    ignore_sent_when_not_visible = bool(getattr(config, "CONVEYOR_IGNORE_SENT_IDS_WHEN_NOT_VISIBLE", True))

    blocking: list[int] = []
    for track_id, tr in tracks.items():
        track_id = int(track_id)
        if track_id <= 0:
            continue
        if ignore_skipped and track_id in skipped_track_ids:
            continue

        visible = bool(tr.get("visible", False))
        in_pick_zone = bool(tr.get("in_pick_zone", False))
        last_in_zone = tr.get("last_in_pick_zone_frame", None)

        if visible and in_pick_zone:
            blocking.append(track_id)
            continue

        if ignore_sent_when_not_visible and track_id in sent_track_ids and not visible:
            continue

        if last_in_zone is not None:
            try:
                frames_since_zone = int(frame_index) - int(last_in_zone)
            except Exception:
                frames_since_zone = hold_frames + 1
            if 0 < frames_since_zone <= hold_frames:
                blocking.append(track_id)

    return sorted(set(blocking))


def draw_pick_zone(
    frame: np.ndarray,
    color: tuple = (255, 0, 255),
    thickness: int = 2,
    label: bool = True,
    draw_boundary: bool | None = None,
    draw_stop_line: bool | None = None,
    stop_line_label: bool | None = None,
) -> None:
    if not pick_zone_is_enabled() or not _pick_zone_loaded or _pick_zone_polygon_np is None:
        return

    if draw_boundary is None:
        draw_boundary = bool(getattr(config, "SHOW_PICK_ZONE_BORDER_ON_FRAME", True))
    if draw_stop_line is None:
        draw_stop_line = bool(getattr(config, "SHOW_PICK_ZONE_STOP_LINE_ON_FRAME", True))
    if stop_line_label is None:
        stop_line_label = bool(getattr(config, "SHOW_PICK_ZONE_STOP_LINE_LABEL", False))

    pts = _pick_zone_polygon_np.astype(np.int32)

    # Revisi tampilan: polygon/kotak PICK ZONE dapat disembunyikan di live-view,
    # sementara logika pick-zone tetap aktif. Jadi yang terlihat cukup garis
    # kuning stop line untuk titik pemberhentian objek.
    if draw_boundary:
        cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=thickness)

        if bool(getattr(config, "PICK_ZONE_FILL_OVERLAY", False)):
            overlay = frame.copy()
            cv2.fillPoly(overlay, [pts], color)
            alpha = float(getattr(config, "PICK_ZONE_FILL_ALPHA", 0.12))
            cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0, frame)

        if label and bool(getattr(config, "SHOW_PICK_ZONE_LABEL_ON_FRAME", True)):
            p0 = pts.reshape(-1, 2)[0]
            cv2.putText(
                frame,
                "PICK ZONE",
                (int(p0[0]), max(18, int(p0[1]) - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )

    if not draw_stop_line:
        return

    # STOP LINE: garis kuning di sisi akhir PICK ZONE. Conveyor hanya STOP
    # saat bbox/mask objek menyentuh garis ini. Setelah STOP, target pick tetap
    # boleh dipilih dari seluruh polygon PICK ZONE.
    info = pick_zone_stop_line_info()
    if info is None:
        return

    p1 = tuple(int(v) for v in info["p1"])
    p2 = tuple(int(v) for v in info["p2"])
    stop_color = (0, 255, 255)
    stop_thickness = max(1, int(getattr(config, "PICK_ZONE_STOP_LINE_THICKNESS", 1)))
    stop_marker_radius = max(0, int(getattr(config, "PICK_ZONE_STOP_LINE_MARKER_RADIUS", 3)))
    cv2.line(frame, p1, p2, stop_color, stop_thickness, cv2.LINE_AA)
    mid = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
    if stop_marker_radius > 0:
        cv2.circle(frame, mid, stop_marker_radius, stop_color, -1, cv2.LINE_AA)

    if stop_line_label:
        cv2.putText(
            frame,
            f"STOP LINE {info['side']}",
            (mid[0] + 8, max(18, mid[1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            stop_color,
            1,
            cv2.LINE_AA,
        )

def draw_pick_zone_object_status(frame: np.ndarray, centroids: list[dict]) -> None:
    if not pick_zone_is_enabled() or not bool(getattr(config, "SHOW_PICK_ZONE_OBJECT_STATUS", True)):
        return
    for obj in centroids:
        if not bool(obj.get("in_pick_zone", False)):
            continue
        cx = int(obj.get("cx", 0))
        cy = int(obj.get("cy", 0))
        track_id = int(obj.get("track_id", 0) or 0)
        if bool(obj.get("touches_pick_stop_line", False)):
            text = f"STOP LINE ID{track_id}" if track_id > 0 else "STOP LINE"
        else:
            text = f"IN PICK ZONE ID{track_id}" if track_id > 0 else "IN PICK ZONE"
        cv2.putText(
            frame,
            text,
            (cx + 10, cy - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )


def select_pick_zone_points_interactive(
    read_frame_fn,
    window_name: str = "Pilih PICK ZONE (klik 4 titik | R=reset | ENTER=simpan | ESC=batal)",
    destroy_window: bool = True,
):
    clicked: list[list[float]] = []

    def _on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicked) < 4:
            clicked.append([float(x), float(y)])

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, _on_mouse)

    print("[PICK_ZONE] Klik 4 titik PICK ZONE. Garis kuning STOP LINE otomatis dibuat di sisi akhir/paling kiri.")
    print("[PICK_ZONE] R=reset titik | ENTER/SPACE=simpan setelah 4 titik | ESC=batal")

    result = None
    while True:
        frame = read_frame_fn()
        if frame is None:
            if (cv2.waitKey(30) & 0xFF) == 27:
                break
            continue

        display = frame.copy()

        # Tahap 2 tetap memakai stream kamera yang sama.
        # Garis ROI hijau tetap digambar supaya posisi PICK ZONE bisa dipilih
        # di dalam area ROI, bukan di tepi atau luar area kerja.
        draw_roi_border(display, color=(0, 255, 0), thickness=2, label=True)

        # Jika sudah ada PICK ZONE lama, tampilkan tipis sebagai referensi.
        if _pick_zone_loaded and _pick_zone_polygon_np is not None:
            old_pts = _pick_zone_polygon_np.astype(np.int32)
            cv2.polylines(display, [old_pts], isClosed=True, color=(180, 0, 180), thickness=1)

        for i, pt in enumerate(clicked):
            p = (int(pt[0]), int(pt[1]))
            cv2.circle(display, p, 6, (255, 0, 255), -1)
            cv2.putText(display, str(i + 1), (p[0] + 8, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2, cv2.LINE_AA)

        if len(clicked) >= 2:
            pts = np.array(clicked, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(display, [pts], isClosed=(len(clicked) >= 3), color=(255, 0, 255), thickness=2)

        # Saat 4 titik sudah dipilih, langsung tampilkan STOP LINE kuning.
        # Conveyor STOP dipicu garis ini, sedangkan pick tetap bisa dilakukan
        # di seluruh area polygon PICK ZONE.
        if len(clicked) == 4:
            tmp_pts = np.array(clicked, dtype=np.float32).reshape((-1, 1, 2))
            bx, by, bw, bh = cv2.boundingRect(tmp_pts.astype(np.int32))
            side = str(getattr(config, "PICK_ZONE_STOP_LINE_SIDE", "LEFT")).strip().upper()
            if side not in {"LEFT", "RIGHT", "TOP", "BOTTOM"}:
                side = "LEFT"

            if side == "LEFT":
                p1, p2 = (bx, by), (bx, by + bh)
            elif side == "RIGHT":
                p1, p2 = (bx + bw, by), (bx + bw, by + bh)
            elif side == "TOP":
                p1, p2 = (bx, by), (bx + bw, by)
            else:
                p1, p2 = (bx, by + bh), (bx + bw, by + bh)

            gate_color = (0, 255, 255)
            stop_thickness = max(1, int(getattr(config, "PICK_ZONE_STOP_LINE_THICKNESS", 1)))
            stop_marker_radius = max(0, int(getattr(config, "PICK_ZONE_STOP_LINE_MARKER_RADIUS", 3)))
            cv2.line(display, p1, p2, gate_color, stop_thickness, cv2.LINE_AA)
            mid = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
            if stop_marker_radius > 0:
                cv2.circle(display, mid, stop_marker_radius, gate_color, -1, cv2.LINE_AA)
            cv2.putText(display, f"STOP LINE {side}", (mid[0] + 8, max(18, mid[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, gate_color, 1, cv2.LINE_AA)

        status = f"Tahap 2/2 PICK ZONE: {len(clicked)}/4 titik | STOP = garis kuning, PICK = seluruh zone"
        # Teks status tanpa background hitam. Outline hitam menjaga teks tetap terbaca.
        cv2.putText(display, status, (6, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(display, status, (6, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow(window_name, display)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("[PICK_ZONE] Pemilihan dibatalkan.")
            result = None
            break
        if key in (ord("r"), ord("R")):
            clicked.clear()
            print("[PICK_ZONE] Titik direset.")
        elif key in (13, 32) and len(clicked) == 4:
            result = [list(p) for p in clicked]
            print(f"[PICK_ZONE] 4 titik dipilih: {result}")
            break

    if destroy_window:
        cv2.destroyWindow(window_name)
    return result




# ================================================================
# JALANKAN LANGSUNG: python roi.py -> pilih ROI lewat stream kamera
# ================================================================

def _run_selector_standalone():
    try:
        cam_index = getattr(config, "CAM_INDEX", 0)
        cam_width = getattr(config, "CAM_WIDTH", 640)
        cam_height = getattr(config, "CAM_HEIGHT", 480)
    except Exception:
        cam_index, cam_width, cam_height = 0, 640, 480

    from camera_thread import CameraThread
    from undistortion import initialize as u_initialize, undistort_frame as u_undistort_frame

    cam = CameraThread(cam_index, cam_width, cam_height)

    frame = None
    for _ in range(60):
        frame = cam.read()
        if frame is not None:
            break
        cv2.waitKey(50)

    if frame is None:
        print("ERROR: Frame awal gagal dibaca dari kamera.")
        cam.release()
        return

    u_initialize(frame.shape[1], frame.shape[0])

    def _read():
        f = cam.read()
        if f is None:
            return None
        return u_undistort_frame(f)

    sample = _read()
    if sample is not None:
        frame_h, frame_w = sample.shape[:2]
    else:
        frame_h, frame_w = frame.shape[0], frame.shape[1]

    calibration_window = "Kalibrasi ROI + PICK ZONE - satu kamera"

    print("[ROI] Tahap 1/2: pilih ROI deteksi YOLO.")
    new_points = select_points_interactive(
        _read,
        window_name=calibration_window,
        destroy_window=False,
    )

    if new_points is not None:
        set_points(new_points, frame_w, frame_h, save=True)
        print("[ROI] Tahap 1 selesai. ROI sudah disimpan.")

        print("[PICK_ZONE] Tahap 2/2: pilih polygon PICK ZONE. Conveyor STOP saat objek menyentuh STOP LINE kuning; robot pick di seluruh zone.")
        new_pick_points = select_pick_zone_points_interactive(
            _read,
            window_name=calibration_window,
            destroy_window=True,
        )
        if new_pick_points is not None:
            set_pick_zone_points(new_pick_points, frame_w, frame_h, save=True)
            print("[PICK_ZONE] Tahap 2 selesai. PICK ZONE sudah disimpan.")
        else:
            print("[PICK_ZONE] Tahap 2 dibatalkan. PICK ZONE lama/fallback tetap dipakai.")

        print("[ROI] Selesai. ROI dan PICK ZONE siap dipakai main.py.")
    else:
        print("[ROI] Tidak ada perubahan disimpan.")

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    _run_selector_standalone()
