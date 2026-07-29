"""Utilitas pembukaan kamera untuk sumber OBS Virtual Camera.

Program utama tidak membuka webcam fisik secara langsung. Webcam diatur di OBS,
kemudian OpenCV membaca keluaran ``OBS Virtual Camera`` melalui DirectShow.
"""

from __future__ import annotations

import cv2

import config


def _directshow_devices() -> list[str]:
    """Mengambil daftar perangkat video DirectShow jika pygrabber tersedia."""
    try:
        from pygrabber.dshow_graph import FilterGraph
    except ImportError:
        return []

    try:
        return list(FilterGraph().get_input_devices())
    except Exception as exc:
        print(f"[CAMERA] Daftar perangkat DirectShow gagal dibaca: {exc}")
        return []


def resolve_camera_index() -> int:
    """Mencari OBS Virtual Camera berdasarkan nama, lalu memakai indeks cadangan."""
    preferred_name = str(getattr(config, "CAM_DEVICE_NAME", "OBS Virtual Camera")).strip()
    fallback_index = int(getattr(config, "CAM_INDEX", 1))
    require_named_device = bool(getattr(config, "CAM_REQUIRE_DEVICE_NAME", False))

    devices = _directshow_devices()
    if devices:
        print("[CAMERA] Perangkat DirectShow yang terdeteksi:")
        for idx, name in enumerate(devices):
            print(f"  [{idx}] {name}")

        preferred_lower = preferred_name.lower()
        for idx, name in enumerate(devices):
            if preferred_lower in name.lower():
                print(f"[CAMERA] Menggunakan {name} pada index {idx}.")
                return idx

        message = f"Perangkat '{preferred_name}' tidak ditemukan. Pastikan Start Virtual Camera di OBS sudah aktif."
        if require_named_device:
            raise RuntimeError(message)
        print(f"[CAMERA] WARNING: {message}")
    else:
        print(
            "[CAMERA] pygrabber belum tersedia, sehingga nama perangkat tidak dapat "
            "diverifikasi. Menggunakan CAM_INDEX sebagai indeks cadangan."
        )
        print("[CAMERA] Opsional: pip install pygrabber")

    print(f"[CAMERA] Menggunakan CAM_INDEX cadangan: {fallback_index}")
    return fallback_index


def open_camera(width: int | None = None, height: int | None = None) -> cv2.VideoCapture:
    """Membuka OBS Virtual Camera dengan konfigurasi proyek."""
    camera_index = resolve_camera_index()
    width = int(width if width is not None else getattr(config, "CAM_WIDTH", 640))
    height = int(height if height is not None else getattr(config, "CAM_HEIGHT", 480))
    fps = float(getattr(config, "CAM_FPS", 30))

    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(
            "OBS Virtual Camera tidak dapat dibuka. Buka OBS Studio, aktifkan webcam "
            "sebagai Video Capture Device, lalu klik Start Virtual Camera."
        )

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(
        f"[CAMERA] Stream aktif: index={camera_index}, "
        f"resolusi={actual_width}x{actual_height}, fps={actual_fps:.1f}"
    )
    return cap
