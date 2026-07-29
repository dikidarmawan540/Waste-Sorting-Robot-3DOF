import cv2
import numpy as np
import os
import sys

# =========================
# LOAD CALIBRATION
# =========================

CALIB_FILE = r"D:\Documents\Yolov11-seg Skripsi\config\camera_calibration.yaml"

if not os.path.isfile(CALIB_FILE):
    print(f"ERROR: File kalibrasi tidak ditemukan:\n{CALIB_FILE}")
    sys.exit()

fs = cv2.FileStorage(CALIB_FILE, cv2.FILE_STORAGE_READ)

camera_matrix = fs.getNode("camera_matrix").mat()
dist_coeffs   = fs.getNode("dist_coeffs").mat()

fs.release()

if camera_matrix is None:
    print("ERROR: Gagal membaca camera_matrix dari file kalibrasi.")
    sys.exit()

if dist_coeffs is None:
    print("ERROR: Gagal membaca dist_coeffs dari file kalibrasi.")
    sys.exit()

# =========================
# GLOBAL VARIABLE
# =========================

map1 = None
map2 = None
roi  = None

# =========================
# INIT — panggil SEKALI SAJA
# =========================

def initialize(width, height):

    global map1, map2, roi

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix,
        dist_coeffs,
        (width, height),
        0,
        (width, height)
    )

    # Precompute mapping — lebih cepat dari cv2.undistort()
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix,
        dist_coeffs,
        None,
        new_camera_matrix,
        (width, height),
        cv2.CV_16SC2    # format optimal untuk remap
    )

# =========================
# UNDISTORT — pakai remap
# =========================

def undistort_frame(frame, crop=True):

    global map1, map2, roi

    if map1 is None or map2 is None:
        raise RuntimeError(
            "undistortion.initialize() belum dipanggil! "
            "Panggil initialize(width, height) sebelum undistort_frame()."
        )

    # Remap jauh lebih cepat dari undistort()
    undistorted = cv2.remap(
        frame,
        map1,
        map2,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT
    )

    x, y, w_roi, h_roi = roi

    if crop and w_roi > 0 and h_roi > 0:
        undistorted = undistorted[y:y+h_roi, x:x+w_roi]

    return undistorted


# =========================
# PREVIEW STREAM — CEK KALIBRASI & CROP
# =========================
# Jalankan file ini langsung (bukan lewat main.py) untuk melihat stream
# kamera live berdampingan: ASLI (distorsi) | UNDISTORT (full, kotak merah
# = area crop hasil getOptimalNewCameraMatrix) | UNDISTORT + CROP (hasil
# akhir yang dipakai main.py). Berguna untuk cek apakah kalibrasi sudah
# pas (garis lurus tidak lagi melengkung) dan apakah crop ROI sudah wajar
# (tidak memotong area kerja yang dibutuhkan).
#
#   python undistortion.py
#
# Kontrol:
#   Q -> keluar
#   G -> toggle grid bantu (garis hijau harus lurus kalau kalibrasi benar)
#   S -> simpan screenshot ke folder ini (preview_*.png)

def _draw_grid(img, spacing=40, color=(0, 255, 0)):
    h, w = img.shape[:2]
    out = img.copy()
    for gx in range(0, w, spacing):
        cv2.line(out, (gx, 0), (gx, h), color, 1, cv2.LINE_AA)
    for gy in range(0, h, spacing):
        cv2.line(out, (0, gy), (w, gy), color, 1, cv2.LINE_AA)
    return out


def _label(img, text):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(
        out, text, (6, 18),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA
    )
    return out


def _run_preview():
    try:
        import config
        cam_index = getattr(config, "CAM_INDEX", 0)
        cam_width = getattr(config, "CAM_WIDTH", 640)
        cam_height = getattr(config, "CAM_HEIGHT", 480)
    except Exception:
        cam_index, cam_width, cam_height = 0, 640, 480

    try:
        from camera_thread import CameraThread
        cam = CameraThread(cam_index, cam_width, cam_height)
        read_frame = cam.read
        release_cam = cam.release
    except Exception:
        try:
            from camera_source import open_camera
            cap = open_camera(cam_width, cam_height)
        except RuntimeError as exc:
            print(f"ERROR: {exc}")
            return

        def read_frame():
            ret, f = cap.read()
            return f if ret else None

        release_cam = cap.release

    frame = None
    for _ in range(60):
        frame = read_frame()
        if frame is not None:
            break
        cv2.waitKey(50)

    if frame is None:
        print("ERROR: Frame awal gagal dibaca dari kamera.")
        release_cam()
        return

    initialize(frame.shape[1], frame.shape[0])
    x, y, w_roi, h_roi = roi
    print(f"[UNDISTORTION PREVIEW] Resolusi kamera: {frame.shape[1]}x{frame.shape[0]}")
    print(f"[UNDISTORTION PREVIEW] ROI crop hasil kalibrasi: x={x} y={y} w={w_roi} h={h_roi}")
    print("[UNDISTORTION PREVIEW] Q=keluar | G=toggle grid | S=screenshot")

    show_grid = False
    shot_count = 0

    while True:
        frame = read_frame()
        if frame is None:
            continue

        full_undistorted = undistort_frame(frame, crop=False)
        cropped = undistort_frame(frame, crop=True)

        # Gambar kotak ROI crop di atas hasil undistort penuh, biar kelihatan
        # bagian mana yang akan dibuang oleh crop.
        full_with_box = full_undistorted.copy()
        if w_roi > 0 and h_roi > 0:
            cv2.rectangle(full_with_box, (x, y), (x + w_roi, y + h_roi), (0, 0, 255), 2)

        panel_original = frame
        panel_full = full_with_box
        panel_cropped = cropped

        if show_grid:
            panel_original = _draw_grid(panel_original)
            panel_full = _draw_grid(panel_full)
            panel_cropped = _draw_grid(panel_cropped)

        target_h = panel_original.shape[0]

        def _resize_to_h(img, h):
            iw = img.shape[1]
            ih = img.shape[0]
            new_w = max(1, int(iw * (h / ih)))
            return cv2.resize(img, (new_w, h))

        panel_full_r = _resize_to_h(panel_full, target_h)
        panel_cropped_r = _resize_to_h(panel_cropped, target_h)

        panel_original = _label(panel_original, "ASLI (distorsi)")
        panel_full_r = _label(panel_full_r, "UNDISTORT (kotak merah = area crop)")
        panel_cropped_r = _label(panel_cropped_r, "UNDISTORT + CROP (dipakai main.py)")

        combined = cv2.hconcat([panel_original, panel_full_r, panel_cropped_r])
        cv2.imshow("Undistortion Preview - cek kalibrasi & crop", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key in (ord("g"), ord("G")):
            show_grid = not show_grid
        elif key in (ord("s"), ord("S")):
            shot_count += 1
            fname = f"preview_{shot_count:02d}.png"
            cv2.imwrite(fname, combined)
            print(f"[UNDISTORTION PREVIEW] Screenshot disimpan: {fname}")

    release_cam()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    _run_preview()
