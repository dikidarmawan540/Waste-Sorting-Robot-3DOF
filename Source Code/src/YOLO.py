import os
import sys
import numpy as np
import torch
from ultralytics import YOLO

import config


def get_device() -> int | str:
    """
    Deteksi ketersediaan GPU.
    Return 0 (GPU index) jika CUDA tersedia, "cpu" jika tidak.
    """
    print("\n==============================")

    if torch.cuda.is_available():
        device = 0
        print(f"GPU: {torch.cuda.get_device_name(0)}")
    else:
        device = "cpu"
        print("GPU tidak terdeteksi, menggunakan CPU")

    print("==============================\n")
    return device


def configured_class_names() -> dict[int, str]:
    """
    Ambil nama kelas dari config tanpa menulis ke properti YOLO.names.

    Catatan penting:
    Pada beberapa versi Ultralytics, property `model.names` bersifat read-only
    sehingga assignment langsung seperti `model.names = ...` menyebabkan error:
    `property 'names' of 'YOLO' object has no setter`.
    """
    configured_names = list(getattr(config, "WASTE_CLASS_NAMES", []))
    configured_nc = int(getattr(config, "WASTE_NC", len(configured_names)))

    if configured_names and len(configured_names) != configured_nc:
        print(
            f"WARNING: WASTE_NC={configured_nc}, tetapi jumlah WASTE_CLASS_NAMES="
            f"{len(configured_names)}. Pakai jumlah names aktual."
        )

    return {i: name for i, name in enumerate(configured_names)}


def load_model(device: int | str) -> YOLO:
    """
    Validasi path model, load engine TensorRT, lalu lakukan warmup FP16.
    """
    if not os.path.isfile(config.MODEL_PATH):
        print(f"ERROR: File model tidak ditemukan:\n{config.MODEL_PATH}")
        sys.exit()

    try:
        model = YOLO(config.MODEL_PATH, task="segment")

        # Jangan assignment ke model.names karena di Ultralytics terbaru read-only.
        # Nama kelas custom disimpan di atribut terpisah, lalu dipakai oleh main.py
        # untuk overlay, centroid, dan mapping class-id -> bin ESP32.
        rbtt_names = configured_class_names()
        if rbtt_names:
            setattr(model, "rbtt_class_names", rbtt_names)
            print(
                "Model class map aktif dari config: "
                f"nc={len(rbtt_names)} names={list(rbtt_names.values())}"
            )

        print("Model TensorRT berhasil dimuat.")
        print("Melakukan warmup...")

        # Warmup dengan dummy frame, half=True sesuai engine FP16
        model.predict(
            np.zeros((config.YOLO_HEIGHT, config.YOLO_WIDTH, 3), dtype=np.uint8),
            device=device,
            verbose=False,
            half=True,
        )
        print("Warmup selesai.")

    except Exception as e:
        print("ERROR: Gagal memuat model.")
        print(e)
        sys.exit()

    return model


# =========================
# PREVIEW STREAM — CEK ROBUSTNESS YOLO
# =========================
# Jalankan file ini langsung (bukan lewat main.py) untuk melihat stream
# kamera live dengan deteksi YOLO berjalan, TANPA logic tracking/ESP32.
# Berguna untuk mengecek apakah model sudah robust (deteksi stabil, class
# benar, confidence wajar) sebelum dipakai penuh di main.py.
#
# Pipeline yang dipakai sama persis dengan main.py:
#   kamera -> undistort_frame() -> roi.crop() (ROI 4 titik) ->
#   letterbox_for_yolo() -> model.predict() -> draw_segmentation_batch()
#
#   python YOLO.py
#
# Kontrol:
#   Q     -> keluar
#   + / - -> naik/turun CONF_THRESHOLD live (default dari config.py)
#   G     -> toggle grid/garis ROI
#   K     -> kirim G1 ke titik deteksi terdekat pusat frame (Z=config.K_TEST_MOVE_Z_MM)
#            + catat Estimasi X/Y ke config.K_TEST_LOG_XLSX_PATH
#   H     -> homing (G28)

def _run_preview():
    import time
    from collections import deque

    import cv2
    import config as _config
    from camera_thread import CameraThread
    from undistortion import initialize as u_initialize, undistort_frame
    from visualization import draw_segmentation_batch, draw_centroid_esp32
    import homography as _homography
    import roi as _roi
    import k_test_logger as _k_test_logger

    k_test_enable = bool(getattr(_config, "K_TEST_ENABLE", True))
    esp32 = None
    if k_test_enable:
        try:
            from esp32_comm import ESP32Comm
            esp32 = ESP32Comm(auto_connect=True)
            if esp32.is_connected:
                print("[YOLO PREVIEW] Tombol K/H aktif: ESP32 terhubung, siap kirim G1/G28.")
            else:
                print("[YOLO PREVIEW] Tombol K/H aktif tapi ESP32 BELUM terhubung (cek SERIAL_PORT).")
        except Exception as e:
            print(f"[YOLO PREVIEW] Gagal siapkan ESP32Comm untuk tombol K/H: {e}")
            esp32 = None

    device = get_device()
    model = load_model(device)
    class_names = getattr(model, "rbtt_class_names", getattr(model, "names", {}))

    cam = CameraThread(_config.CAM_INDEX, _config.CAM_WIDTH, _config.CAM_HEIGHT)

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
    _test = undistort_frame(frame)
    undist_h, undist_w = _test.shape[:2]
    del _test

    _roi.initialize(undist_w, undist_h)

    homography_ready = False
    try:
        _homography.initialize()
        homography_ready = True
        print(f"[YOLO PREVIEW] Homography aktif: {_homography.coordinate_space()} | file={getattr(_config, 'HOMOGRAPHY_PATH', '-')}")
    except Exception as e:
        print(f"[YOLO PREVIEW] Homography belum aktif: {e}")
        print("[YOLO PREVIEW] Overlay koordinat robot dinonaktifkan sampai file homography tersedia.")

    conf_threshold = float(getattr(_config, "CONF_THRESHOLD", 0.75))
    iou_threshold = float(getattr(_config, "IOU_THRESHOLD", 0.7))
    use_letterbox_roi = bool(getattr(_config, "YOLO_USE_LETTERBOX_ROI", True))
    show_roi_border = True

    fps_deque = deque(maxlen=30)
    prev_time = time.perf_counter()

    print("[YOLO PREVIEW] Q=keluar | +/-=ubah conf threshold | G=toggle ROI border")
    if homography_ready and bool(getattr(_config, "YOLO_PREVIEW_SHOW_ROBOT_COORDS", True)):
        print("[YOLO PREVIEW] Overlay koordinat robot per deteksi: AKTIF")

    while True:
        frame = cam.read()
        if frame is None:
            continue

        frame_original = undistort_frame(frame)
        frame_roi, roi_offset_x, roi_offset_y = _roi.crop(frame_original)
        roi_h, roi_w = frame_roi.shape[:2]

        if use_letterbox_roi:
            frame_yolo_input, letterbox_meta = _roi.letterbox_for_yolo(
                frame_roi,
                _config.YOLO_WIDTH,
                _config.YOLO_HEIGHT,
                color=getattr(_config, "YOLO_LETTERBOX_COLOR", 114),
            )
        else:
            frame_yolo_input = cv2.resize(frame_roi, (_config.YOLO_WIDTH, _config.YOLO_HEIGHT))
            letterbox_meta = _roi.no_letterbox_meta(roi_w, roi_h, _config.YOLO_WIDTH, _config.YOLO_HEIGHT)

        try:
            results = model.predict(
                source=frame_yolo_input,
                conf=conf_threshold,
                iou=iou_threshold,
                device=device,
                verbose=False,
                half=True,
            )
        except Exception as e:
            print("ERROR: Inferensi YOLO gagal.")
            print(e)
            break

        result = results[0]
        try:
            result.names = class_names
        except Exception:
            pass

        frame_output = frame_original
        draw_segmentation_batch(
            frame_output,
            result,
            roi_w,
            roi_h,
            offset_x=roi_offset_x,
            offset_y=roi_offset_y,
            letterbox_meta=letterbox_meta,
        )

        centroids = []
        if getattr(_config, "YOLO_PREVIEW_SHOW_CENTROIDS", True):
            centroids = draw_centroid_esp32(
                frame_output,
                getattr(result, "masks", None).data.cpu().numpy() if getattr(result, "masks", None) is not None else None,
                result.boxes,
                class_names,
                roi_w,
                roi_h,
                offset_x=roi_offset_x,
                offset_y=roi_offset_y,
                letterbox_meta=letterbox_meta,
            )

        if show_roi_border:
            _roi.draw_roi_border(frame_output)

        if homography_ready and bool(getattr(_config, "YOLO_PREVIEW_SHOW_ROBOT_COORDS", True)):
            coord_scale = float(getattr(_config, "YOLO_PREVIEW_COORD_SCALE", 0.42))
            coord_thickness = int(getattr(_config, "YOLO_PREVIEW_COORD_THICKNESS", 1))
            coord_color = tuple(getattr(_config, "YOLO_PREVIEW_COORD_COLOR", (0, 255, 255)))
            coord_gap = int(getattr(_config, "YOLO_PREVIEW_COORD_LINE_GAP", 16))
            hide_outside = bool(getattr(_config, "YOLO_PREVIEW_HIDE_COORDS_OUTSIDE_WORKSPACE", False))

            for det in centroids:
                try:
                    x_robot_mm, y_robot_mm = _homography.pixel_to_robot(det["cx"], det["cy"])
                    inside_ws = _homography.is_inside_workspace(x_robot_mm, y_robot_mm)
                    if hide_outside and not inside_ws:
                        continue

                    text1 = f"X: {x_robot_mm:.1f} mm"
                    text2 = f"Y: {y_robot_mm:.1f} mm"
                    base_x = int(det["cx"]) + 10
                    base_y = int(det["cy"]) - 10
                    if base_y < 24:
                        base_y = int(det["cy"]) + 18

                    draw_color = coord_color if inside_ws else (0, 140, 255)
                    cv2.putText(frame_output, text1, (base_x, base_y), cv2.FONT_HERSHEY_SIMPLEX, coord_scale, draw_color, coord_thickness, cv2.LINE_AA)
                    cv2.putText(frame_output, text2, (base_x, base_y + coord_gap), cv2.FONT_HERSHEY_SIMPLEX, coord_scale, draw_color, coord_thickness, cv2.LINE_AA)
                except Exception:
                    continue

        n_detections = 0 if result.boxes is None else len(result.boxes)

        current_time = time.perf_counter()
        elapsed = current_time - prev_time
        if elapsed > 0:
            fps_deque.append(1.0 / elapsed)
        avg_fps = sum(fps_deque) / len(fps_deque) if fps_deque else 0
        prev_time = current_time

        status_scale = float(getattr(_config, "YOLO_PREVIEW_STATUS_SCALE", 0.55))
        status_thickness = int(getattr(_config, "YOLO_PREVIEW_STATUS_THICKNESS", 1))
        status_color = tuple(getattr(_config, "YOLO_PREVIEW_STATUS_COLOR", (0, 255, 0)))
        margin_x = int(getattr(_config, "YOLO_PREVIEW_STATUS_MARGIN_X", 10))
        margin_bottom = int(getattr(_config, "YOLO_PREVIEW_STATUS_MARGIN_BOTTOM", 12))
        status_text = f"FPS: {avg_fps:.1f} | Conf: {conf_threshold:.2f} | Deteksi: {n_detections}"
        text_y = max(18, frame_output.shape[0] - margin_bottom)
        cv2.putText(
            frame_output,
            status_text,
            (margin_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            status_scale,
            status_color,
            status_thickness,
            cv2.LINE_AA,
        )

        cv2.imshow("YOLO Preview - cek robustness deteksi", frame_output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key in (ord("+"), ord("=")):
            conf_threshold = min(0.99, conf_threshold + 0.05)
            print(f"[YOLO PREVIEW] Conf threshold: {conf_threshold:.2f}")
        elif key == ord("-"):
            conf_threshold = max(0.01, conf_threshold - 0.05)
            print(f"[YOLO PREVIEW] Conf threshold: {conf_threshold:.2f}")
        elif key in (ord("g"), ord("G")):
            show_roi_border = not show_roi_border
        elif key in (ord("h"), ord("H")):
            if not k_test_enable:
                print("[HOMING] Nonaktif (config.K_TEST_ENABLE=False).")
            elif esp32 is None or not esp32.is_connected:
                print("[HOMING] ESP32 belum terhubung, tombol H diabaikan.")
            else:
                print("[HOMING] Mengirim G28...")
                sent = esp32.home()
                print("[HOMING] G28 terkirim." if sent else "[HOMING] Gagal kirim G28.")
        elif key in (ord("k"), ord("K")):
            if not k_test_enable:
                print("[K_TEST] Fitur K nonaktif (config.K_TEST_ENABLE=False).")
            elif not homography_ready:
                print("[K_TEST] Homography belum aktif, tidak bisa hitung koordinat robot.")
            elif not centroids:
                print("[K_TEST] Tidak ada deteksi saat ini, tombol K diabaikan.")
            else:
                # Pilih deteksi TERDEKAT ke pusat frame.
                fh, fw = frame_output.shape[:2]
                center_x, center_y = fw / 2.0, fh / 2.0
                best = min(
                    centroids,
                    key=lambda d: (d["cx"] - center_x) ** 2 + (d["cy"] - center_y) ** 2,
                )
                try:
                    x_robot_mm, y_robot_mm = _homography.pixel_to_robot(best["cx"], best["cy"])
                    z_test_mm = float(getattr(_config, "K_TEST_MOVE_Z_MM", 20.0))

                    # Pencatatan CSV/XLSX SELALU jalan asal ada deteksi + homography,
                    # tidak digantungkan ke status koneksi/kirim ESP32.
                    path, trial_no = _k_test_logger.append_row(_config, x_robot_mm, y_robot_mm)

                    if esp32 is not None and esp32.is_connected:
                        sent = esp32.move_g1(x_mm=x_robot_mm, y_mm=y_robot_mm, z_mm=z_test_mm)
                        move_status = (
                            f"G1 X{x_robot_mm:.1f} Y{y_robot_mm:.1f} Z{z_test_mm:.1f} "
                            + ("terkirim" if sent else "GAGAL kirim")
                        )
                    else:
                        move_status = "ESP32 belum terhubung, G1 tidak dikirim"

                    print(
                        f"[K_TEST] Percobaan {trial_no}: Estimasi X{x_robot_mm:.1f} Y{y_robot_mm:.1f} "
                        f"dicatat ke {os.path.basename(path)} | {move_status}"
                    )
                    print(
                        "     Aktual X/Y silakan isi manual di kolom D/E baris "
                        f"{trial_no + 2} pada {os.path.basename(path)} (Excel harus ditutup "
                        "dulu selama YOLO.py masih jalan)."
                    )
                except Exception as e:
                    print(f"[K_TEST] Gagal proses tombol K: {e}")

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    _run_preview()
