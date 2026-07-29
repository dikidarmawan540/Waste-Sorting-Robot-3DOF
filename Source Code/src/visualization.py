import cv2
import numpy as np

import config


def _unletterbox_array(arr: np.ndarray, meta: dict | None) -> np.ndarray:
    """Ambil kembali area asli ROI dari output/mask YOLO yang memakai letterbox."""
    if not meta or not bool(meta.get("enabled", False)):
        return arr

    h, w = arr.shape[:2]
    target_w = float(meta.get("target_w", w) or w)
    target_h = float(meta.get("target_h", h) or h)

    # result.plot() biasanya 640x640, sedangkan mask tensor bisa ikut resolusi internal.
    # Karena itu padding dari meta diskalakan ke ukuran array aktual.
    sx = w / target_w
    sy = h / target_h

    pad_x = int(round(float(meta.get("pad_x", 0)) * sx))
    pad_y = int(round(float(meta.get("pad_y", 0)) * sy))
    new_w = int(round(float(meta.get("new_w", target_w)) * sx))
    new_h = int(round(float(meta.get("new_h", target_h)) * sy))

    x1 = max(0, min(pad_x, w - 1))
    y1 = max(0, min(pad_y, h - 1))
    x2 = max(x1 + 1, min(x1 + max(1, new_w), w))
    y2 = max(y1 + 1, min(y1 + max(1, new_h), h))
    return arr[y1:y2, x1:x2]


def draw_segmentation_batch(
    frame_out,
    result,
    roi_w: int,
    roi_h: int,
    offset_x: int = 0,
    offset_y: int = 0,
    letterbox_meta: dict | None = None,
) -> None:
    """
    Render mask segmentasi lalu tempel ke frame penuh di posisi ROI.

    Jika input YOLO memakai letterbox, area padding dibuang dulu sebelum
    di-resize ke ukuran ROI asli. Ini mencegah visual mask bergeser/gepeng.
    """
    plotted = result.plot(
        conf=True,
        labels=True,
        masks=True,
        boxes=False,
    )

    plotted_unpad = _unletterbox_array(plotted, letterbox_meta)
    plotted_roi = cv2.resize(plotted_unpad, (roi_w, roi_h), interpolation=cv2.INTER_LINEAR)

    frame_out[
        offset_y : offset_y + roi_h,
        offset_x : offset_x + roi_w,
    ] = plotted_roi


def draw_centroid_esp32(
    frame_out,
    masks_data,
    boxes_data,
    class_names,
    roi_w: int,
    roi_h: int,
    offset_x: int = 0,
    offset_y: int = 0,
    letterbox_meta: dict | None = None,
) -> list:
    """
    Hitung centroid setiap objek dari mask, gambar lingkaran + koordinat
    di frame penuh, dan kembalikan list centroid (dalam koordinat frame penuh)
    untuk dikirim ke ESP32.

    Return
    ------
    centroids : list of dict {cls_id, class_name, conf, cx, cy}
    """
    centroids = []

    if masks_data is None or boxes_data is None or len(boxes_data) == 0:
        return centroids

    for i in range(len(boxes_data)):
        cls_id     = int(boxes_data.cls[i])
        conf       = float(boxes_data.conf[i])
        if isinstance(class_names, dict):
            class_name = class_names.get(cls_id, str(cls_id))
        else:
            class_name = class_names[cls_id] if cls_id < len(class_names) else str(cls_id)
        color      = config.MASK_COLORS[cls_id % len(config.MASK_COLORS)]

        # Jika input YOLO memakai letterbox, buang padding dulu agar centroid
        # dihitung dari bentuk objek asli, bukan dari kanvas 640x640 berpadded.
        mask_source = _unletterbox_array(masks_data[i], letterbox_meta)

        # Resize mask ke ukuran ROI asli
        mask_resized = cv2.resize(
            mask_source,
            (roi_w, roi_h),
            interpolation=cv2.INTER_LINEAR,
        )
        binary_mask = (mask_resized > 0.5).astype(np.uint8)

        if binary_mask.sum() == 0:
            continue

        ys, xs = np.where(binary_mask > 0)
        if xs.size == 0 or ys.size == 0:
            continue

        M = cv2.moments(binary_mask)
        if M["m00"] == 0:
            continue

        cx_roi = int(M["m10"] / M["m00"])
        cy_roi = int(M["m01"] / M["m00"])

        # Koreksi ke koordinat frame penuh
        cx = cx_roi + offset_x
        cy = cy_roi + offset_y
        x1_full = int(xs.min()) + offset_x
        y1_full = int(ys.min()) + offset_y
        x2_full = int(xs.max()) + offset_x
        y2_full = int(ys.max()) + offset_y

        # Gambar centroid saja. Label pixel (cx,cy) sengaja tidak ditampilkan
        # supaya tampilan tidak rancu dengan koordinat robot.
        cv2.circle(frame_out, (cx, cy), 7, (0, 0, 255), -1)

        centroids.append({
            "det_index":   i,
            "cls_id":     cls_id,
            "class_name": class_name,
            "conf":       conf,
            "cx":         cx,
            "cy":         cy,
            "x1":         x1_full,
            "y1":         y1_full,
            "x2":         x2_full,
            "y2":         y2_full,
            "bbox":       [x1_full, y1_full, x2_full, y2_full],
            "mask_area_px": int(binary_mask.sum()),
        })

    return centroids


def draw_overlay_info(
    frame_out,
    avg_fps: float,
    mouse_x: int | None = None,
    mouse_y: int | None = None,
    esp32_status: str = "OFF",
    mouse_robot=None,
    pick_id=None,
    pick_class=None,
) -> None:
    """
    Gambar overlay ringkas di posisi yang diatur config.

    Sesuai revisi tampilan, overlay hanya menampilkan FPS dan informasi
    objek yang akan diproses pick-and-place: ID dan kelas. Teks debug lama
    seperti koordinat robot mouse, Conf, Engine, dan ESP32 tidak ditampilkan
    lagi agar frame kamera tetap bersih.

    Default revisi: kiri bawah, supaya area atas conveyor/ROI tetap terlihat.
    Parameter mouse_x, mouse_y, esp32_status, dan mouse_robot tetap diterima
    untuk kompatibilitas dengan pemanggilan lama.
    """
    scale = float(getattr(config, "OVERLAY_TEXT_SCALE", 0.45))
    thickness = int(getattr(config, "OVERLAY_TEXT_THICKNESS", 1))
    line_gap = int(getattr(config, "OVERLAY_TEXT_LINE_GAP", 24))
    color = (0, 255, 255)

    if pick_id is None or str(pick_id).strip() == "":
        id_text = "--"
    else:
        try:
            id_text = f"{int(pick_id):02d}"
        except Exception:
            id_text = str(pick_id)

    class_text = str(pick_class) if pick_class not in (None, "") else "--"

    lines = [
        f"FPS: {avg_fps:.1f}",
        f"ID: {id_text}   Kelas: {class_text}",
    ]

    h, w = frame_out.shape[:2]
    margin_x = int(getattr(config, "OVERLAY_TEXT_MARGIN_X", 10))
    margin_bottom = int(getattr(config, "OVERLAY_TEXT_MARGIN_BOTTOM", 12))
    position = str(getattr(config, "OVERLAY_TEXT_POSITION", "BOTTOM_LEFT")).strip().upper()

    x = max(0, margin_x)
    if position == "TOP_LEFT":
        y = max(18, margin_bottom + 14)
    else:
        # Baseline baris pertama diletakkan cukup tinggi agar baris terakhir
        # masih aman di dalam frame bagian bawah.
        y = h - max(8, margin_bottom) - (len(lines) - 1) * line_gap
        y = max(18, min(y, h - 8))

    for i, text in enumerate(lines):
        cv2.putText(
            frame_out,
            text,
            (x, y + i * line_gap),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )


def draw_class_counters(
    frame_out,
    class_counts: dict,
    class_order: list,
) -> None:
    """Gambar teks counting per kelas (Kaca/Kertas/Logam/Plastik) di stream.

    class_counts : dict {class_name: total_count}
    class_order  : urutan nama kelas yang ingin ditampilkan (mis. dari
                   config.WASTE_CLASS_NAMES)
    """
    scale = float(getattr(config, "OVERLAY_TEXT_SCALE", 0.45))
    thickness = int(getattr(config, "OVERLAY_TEXT_THICKNESS", 1))
    line_gap = int(getattr(config, "CLASS_COUNTERS_LINE_GAP", 22))
    color = (255, 255, 0)

    header = "COUNTING:"
    lines = [header] + [
        f"{name}: {int(class_counts.get(name, 0))}" for name in class_order
    ]

    h, w = frame_out.shape[:2]
    margin_x = int(getattr(config, "CLASS_COUNTERS_MARGIN_X", 10))
    margin_y = int(getattr(config, "CLASS_COUNTERS_MARGIN_Y", 24))
    position = str(getattr(config, "CLASS_COUNTERS_POSITION", "TOP_RIGHT")).strip().upper()

    # Perkiraan lebar teks terpanjang untuk perataan kanan.
    max_text_w = 0
    for text in lines:
        (text_w, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
        max_text_w = max(max_text_w, text_w)

    if position == "TOP_LEFT":
        x = margin_x
        y = margin_y
    elif position == "BOTTOM_LEFT":
        x = margin_x
        y = h - margin_y - (len(lines) - 1) * line_gap
    elif position == "BOTTOM_RIGHT":
        x = max(margin_x, w - margin_x - max_text_w)
        y = h - margin_y - (len(lines) - 1) * line_gap
    else:  # TOP_RIGHT (default)
        x = max(margin_x, w - margin_x - max_text_w)
        y = margin_y

    for i, text in enumerate(lines):
        cv2.putText(
            frame_out,
            text,
            (x, y + i * line_gap),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )
