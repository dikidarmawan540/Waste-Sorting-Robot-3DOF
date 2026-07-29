import time
import math
import threading
import queue
from collections import deque

import cv2
import numpy as np

import config
from YOLO import get_device, load_model
from camera_thread import CameraThread
from undistortion import initialize, undistort_frame
from visualization import (
    draw_segmentation_batch,
    draw_centroid_esp32,
    draw_overlay_info,
    draw_class_counters,
)
from esp32_comm import ESP32Comm
from conveyor import ConveyorController
import homography
import roi
from tracking_utils import (
    use_bytetrack,
    bytetrack_config_path,
    bytetrack_predict_conf,
    update_tracks_from_bytetrack,
)
import waste_counter

# =========================
# UTIL: TERMINAL COMMAND MODE
# =========================

terminal_commands: "queue.Queue[str]" = queue.Queue()
exit_requested = False


def terminal_input_worker():
    """Baca command dari terminal VS Code tanpa menghentikan kamera.

    Contoh saat main.py berjalan:
      G28
      HELP
      M114
      YOLO
      PUMP OFF
      SORT X0 Y180 Z40 B0
      EXIT
    """
    print("\n[TERMINAL COMMAND MODE]")
    print("  Ketik command ESP32 di terminal lalu Enter, contoh: G28, HELP, M114, YOLO, PUMP OFF")
    print("  Ketik EXIT/QUIT/Q untuk keluar dari main.py dan mematikan komunikasi ESP32.\n")
    while True:
        try:
            line = input().strip()
        except EOFError:
            break
        except KeyboardInterrupt:
            terminal_commands.put("EXIT")
            break
        if line:
            terminal_commands.put(line)


def process_terminal_commands(esp32: ESP32Comm, conveyor: ConveyorController | None = None) -> bool:
    """Kirim semua command terminal yang sedang antre. Return True jika minta exit."""
    want_exit = False
    while True:
        try:
            cmd = terminal_commands.get_nowait().strip()
        except queue.Empty:
            break

        if not cmd:
            continue

        upper = cmd.upper()
        if upper in {"Q", "QUIT", "EXIT"}:
            print("[TERMINAL] EXIT diterima. Keluar dari main.py...")
            want_exit = True
            continue

        if upper.startswith("CONV ") or upper.startswith("CONVEYOR "):
            subcmd = cmd.split(maxsplit=1)[1].strip() if len(cmd.split(maxsplit=1)) > 1 else ""
            sub_upper = subcmd.upper()
            if conveyor is None:
                print("[CONVEYOR] Controller belum aktif.")
                continue
            if sub_upper == "START":
                conveyor.start()
            elif sub_upper == "STOP":
                conveyor.stop()
            elif sub_upper == "STATUS":
                print(f"[CONVEYOR] {conveyor.status}")
            elif sub_upper == "READ":
                conveyor.read_available(duration=2.0, echo=True)
            elif sub_upper == "RECONNECT":
                conveyor.reconnect()
            elif subcmd:
                conveyor.send_raw(subcmd)
                conveyor.read_available(duration=0.20, echo=True)
            else:
                print("[CONVEYOR] Pakai: CONV START / CONV STOP / CONV STATUS / CONV F / CONV R")
            continue

        if upper in {"LOCALHELP", "LHELP"}:
            print("[TERMINAL HELP]")
            print("  Command dikirim langsung ke ESP32 robot. Contoh: HELP, G28, M114, M119, INA, YOLO, PUMP ON/OFF")
            print("  Command conveyor: CONV START, CONV STOP, CONV STATUS, CONV F, CONV R")
            print("  EXIT/QUIT/Q = keluar dari main.py")
            continue

        # Kirim apa adanya ke ESP32. Ini membuat main.py tidak hanya bergantung hotkey.
        print(f"[TERMINAL -> ESP32] {cmd}")
        ok = esp32.send_line(cmd, force=True)
        if ok:
            # Baca respons singkat agar HELP/M114/YOLO langsung terlihat di terminal.
            # Untuk motion panjang, respons lanjutan tetap akan terbaca oleh polling serial di loop utama.
            esp32.read_available(duration=0.25, echo=True)
    return want_exit


def safe_shutdown(cam: CameraThread | None, esp32: ESP32Comm | None, conveyor: ConveyorController | None = None):
    """Pastikan conveyor berhenti, relay/pump mati, port serial ditutup, dan kamera dilepas."""
    print("\n[SHUTDOWN] Menutup main.py...")

    if conveyor is not None:
        try:
            print("[SHUTDOWN] Stop conveyor dan tutup komunikasi conveyor...")
            conveyor.stop()
            conveyor.close()
        except Exception as e:
            print(f"[SHUTDOWN] Gagal stop/tutup conveyor: {e}")

    if esp32 is not None:
        try:
            if bool(getattr(config, "PUMP_OFF_ON_EXIT", True)):
                print("[SHUTDOWN] Kirim PUMP OFF ke ESP32...")
                esp32.send_line("PUMP OFF", force=True)
                esp32.read_available(duration=0.30, echo=True)
        except Exception as e:
            print(f"[SHUTDOWN] Gagal kirim PUMP OFF: {e}")

        try:
            print("[SHUTDOWN] Tutup komunikasi serial ESP32.")
            esp32.close()
        except Exception as e:
            print(f"[SHUTDOWN] Gagal menutup serial: {e}")

    if cam is not None:
        try:
            cam.release()
        except Exception:
            pass

    try:
        cv2.destroyAllWindows()
    except Exception:
        pass

    print("[SHUTDOWN] Program selesai. COM port sudah dilepas.")


# =========================
# OBJECT ID TRACKING
# =========================

def update_object_tracks(
    centroids: list[dict],
    tracks: dict[int, dict],
    next_track_id: int,
    frame_index: int,
) -> int:
    """Assign ID stabil berbasis class + jarak centroid antar-frame."""
    if not bool(getattr(config, "TRACKING_ENABLE", True)):
        for idx, obj in enumerate(centroids, start=1):
            obj["track_id"] = idx
        return next_track_id

    max_dist = float(getattr(config, "TRACK_MAX_MATCH_DISTANCE_PX", 80.0))
    max_missed = int(getattr(config, "TRACK_MAX_MISSED_FRAMES", 8))
    purge_missed = int(getattr(config, "TRACK_PURGE_MISSED_FRAMES", 90))

    for tr in tracks.values():
        tr["visible"] = False
        tr["matched_this_frame"] = False

    candidates: list[tuple[float, int, int]] = []
    for det_idx, obj in enumerate(centroids):
        cls_id = int(obj.get("cls_id", -1))
        cx = float(obj.get("cx", 0.0))
        cy = float(obj.get("cy", 0.0))
        for track_id, tr in tracks.items():
            if int(tr.get("cls_id", -999)) != cls_id:
                continue
            if int(tr.get("miss_count", 0)) > max_missed:
                continue
            dist = math.hypot(cx - float(tr.get("cx", cx)), cy - float(tr.get("cy", cy)))
            if dist <= max_dist:
                candidates.append((dist, track_id, det_idx))

    assigned_det: dict[int, int] = {}
    used_tracks: set[int] = set()
    used_dets: set[int] = set()

    for _, track_id, det_idx in sorted(candidates, key=lambda item: item[0]):
        if track_id in used_tracks or det_idx in used_dets:
            continue
        assigned_det[det_idx] = track_id
        used_tracks.add(track_id)
        used_dets.add(det_idx)

    for det_idx, obj in enumerate(centroids):
        if det_idx in assigned_det:
            track_id = assigned_det[det_idx]
            tr = tracks[track_id]
        else:
            track_id = next_track_id
            next_track_id += 1
            tr = {
                "track_id": track_id,
                "first_seen_frame": frame_index,
                "first_seen_order": track_id,
                "seen_count": 0,
                "visible_streak": 0,
                "miss_count": 0,
                "skipped": False,
            }
            tracks[track_id] = tr

        obj["track_id"] = track_id
        in_pick_zone = bool(obj.get("in_pick_zone", False))
        touches_stop_line = bool(obj.get("touches_pick_stop_line", False))
        if in_pick_zone:
            tr["last_in_pick_zone_frame"] = frame_index
        if touches_stop_line:
            tr["last_pick_stop_line_frame"] = frame_index
        tr.update({
            "cls_id": int(obj.get("cls_id", -1)),
            "class_name": str(obj.get("class_name", obj.get("cls_id", "?"))),
            "conf": float(obj.get("conf", 0.0)),
            "cx": float(obj.get("cx", 0.0)),
            "cy": float(obj.get("cy", 0.0)),
            "x1": float(obj.get("x1", obj.get("cx", 0.0))),
            "y1": float(obj.get("y1", obj.get("cy", 0.0))),
            "x2": float(obj.get("x2", obj.get("cx", 0.0))),
            "y2": float(obj.get("y2", obj.get("cy", 0.0))),
            "bbox": obj.get("bbox", [obj.get("cx", 0.0), obj.get("cy", 0.0), obj.get("cx", 0.0), obj.get("cy", 0.0)]),
            "in_pick_zone": in_pick_zone,
            "touches_pick_stop_line": touches_stop_line,
            "last_seen_frame": frame_index,
            "miss_count": 0,
            "visible": True,
            "matched_this_frame": True,
            "seen_count": int(tr.get("seen_count", 0)) + 1,
            "visible_streak": int(tr.get("visible_streak", 0)) + 1,
        })

    for track_id, tr in list(tracks.items()):
        if not bool(tr.get("matched_this_frame", False)):
            tr["miss_count"] = int(tr.get("miss_count", 0)) + 1
            tr["visible"] = False
            tr["visible_streak"] = 0
            tr["in_pick_zone"] = False
            tr["touches_pick_stop_line"] = False
        if int(tr.get("miss_count", 0)) > purge_missed:
            del tracks[track_id]

    return next_track_id


def choose_target_object(
    centroids: list[dict],
    tracks: dict[int, dict],
    skipped_track_ids: set[int],
    sent_track_ids: set[int],
) -> dict | None:
    """Pilih ID stabil yang berada di SELURUH area PICK ZONE.

    Conveyor STOP dipicu oleh stop line, tetapi target pick tidak dibatasi ke
    garis/center agar objek di batas PICK ZONE tetap bisa terangkat penuh.
    """
    stable_min = int(getattr(config, "TRACK_STABLE_MIN_FRAMES", 2))
    candidates: list[dict] = []

    for obj in centroids:
        track_id = int(obj.get("track_id", 0) or 0)
        if track_id <= 0:
            continue
        if track_id in skipped_track_ids or track_id in sent_track_ids:
            continue
        if not bool(obj.get("in_pick_zone", False)):
            continue

        tr = tracks.get(track_id)
        if not tr or not bool(tr.get("visible", False)):
            continue
        if int(tr.get("visible_streak", 0)) < stable_min:
            continue

        candidates.append(obj)

    if not candidates:
        return None

    candidates.sort(
        key=lambda obj: (
            int(tracks[int(obj["track_id"])].get("first_seen_order", int(obj["track_id"]))),
            -int(tracks[int(obj["track_id"])].get("visible_streak", 0)),
            -float(obj.get("conf", 0.0)),
        )
    )
    return candidates[0]




def draw_tracking_labels(frame_out, centroids: list[dict], tracks: dict[int, dict]) -> None:
    if not bool(getattr(config, "TRACK_DRAW_ID", True)):
        return

    for obj in centroids:
        track_id = int(obj.get("track_id", 0) or 0)
        if track_id <= 0:
            continue
        tr = tracks.get(track_id, {})
        miss_count = int(tr.get("miss_count", 0))
        visible_streak = int(tr.get("visible_streak", 0))
        label = f"ID{track_id}"
        scale = float(getattr(config, "OVERLAY_TEXT_SCALE", 0.45))
        thickness = int(getattr(config, "OVERLAY_TEXT_THICKNESS", 1))
        cx = int(obj.get("cx", 0))
        cy = int(obj.get("cy", 0))
        cv2.putText(
            frame_out,
            label,
            (cx + 10, cy + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (0, 255, 255),
            thickness,
            cv2.LINE_AA,
        )


# =========================
# DEVICE & MODEL
# =========================

DEVICE = get_device()
model = load_model(DEVICE)

# Nama kelas untuk overlay dan mapping bin. Jangan bergantung pada model.names
# karena property itu bisa read-only dan metadata engine kadang tidak sesuai dataset.
CLASS_NAMES = getattr(model, "rbtt_class_names", getattr(model, "names", {}))

# =========================
# KAMERA
# =========================

cam = None
esp32 = None
conveyor = None

try:
    cam = CameraThread(config.CAM_INDEX, config.CAM_WIDTH, config.CAM_HEIGHT)

    frame = None
    for _ in range(30):
        frame = cam.read()
        if frame is not None:
            break
        time.sleep(0.03)

    if frame is None:
        print("ERROR: Frame awal gagal dibaca.")
        raise SystemExit

    print(f"Camera Resolution : {frame.shape[1]} x {frame.shape[0]}")

    # =========================
    # UNDISTORTION
    # =========================

    initialize(frame.shape[1], frame.shape[0])
    _test = undistort_frame(frame)
    UNDIST_H, UNDIST_W = _test.shape[:2]
    del _test

    # =========================
    # HOMOGRAPHY: PIXEL -> ROBOT XY
    # =========================

    homography.initialize()
    print(f"[MAIN] Koordinat aktif: {homography.coordinate_space()} | main.py mengirim X_robot,Y_robot ke ESP32")

    # =========================
    # ROI
    # =========================

    roi.initialize(UNDIST_W, UNDIST_H)
    roi.initialize_pick_zone(UNDIST_W, UNDIST_H, roi_rect=roi.rect())

    # =========================
    # ESP32 ROBOT + CONVEYOR
    # =========================

    esp32 = ESP32Comm()
    conveyor = ConveyorController(auto_connect=True)

    # Conveyor TIDAK langsung jalan saat main.py mulai.
    # Conveyor baru START setelah homing robot selesai dikonfirmasi firmware
    # melalui marker ROBOT_HOMED_MARKERS di config.py, misalnya "G28 DONE".
    try:
        conveyor.stop()
    except Exception as e:
        print(f"[CONVEYOR] Gagal memastikan STOP awal: {e}")

    print("""
[MAIN ESP32 HOTKEY]
  H  : kirim G28 / homing
  ?  : kirim HELP ke ESP32
  M  : kirim M114 / cek posisi
  L  : kirim M119 / cek limit switch
  I  : kirim INA / cek arus
  Y  : kirim YOLO / cek status koordinat YOLO terakhir
  O  : PUMP ON
  P  : PUMP OFF
  C  : conveyor START
  S  : conveyor STOP
  R  : pilih ulang ROI deteksi YOLO lalu PICK ZONE (2 tahap)
  T  : pilih ulang PICK ZONE saja
  Q  : keluar, stop conveyor, kirim PUMP OFF, tutup serial ESP32

[TERMINAL COMMAND]
  Saat main.py berjalan, Anda juga bisa mengetik command langsung di terminal VS Code:
  G28, HELP, M114, M119, INA, YOLO, PUMP OFF, SORT X0 Y180 Z40 B0, dll.
  Untuk conveyor: CONV START, CONV STOP, CONV STATUS, CONV F, CONV R.

Catatan: jangan buka Arduino Serial Monitor saat main.py berjalan.
""")

    if bool(getattr(config, "TERMINAL_COMMAND_ENABLE", True)):
        threading.Thread(target=terminal_input_worker, daemon=True).start()

    # =========================
    # MOUSE & WINDOW
    # =========================

    mouse_x = 0
    mouse_y = 0

    def mouse_position(event, x, y, flags, param):
        nonlocal_vars = param
        # param adalah dict agar callback bisa menulis tanpa global.
        if event == cv2.EVENT_MOUSEMOVE:
            nonlocal_vars["x"] = x
            nonlocal_vars["y"] = y

    mouse_state = {"x": 0, "y": 0}
    cv2.namedWindow(config.WINDOW_MAIN)
    cv2.setMouseCallback(config.WINDOW_MAIN, mouse_position, mouse_state)

    # =========================
    # FPS, YOLO BUFFER, SERIAL TIMING
    # =========================

    fps_deque = deque(maxlen=30)
    prev_time = time.perf_counter()

    _roi_x, _roi_y, _roi_w, _roi_h = roi.rect()
    frame_yolo_buf = np.empty((config.YOLO_HEIGHT, config.YOLO_WIDTH, 3), dtype=np.uint8)
    use_letterbox_roi = bool(getattr(config, "YOLO_USE_LETTERBOX_ROI", True))
    bytetrack_enabled = bool(getattr(config, "TRACKING_ENABLE", True)) and use_bytetrack(config)
    bytetrack_tracker = bytetrack_config_path(config)
    print(f"[TRACKING] Backend: {'BYTETRACK' if bytetrack_enabled else 'CENTROID_FALLBACK'}")
    if bytetrack_enabled:
        print(f"[TRACKING] Tracker config: {bytetrack_tracker}")

    last_serial_poll = 0.0
    serial_poll_interval = float(getattr(config, "SERIAL_READ_INTERVAL", 0.10))
    serial_read_duration = float(getattr(config, "SERIAL_READ_DURATION", 0.001))

    last_yolo_send_time = 0.0
    last_sent_xy: tuple[float, float] | None = None
    yolo_send_interval = float(getattr(config, "YOLO_SEND_INTERVAL", 0.35))
    yolo_resend_distance = float(getattr(config, "YOLO_RESEND_DISTANCE_MM", 10.0))
    yolo_force_resend_sec = float(getattr(config, "YOLO_FORCE_RESEND_SEC", 1.20))

    # =========================
    # OBJECT ID / ROBOT BUSY STATE
    # =========================

    frame_index = 0
    object_tracks: dict[int, dict] = {}
    next_track_id = 1
    skipped_track_ids: set[int] = set()
    sent_track_ids: set[int] = set()

    # Counting per kelas (Kaca/Kertas/Logam/Plastik).
    # Di-counting saat bin success (EVENT:SORT_DONE tanpa FAIL).
    # Satu track_id maksimal counting 1 kali.
    class_counts: dict[str, int] = waste_counter.init_class_counts(config)
    class_count_order = list(getattr(config, "WASTE_CLASS_NAMES", []))
    counted_track_ids: set[int] = set()
    active_target_id: int | None = None
    # Snapshot kelas target disimpan saat command SORT dikirim. Jangan hanya
    # mengambil class_name dari object_tracks saat EVENT:SORT_DONE, karena
    # track biasanya sudah hilang/purge selama robot bergerak ke bin.
    active_target_class: str | None = None
    robot_busy = False
    robot_busy_since: float | None = None
    robot_busy_timeout = float(getattr(config, "ROBOT_BUSY_TIMEOUT_SEC", 35.0))
    target_lost_skip_frames = int(getattr(config, "TRACK_TARGET_LOST_SKIP_FRAMES", 3))
    active_target_occlusion_logged = False

    # --- Homing gate ---
    # Deteksi/pemilihan target baru hanya boleh berjalan setelah robot
    # selesai homing. Sebelum homing selesai, robot_homed=False membuat
    # sistem berperilaku seperti robot_busy (deteksi dimatikan total).
    robot_homed = False
    robot_homing_started_at: float | None = None
    robot_homing_wait_timeout = float(getattr(config, "ROBOT_HOMING_WAIT_TIMEOUT_SEC", 30.0))
    robot_homed_markers = [m.upper() for m in getattr(config, "ROBOT_HOMED_MARKERS", ["EVENT:HOMED"])]

    if bool(getattr(config, "ROBOT_HOMING_AUTO_ON_START", True)):
        print("[ROBOT] Mengirim G28 (homing) otomatis di awal program. Menunggu konfirmasi homing selesai...")
        esp32.send_line("G28", force=True)
        robot_homing_started_at = time.perf_counter()
    else:
        # Homing tidak diminta otomatis; anggap sudah "homed" supaya deteksi
        # langsung berjalan seperti versi lama (mis. homing dipicu manual
        # lewat tombol H sebelum sistem dipakai).
        robot_homed = True

    # --- Arrival / ID-ack gate (fase antara "sent" dan "busy penuh") ---
    # target_in_transit True sejak SORT dikirim sampai firmware mengonfirmasi
    # lengan sudah sampai target + menerima ID + siap merespons. Selama fase
    # ini deteksi TETAP berjalan normal (bukan robot_busy), hanya saja tidak
    # ada target baru lain yang dipilih/dikirim.
    target_in_transit = False
    target_sent_at: float | None = None
    robot_arrived_markers = [m.upper() for m in getattr(config, "ROBOT_ARRIVED_MARKERS", ["EVENT:ARRIVED"])]
    robot_arrival_fallback_sec = float(getattr(config, "ROBOT_ARRIVAL_FALLBACK_SEC", 8.0))
    robot_cycle_done_markers = [
        m.upper() for m in getattr(config, "ROBOT_CYCLE_DONE_MARKERS", ["EVENT:READY_FOR_NEXT_YOLO"])
    ]

    # --- Post-cycle settle (lengan masih perlu waktu keluar dari ROI) ---
    # Sesaat setelah satu siklus SORT selesai (READY_FOR_NEXT_YOLO), lengan
    # kemungkinan masih terlihat kamera selagi naik/menjauh. Deteksi ditahan
    # mati selama ROBOT_POST_CYCLE_SETTLE_SEC supaya lengan tidak ke-deteksi
    # YOLO sebagai objek baru (false positive) sehingga robot mengirim SORT
    # ke posisi lengannya sendiri.
    robot_settling = False
    robot_settle_until: float | None = None
    robot_post_cycle_settle_sec = float(getattr(config, "ROBOT_POST_CYCLE_SETTLE_SEC", 1.5))

    # =========================
    # CONVEYOR STATE
    # =========================
    # ROI hanya dipakai untuk crop/deteksi YOLO.
    # Conveyor STOP dipicu oleh STOP LINE di sisi PICK ZONE.
    # Setelah conveyor berhenti, robot boleh pick dari seluruh polygon PICK ZONE.
    conveyor_stopped_for_pick = False
    conveyor_settling = False
    conveyor_settle_until: float | None = None
    conveyor_stop_settle_sec = float(getattr(config, "CONVEYOR_STOP_SETTLE_SEC", 0.55))
    conveyor_stopped_at: float | None = None
    conveyor_pick_wait_timeout_sec = float(getattr(config, "CONVEYOR_PICK_WAIT_TIMEOUT_SEC", 3.0))
    conveyor_started_after_homing = False
    conveyor_waiting_for_roi_clear = False
    pick_zone_clear_streak = 0
    pick_zone_clear_stable_frames = int(getattr(config, "PICK_ZONE_CLEAR_STABLE_FRAMES", 3))
    manual_conveyor_stop = False

    # Jika auto-homing dimatikan, robot_homed=True berarti pengguna menganggap
    # robot sudah homing sebelum program dipakai. Dalam kondisi itu conveyor
    # boleh langsung jalan karena gate homing sudah dianggap terpenuhi.
    if robot_homed:
        try:
            if conveyor is not None:
                print("[CONVEYOR] Robot sudah dianggap homed. Conveyor START.")
                conveyor.start()
                conveyor_started_after_homing = True
        except Exception as e:
            print(f"[CONVEYOR] Gagal START setelah status homed awal: {e}")

    # =========================
    # LOOP UTAMA
    # =========================

    while True:
        if process_terminal_commands(esp32, conveyor):
            break

        frame = cam.read()
        if frame is None:
            print("ERROR: Frame kamera tidak terbaca.")
            break

        now_loop = time.perf_counter()

        # Baca respons ESP32 secara periodik, bukan setiap frame dengan blocking panjang.
        # CATATAN PENTING (firmware Final_V1): satu command SORT diproses TOTAL
        # SYNCHRONOUS oleh ESP32 (blocking sampai selesai) -- firmware TIDAK
        # PERNAH memproses command lain di tengah satu siklus SORT. Jadi robot
        # tidak mungkin "terinterupsi" oleh command Python lain di tengah jalan;
        # kalau pump menyala lalu tiba-tiba mati dan robot lanjut ke objek lain,
        # itu ARTINYA firmware sendiri yang membatalkan pick karena
        # EVENT:PICK_FAIL_NO_INA_CONTINUE_TO_BIN / PICK_PLACE_RESULT status=FAIL
        # (arus INA219 tidak pernah mencapai threshold, tetapi robot tetap lanjut ke bin).
        # Penyebab paling umum: YOLO sempat mendeteksi lengan/gripper sendiri
        # (atau bayangannya) sebagai "objek" saat lengan masih terlihat kamera,
        # lalu koordinat itu dikirim sebagai target -- makanya di area yang
        # dituju memang tidak ada sampah. Guard ROBOT_POST_CYCLE_SETTLE_SEC di
        # bawah + gate robot_busy/homing dipakai untuk menutup celah ini.
        #
        # Firmware SELALU mencetak "EVENT:READY_FOR_NEXT_YOLO" di akhir siklus SORT.
        # Marker inilah (ROBOT_CYCLE_DONE_MARKERS) yang jadi acuan PASTI untuk
        # melepas lock. Data hasil pick-place dicatat otomatis dari baris
        # DATA:PICK_PLACE_CSV oleh ESP32Comm ke logs/hasil_pick_place.csv.
        if now_loop - last_serial_poll >= serial_poll_interval:
            try:
                serial_lines = esp32.read_available(duration=serial_read_duration, echo=True)
                for serial_line in serial_lines:
                    upper_line = serial_line.upper()

                    # --- Homing selesai: baru sekarang deteksi/pemilihan target diizinkan.
                    if not robot_homed and any(marker in upper_line for marker in robot_homed_markers):
                        print("[ROBOT] Homing selesai dikonfirmasi firmware. Deteksi diaktifkan.")
                        robot_homed = True
                        if not conveyor_started_after_homing:
                            try:
                                if conveyor is not None:
                                    print("[CONVEYOR] Homing selesai. Conveyor START.")
                                    conveyor.start()
                                    conveyor_started_after_homing = True
                                    conveyor_stopped_for_pick = False
                                    conveyor_settling = False
                                    conveyor_settle_until = None
                                    conveyor_stopped_at = None
                            except Exception as e:
                                print(f"[CONVEYOR] Gagal START setelah homing selesai: {e}")

                    # --- Lengan sampai target + pump aktif di titik pick.
                    # robot_busy hanya mengunci pemilihan target baru; deteksi tetap jalan.
                    if target_in_transit and any(marker in upper_line for marker in robot_arrived_markers):
                        print(f"[ROBOT] Lengan sampai target pick untuk ID{active_target_id}.")
                        target_in_transit = False
                        target_sent_at = None
                        if bool(getattr(config, "ROBOT_BUSY_LOCK_ENABLE", True)):
                            robot_busy = True
                            robot_busy_since = now_loop

                    # --- Log status siklus (tidak melepas lock di sini, hanya info +
                    # keputusan boleh/tidaknya ID ini dicoba lagi nanti).
                    if "EVENT:SORT_DONE" in upper_line:
                        if active_target_id is not None:
                            if "PICK_RESULT=FAIL" in upper_line or "REASON=INA_NOT" in upper_line:
                                print(
                                    f"[ROBOT] SORT selesai ke bin, tetapi pick ID{active_target_id} GAGAL "
                                    "berdasarkan indikator INA. Data tersimpan di CSV."
                                )
                                sent_track_ids.discard(active_target_id)
                            else:
                                print(f"[ROBOT] SORT selesai (place ke bin) untuk ID{active_target_id}.")

                                # Gunakan snapshot kelas saat SORT dikirim. Pada saat
                                # EVENT:SORT_DONE diterima, object_tracks sering sudah
                                # menghapus ID karena objek lama tidak terlihat. Versi
                                # sebelumnya lalu menghitung ke key '?' yang tidak pernah
                                # digambar di overlay, sehingga counter stream tampak diam.
                                done_class_name = active_target_class
                                if not done_class_name:
                                    tr_done = object_tracks.get(active_target_id)
                                    if tr_done is not None:
                                        done_class_name = str(tr_done.get("class_name", ""))

                                if waste_counter.register_bin_success(
                                    active_target_id,
                                    str(done_class_name or ""),
                                    class_counts,
                                    counted_track_ids,
                                    config,
                                ):
                                    waste_counter.write_class_count_csv(
                                        class_counts, class_count_order, config
                                    )
                    elif "EVENT:PICK_FAIL_NO_INA_CONTINUE_TO_BIN" in upper_line or "EVENT:PICK_FAIL_INA_NOT_READY_CONTINUE_TO_BIN" in upper_line:
                        if active_target_id is not None:
                            print(
                                f"[ROBOT] ID{active_target_id} GAGAL indikator INA, tetapi firmware tetap lanjut ke bin "
                                "dan akan mengirim DATA:PICK_PLACE_CSV untuk log pengujian."
                            )
                            # Supaya objek yang tetap tertinggal di pick zone bisa dicoba lagi pada siklus berikutnya.
                            sent_track_ids.discard(active_target_id)
                    elif "EVENT:PICK_PLACE_RESULT" in upper_line:
                        if active_target_id is not None:
                            print(f"[ROBOT] Ringkasan pick-place ID{active_target_id}: {serial_line}")
                    elif "EVENT:SORT_ABORT_NO_OBJECT" in upper_line:
                        if active_target_id is not None:
                            print(
                                f"[ROBOT] ID{active_target_id} GAGAL terhisap (firmware lama abort karena INA threshold tidak tercapai)."
                            )
                            sent_track_ids.discard(active_target_id)
                    elif "EVENT:SORT_REJECTED" in upper_line or "EVENT:SORT_ERROR" in upper_line:
                        if active_target_id is not None:
                            print(f"[ROBOT] SORT bermasalah untuk ID{active_target_id}: {serial_line}")
                            if "REJECTED_NOT_HOMED" in upper_line:
                                sent_track_ids.discard(active_target_id)

                    # --- Sinyal PASTI akhir siklus: lepas semua lock + mulai jeda settle
                    # supaya lengan sempat benar-benar keluar dari ROI sebelum target baru dikirim.
                    if any(marker in upper_line for marker in robot_cycle_done_markers):
                        robot_busy = False
                        active_target_id = None
                        active_target_class = None
                        active_target_occlusion_logged = False
                        robot_busy_since = None
                        target_in_transit = False
                        target_sent_at = None
                        # Jangan langsung START conveyor setelah robot selesai.
                        # Sistem harus melihat PICK ZONE clear lebih dulu. Jika masih ada ID
                        # lain di PICK ZONE, robot akan langsung lanjut pick ID berikutnya.
                        conveyor_stopped_for_pick = True
                        conveyor_waiting_for_roi_clear = True
                        conveyor_settling = False
                        conveyor_settle_until = None
                        conveyor_stopped_at = now_loop
                        pick_zone_clear_streak = 0
                        print("[CONVEYOR] Robot selesai. Conveyor tetap STOP sampai PICK ZONE benar-benar kosong.")
                        if robot_post_cycle_settle_sec > 0:
                            robot_settling = True
                            robot_settle_until = now_loop + robot_post_cycle_settle_sec
            except Exception:
                pass
            try:
                if conveyor is not None:
                    conveyor.read_available(duration=0.001, echo=True)
            except Exception:
                pass
            last_serial_poll = now_loop

        # Selesai jeda settle pasca-siklus -> deteksi boleh aktif lagi.
        if robot_settling and robot_settle_until is not None and now_loop >= robot_settle_until:
            robot_settling = False
            robot_settle_until = None

        # Selesai jeda setelah conveyor STOP -> objek diasumsikan sudah diam,
        # sehingga koordinat centroid aman dikirim ke robot.
        if conveyor_settling and conveyor_settle_until is not None and now_loop >= conveyor_settle_until:
            conveyor_settling = False
            conveyor_settle_until = None
            print("[CONVEYOR] Settle selesai. Target stabil di ROI akan dikirim ke robot.")

        # Safety: kalau homing tidak pernah dikonfirmasi firmware (marker tidak cocok/
        # firmware tidak mengirim event tsb), jangan macet selamanya -- setelah timeout
        # tetap izinkan deteksi berjalan.
        if (
            not robot_homed
            and robot_homing_started_at is not None
            and (now_loop - robot_homing_started_at) >= robot_homing_wait_timeout
        ):
            print(
                f"[ROBOT] Tidak ada konfirmasi homing selesai dalam {robot_homing_wait_timeout:.1f}s. "
                "Deteksi tetap diaktifkan sebagai fallback (cek ROBOT_HOMED_MARKERS di config.py)."
            )
            robot_homed = True
            if not conveyor_started_after_homing:
                try:
                    if conveyor is not None:
                        print("[CONVEYOR] Fallback homing aktif. Conveyor START.")
                        conveyor.start()
                        conveyor_started_after_homing = True
                        conveyor_stopped_for_pick = False
                        conveyor_settling = False
                        conveyor_settle_until = None
                        conveyor_stopped_at = None
                except Exception as e:
                    print(f"[CONVEYOR] Gagal START pada fallback homing: {e}")

        # Safety: kalau firmware tidak pernah mengirim marker "arrived", jangan biarkan
        # target baru terus aktif selamanya saat lengan kemungkinan sudah di dalam ROI.
        if (
            target_in_transit
            and target_sent_at is not None
            and (now_loop - target_sent_at) >= robot_arrival_fallback_sec
        ):
            print(
                f"[ROBOT] Tidak ada konfirmasi ARRIVED dalam {robot_arrival_fallback_sec:.1f}s. "
                "Menganggap lengan sudah di area kerja (fallback); target baru dikunci sementara."
            )
            target_in_transit = False
            target_sent_at = None
            if bool(getattr(config, "ROBOT_BUSY_LOCK_ENABLE", True)):
                robot_busy = True
                robot_busy_since = now_loop

        if robot_busy and robot_busy_since is not None and (now_loop - robot_busy_since) >= robot_busy_timeout:
            print(f"[ROBOT] Busy timeout {robot_busy_timeout:.1f}s. Lock target dilepas agar sistem tidak macet.")
            robot_busy = False
            active_target_id = None
            active_target_class = None
            robot_busy_since = None
            target_in_transit = False
            target_sent_at = None

        frame_original = undistort_frame(frame)

        # Crop ROI area kerja (rectangle sederhana)
        frame_roi, roi_offset_x, roi_offset_y = roi.crop(frame_original)
        roi_h, roi_w = frame_roi.shape[:2]

        if use_letterbox_roi:
            frame_yolo_input, letterbox_meta = roi.letterbox_for_yolo(
                frame_roi,
                config.YOLO_WIDTH,
                config.YOLO_HEIGHT,
                color=getattr(config, "YOLO_LETTERBOX_COLOR", 114),
            )
        else:
            cv2.resize(frame_roi, (config.YOLO_WIDTH, config.YOLO_HEIGHT), dst=frame_yolo_buf)
            frame_yolo_input = frame_yolo_buf
            letterbox_meta = roi.no_letterbox_meta(roi_w, roi_h, config.YOLO_WIDTH, config.YOLO_HEIGHT)

        yolo_infer_start_perf = time.perf_counter()
        try:
            if bytetrack_enabled:
                results = model.track(
                    source=frame_yolo_input,
                    conf=bytetrack_predict_conf(config),
                    iou=config.IOU_THRESHOLD,
                    device=DEVICE,
                    verbose=False,
                    half=True,
                    persist=True,
                    tracker=bytetrack_tracker,
                )
            else:
                results = model.predict(
                    source=frame_yolo_input,
                    conf=config.CONF_THRESHOLD,
                    iou=config.IOU_THRESHOLD,
                    device=DEVICE,
                    verbose=False,
                    half=True,
                )
        except Exception as e:
            if bytetrack_enabled:
                print("\nWARNING: ByteTrack gagal aktif. Fallback ke tracker centroid manual.")
                print(e)
                bytetrack_enabled = False
                try:
                    results = model.predict(
                        source=frame_yolo_input,
                        conf=config.CONF_THRESHOLD,
                        iou=config.IOU_THRESHOLD,
                        device=DEVICE,
                        verbose=False,
                        half=True,
                    )
                except Exception as e2:
                    print("\nERROR: Inferensi YOLO fallback gagal.")
                    print(e2)
                    break
            else:
                print("\nERROR: Inferensi YOLO gagal.")
                print(e)
                break

        result = results[0]
        yolo_infer_done_perf = time.perf_counter()
        yolo_latency_ms_current = (yolo_infer_done_perf - yolo_infer_start_perf) * 1000.0

        # Agar label result.plot() mengikuti names dari config jika Results.names dapat ditulis.
        # Jika versi Ultralytics menjadikan ini read-only juga, abaikan; mapping bin tetap benar
        # karena draw_centroid_esp32 memakai CLASS_NAMES di bawah.
        try:
            result.names = CLASS_NAMES
        except Exception:
            pass

        frame_output = frame_original

        # Deteksi TETAP berjalan saat robot bekerja. Yang dikunci hanya
        # pemilihan/pengiriman target baru. Tujuannya agar sistem tetap tahu
        # apakah masih ada ID/objek di area deteksi; conveyor tetap dikunci oleh
        # PICK ZONE, bukan ROI penuh.
        centroids: list[dict] = []
        if robot_homed:
            draw_segmentation_batch(
                frame_output,
                result,
                roi_w,
                roi_h,
                offset_x=roi_offset_x,
                offset_y=roi_offset_y,
                letterbox_meta=letterbox_meta,
            )

            if result.masks is not None and result.boxes is not None:
                masks_data = result.masks.data.cpu().numpy()

                centroids = draw_centroid_esp32(
                    frame_output,
                    masks_data,
                    result.boxes,
                    CLASS_NAMES,
                    roi_w,
                    roi_h,
                    offset_x=roi_offset_x,
                    offset_y=roi_offset_y,
                    letterbox_meta=letterbox_meta,
                )
        else:
            cv2.putText(
                frame_output,
                "MENUNGGU HOMING SELESAI...",
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2, cv2.LINE_AA,
            )

        roi.annotate_pick_zone_objects(centroids)

        frame_index += 1
        if bytetrack_enabled:
            bytetrack_ok = update_tracks_from_bytetrack(
                centroids,
                result.boxes if result is not None else None,
                object_tracks,
                frame_index,
                config,
            )
            if not bytetrack_ok:
                next_track_id = update_object_tracks(centroids, object_tracks, next_track_id, frame_index)
        else:
            next_track_id = update_object_tracks(centroids, object_tracks, next_track_id, frame_index)
        draw_tracking_labels(frame_output, centroids, object_tracks)

        # Actual X/Y tidak dihitung dari tracking ID. Nilai aktual diukur sendiri
        # dan diinput manual pada kolom E/F file hasil pengambilan data.

        roi.draw_pick_zone_object_status(frame_output, centroids)
        if bool(getattr(config, "SHOW_ROI_BORDER_ON_FRAME", True)):
            roi.draw_roi_border(
                frame_output,
                thickness=int(getattr(config, "ROI_BORDER_THICKNESS", 1)),
                label=False,
            )
        roi.draw_pick_zone(
            frame_output,
            label=bool(getattr(config, "SHOW_PICK_ZONE_LABEL_ON_FRAME", False)),
            draw_boundary=bool(getattr(config, "SHOW_PICK_ZONE_BORDER_ON_FRAME", False)),
            draw_stop_line=bool(getattr(config, "SHOW_PICK_ZONE_STOP_LINE_ON_FRAME", True)),
            stop_line_label=bool(getattr(config, "SHOW_PICK_ZONE_STOP_LINE_LABEL", False)),
        )

        # ID di seluruh PICK ZONE dipakai untuk lock/clear conveyor setelah STOP.
        pick_zone_blocking_ids = roi.blocking_pick_zone_track_ids(
            object_tracks,
            skipped_track_ids,
            sent_track_ids,
            frame_index,
        )
        pick_zone_blocked = len(pick_zone_blocking_ids) > 0

        # ID yang menyentuh STOP LINE dipakai hanya untuk trigger STOP awal.
        pick_stop_line_ids = roi.pick_stop_line_trigger_track_ids(
            object_tracks,
            skipped_track_ids,
            sent_track_ids,
            frame_index,
        )
        pick_stop_line_triggered = len(pick_stop_line_ids) > 0

        if pick_zone_blocked:
            pick_zone_clear_streak = 0
        else:
            pick_zone_clear_streak += 1

        # Bersihkan ID skip/sent yang track-nya sudah benar-benar hilang lama.
        for old_id in list(skipped_track_ids):
            if old_id not in object_tracks:
                skipped_track_ids.discard(old_id)
        for old_id in list(sent_track_ids):
            if old_id not in object_tracks:
                sent_track_ids.discard(old_id)
        waste_counter.cleanup_counted_ids(object_tracks, counted_track_ids)

        # Jika ID yang sudah pernah dikirim ternyata masih visible setelah robot
        # selesai dan settle, anggap objek belum terambil/masih ada di ROI. Lepas
        # status sent agar bisa dipilih ulang, bukan membuat conveyor terkunci.
        if not robot_busy and not target_in_transit and not robot_settling:
            sent_visible_retry_frames = int(getattr(config, "SENT_VISIBLE_RETRY_FRAMES", 10))
            for sent_id in list(sent_track_ids):
                tr_sent = object_tracks.get(sent_id)
                if (
                    tr_sent is not None
                    and bool(tr_sent.get("visible", False))
                    and int(tr_sent.get("visible_streak", 0)) >= sent_visible_retry_frames
                ):
                    sent_track_ids.discard(sent_id)
                    print(
                        f"[TRACK] ID{sent_id} masih visible setelah robot selesai; "
                        "status sent dilepas agar bisa dicoba lagi."
                    )

        # Jika ID target hilang saat robot bekerja, jangan langsung skip.
        # Itu biasanya occlusion oleh gripper/arm atau objek memang sedang terangkat.
        # Skip hanya dipakai untuk kasus invalid seperti di luar workspace.
        if active_target_id is not None:
            active_track = object_tracks.get(active_target_id)
            if (
                active_track is not None
                and int(active_track.get("miss_count", 0)) >= target_lost_skip_frames
                and (robot_busy or target_in_transit)
                and not active_target_occlusion_logged
            ):
                active_target_occlusion_logged = True
                print(
                    f"[TRACK] ID{active_target_id} sementara hilang/tertutup saat robot bekerja; "
                    "ID ditahan, bukan langsung di-skip."
                )

        target_obj = None

        # Conveyor STOP saat bbox/mask objek menyentuh STOP LINE PICK ZONE.
        # Robot belum langsung pick; tunggu conveyor settle, lalu pilih target stabil
        # dari seluruh area PICK ZONE.
        if (
            pick_stop_line_triggered
            and robot_homed
            and not manual_conveyor_stop
            and not conveyor_stopped_for_pick
            and not robot_busy
            and not target_in_transit
            and not robot_settling
        ):
            ids_text = ",".join(str(i) for i in pick_stop_line_ids[:6])
            print(f"[CONVEYOR] ID menyentuh STOP LINE PICK ZONE ({ids_text}). Conveyor STOP; robot pick setelah objek settle.")
            try:
                if conveyor is not None:
                    conveyor.stop()
            except Exception as e:
                print(f"[CONVEYOR] Gagal STOP: {e}")
            conveyor_stopped_for_pick = True
            conveyor_waiting_for_roi_clear = False
            conveyor_settling = True
            conveyor_settle_until = now_loop + conveyor_stop_settle_sec
            conveyor_stopped_at = now_loop

        if (
            centroids
            and robot_homed
            and conveyor_stopped_for_pick
            and not robot_busy
            and not target_in_transit
            and not robot_settling
            and not conveyor_settling
        ):
            target_obj = choose_target_object(
                centroids,
                object_tracks,
                skipped_track_ids,
                sent_track_ids,
            )

        # Conveyor hanya boleh START jika seluruh PICK ZONE clear.
        # Objek di ROI luar PICK ZONE tidak menahan conveyor.
        if (
            conveyor_stopped_for_pick
            and not manual_conveyor_stop
            and not conveyor_settling
            and not robot_busy
            and not target_in_transit
            and not robot_settling
            and active_target_id is None
            and target_obj is None
            and not pick_zone_blocked
            and pick_zone_clear_streak >= pick_zone_clear_stable_frames
        ):
            try:
                if conveyor is not None and robot_homed:
                    if conveyor_waiting_for_roi_clear:
                        print("[CONVEYOR] PICK ZONE clear. Conveyor START lagi.")
                    conveyor.start()
            except Exception as e:
                print(f"[CONVEYOR] Gagal START saat PICK ZONE clear: {e}")
            conveyor_stopped_for_pick = False
            conveyor_waiting_for_roi_clear = False
            conveyor_stopped_at = None
            pick_zone_clear_streak = 0

        # Safety: jika conveyor STOP tetapi tidak ada ID blocking di PICK ZONE selama
        # beberapa frame, conveyor boleh jalan lagi. ROI luar tidak lagi menjadi
        # syarat stop/start conveyor.
        if (
            conveyor_stopped_for_pick
            and not manual_conveyor_stop
            and not conveyor_settling
            and not robot_busy
            and not target_in_transit
            and not robot_settling
            and active_target_id is None
            and not pick_zone_blocked
            and pick_zone_clear_streak >= pick_zone_clear_stable_frames
            and conveyor_stopped_at is not None
            and (now_loop - conveyor_stopped_at) >= conveyor_pick_wait_timeout_sec
        ):
            print("[CONVEYOR] PICK ZONE kosong/stabil. Conveyor START ulang.")
            try:
                if conveyor is not None:
                    conveyor.start()
            except Exception as e:
                print(f"[CONVEYOR] Gagal START ulang: {e}")
            conveyor_stopped_for_pick = False
            conveyor_waiting_for_roi_clear = False
            conveyor_stopped_at = None
            pick_zone_clear_streak = 0

        if pick_zone_blocked and bool(getattr(config, "SHOW_PICK_ZONE_OCCUPANCY_ON_FRAME", True)):
            ids_text = ",".join(str(i) for i in pick_zone_blocking_ids[:8])
            more = "..." if len(pick_zone_blocking_ids) > 8 else ""
            cv2.putText(
                frame_output,
                f"PICK ZONE OCCUPIED ID: {ids_text}{more} | CONVEYOR LOCK",
                (10, 200),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )

        if pick_stop_line_triggered and bool(getattr(config, "SHOW_PICK_ZONE_OCCUPANCY_ON_FRAME", True)):
            ids_text = ",".join(str(i) for i in pick_stop_line_ids[:8])
            more = "..." if len(pick_stop_line_ids) > 8 else ""
            cv2.putText(
                frame_output,
                f"STOP LINE TOUCH ID: {ids_text}{more}",
                (10, 224),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        if robot_busy or target_in_transit or conveyor_settling:
            if conveyor_settling:
                busy_label = "CONVEYOR STOP SETTLING"
            else:
                busy_label = "ROBOT BUSY" if robot_busy else "ROBOT IN TRANSIT (menunggu konfirmasi)"
            if active_target_id is not None:
                busy_label += f" | active ID{active_target_id}"
            cv2.putText(
                frame_output,
                busy_label,
                (10, 175),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        if target_obj is not None:
            track_id = int(target_obj.get("track_id", 0) or 0)
            cx_full = target_obj["cx"]
            cy_full = target_obj["cy"]
            cls_id = int(target_obj.get("cls_id", -1))
            class_name = str(target_obj.get("class_name", cls_id))

            class_to_bin = getattr(config, "WASTE_CLASS_ID_TO_BIN_INDEX", {})
            bin_index = int(class_to_bin.get(cls_id, getattr(config, "ROBOT_BIN_INDEX", 0)))
            bin_names = list(getattr(config, "WASTE_CLASS_NAMES", []))
            bin_name = bin_names[bin_index] if 0 <= bin_index < len(bin_names) else f"B{bin_index}"

            # Homography langsung menghasilkan koordinat ROBOT BASE.
            x_robot_mm, y_robot_mm = homography.pixel_to_robot(cx_full, cy_full)

            if bool(getattr(config, "SHOW_TARGET_ROBOT_COORD_ON_FRAME", False)):
                cv2.putText(
                    frame_output,
                    f"TARGET ID{track_id} ROBOT X{x_robot_mm:.1f} Y{y_robot_mm:.1f}",
                    (int(cx_full) + 10, int(cy_full) + 38),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    float(getattr(config, "OVERLAY_TEXT_SCALE", 0.45)),
                    (0, 255, 255),
                    int(getattr(config, "OVERLAY_TEXT_THICKNESS", 1)),
                    cv2.LINE_AA,
                )

            if bool(getattr(config, "SHOW_TARGET_BIN_ON_FRAME", True)):
                cv2.putText(
                    frame_output,
                    f"ID{track_id} | {class_name} -> BIN B{bin_index} {bin_name}",
                    (int(cx_full) + 10, int(cy_full) + 62),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    float(getattr(config, "OVERLAY_TEXT_SCALE", 0.45)),
                    (0, 255, 255),
                    int(getattr(config, "OVERLAY_TEXT_THICKNESS", 1)),
                    cv2.LINE_AA,
                )

            if homography.is_inside_workspace(x_robot_mm, y_robot_mm):
                now_send = time.perf_counter()
                dist_from_last = math.inf
                if last_sent_xy is not None:
                    dist_from_last = math.hypot(x_robot_mm - last_sent_xy[0], y_robot_mm - last_sent_xy[1])

                enough_interval = (now_send - last_yolo_send_time) >= yolo_send_interval
                moved_enough = dist_from_last >= yolo_resend_distance
                forced_resend = (now_send - last_yolo_send_time) >= yolo_force_resend_sec

                if enough_interval and (last_sent_xy is None or moved_enough or forced_resend):
                    mode = getattr(config, "ESP32_COMMAND_MODE", "SORT").upper()
                    if mode == "G1":
                        sent = esp32.move_g1(
                            x_mm=x_robot_mm,
                            y_mm=y_robot_mm,
                            z_mm=getattr(config, "ROBOT_PICK_Z_MM", 40.0),
                        )
                    else:
                        decision_latency_ms_current = max(0.0, (now_send - yolo_infer_done_perf) * 1000.0)
                        # Actual X/Y tidak diambil dari tracking ID. Nilai actual (opsional)
                        # diukur dan diinput manual pada kolom F/G file pengambilan data.
                        # Kelas, latency, dan status Success/Fail (indikator INA/touch)
                        # otomatis dicatat oleh ESP32Comm ke hasil_pick_place.csv/xlsx.
                        sent = esp32.send_sort(
                            x_robot_mm,
                            y_robot_mm,
                            z_mm=getattr(config, "ROBOT_PICK_Z_MM", 40.0),
                            bin_index=bin_index,
                            object_id=track_id,
                            yolo_latency_ms=yolo_latency_ms_current,
                            decision_latency_ms=decision_latency_ms_current,
                            class_name=class_name,
                        )

                    if sent:
                        last_yolo_send_time = now_send
                        last_sent_xy = (x_robot_mm, y_robot_mm)
                        sent_track_ids.add(track_id)
                        active_target_id = track_id
                        active_target_class = class_name
                        active_target_occlusion_logged = False
                        conveyor_waiting_for_roi_clear = False
                        # Belum langsung "busy" (deteksi tidak langsung dimatikan).
                        # Menunggu firmware konfirmasi lengan sampai target + ID
                        # diterima + lengan siap merespons (ROBOT_ARRIVED_MARKERS).
                        target_in_transit = True
                        target_sent_at = now_send
                        conveyor_stopped_at = None
                        print(
                            f"[TARGET ROBOT SENT] ID{track_id} cls={class_name} -> B{bin_index} {bin_name} "
                            f"px=({cx_full},{cy_full}) "
                            f"robot=(X{x_robot_mm:.1f},Y{y_robot_mm:.1f})mm"
                        )
            else:
                # Target di luar workspace: skip ID ini agar ID lain yang valid bisa langsung dipilih.
                if track_id > 0 and track_id not in skipped_track_ids:
                    skipped_track_ids.add(track_id)
                    print(
                        f"[TRACK] ID{track_id} di-skip karena di luar workspace robot: "
                        f"X{x_robot_mm:.1f} Y{y_robot_mm:.1f}"
                    )

        current_time = time.perf_counter()
        elapsed = current_time - prev_time
        if elapsed > 0:
            fps_deque.append(1.0 / elapsed)
        avg_fps = sum(fps_deque) / len(fps_deque) if fps_deque else 0
        prev_time = current_time

        # Overlay ringkas: FPS + ID/Kelas target. Posisi diatur dari config.py
        # (default revisi: kiri bawah, bukan kiri atas).
        mouse_x = mouse_state["x"]
        mouse_y = mouse_state["y"]
        try:
            mouse_robot = homography.pixel_to_robot(mouse_x, mouse_y)
        except Exception:
            mouse_robot = None

        overlay_pick_id = None
        overlay_pick_class = None
        if target_obj is not None:
            overlay_pick_id = int(target_obj.get("track_id", 0) or 0)
            overlay_pick_class = str(target_obj.get("class_name", target_obj.get("cls_id", "--")))
        elif active_target_id is not None:
            overlay_pick_id = int(active_target_id)
            active_track = object_tracks.get(int(active_target_id), {})
            overlay_pick_class = str(active_track.get("class_name", active_track.get("cls_id", "--")))

        draw_overlay_info(
            frame_output,
            avg_fps,
            mouse_x,
            mouse_y,
            esp32_status=f"{esp32.status} | CONV {conveyor.status if conveyor is not None else 'OFF'}",
            mouse_robot=mouse_robot,
            pick_id=overlay_pick_id,
            pick_class=overlay_pick_class,
        )

        if bool(getattr(config, "SHOW_CLASS_COUNTERS_ON_FRAME", True)):
            draw_class_counters(frame_output, class_counts, class_count_order)

        cv2.imshow(config.WINDOW_MAIN, frame_output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            print("[HOTKEY] Q -> keluar, stop conveyor, PUMP OFF, tutup serial ESP32")
            break
        elif key == ord("r"):
            print("[HOTKEY] R -> tahap 1/2 pilih ROI deteksi YOLO")
            calibration_window = "Kalibrasi ROI + PICK ZONE - satu kamera"
            new_roi_points = roi.select_points_interactive(
                lambda: undistort_frame(cam.read()),
                window_name=calibration_window,
                destroy_window=False,
            )
            if new_roi_points is not None:
                roi.set_points(new_roi_points, UNDIST_W, UNDIST_H, save=True)
                print("[HOTKEY] R -> tahap 2/2 pilih PICK ZONE. STOP aktif di garis kuning; garis ROI hijau tetap terlihat")
                new_pick_points = roi.select_pick_zone_points_interactive(
                    lambda: undistort_frame(cam.read()),
                    window_name=calibration_window,
                    destroy_window=True,
                )
                if new_pick_points is not None:
                    roi.set_pick_zone_points(new_pick_points, UNDIST_W, UNDIST_H, save=True)
                    pick_zone_clear_streak = 0
                else:
                    print("[PICK_ZONE] Tahap 2 dibatalkan. PICK ZONE lama/fallback tetap dipakai.")
        elif key in (ord("t"), ord("T")):
            print("[HOTKEY] T -> pilih ulang PICK ZONE saja. STOP aktif di garis kuning; garis ROI hijau tetap terlihat")
            new_pick_points = roi.select_pick_zone_points_interactive(
                lambda: undistort_frame(cam.read()),
                window_name="Kalibrasi PICK ZONE - ROI tetap terlihat",
            )
            if new_pick_points is not None:
                roi.set_pick_zone_points(new_pick_points, UNDIST_W, UNDIST_H, save=True)
                pick_zone_clear_streak = 0
        elif key in (ord("h"), ord("H")):
            print("[HOTKEY] H -> G28 (deteksi dinonaktifkan sampai homing dikonfirmasi ulang)")
            try:
                if conveyor is not None:
                    print("[CONVEYOR] Re-homing dimulai. Conveyor STOP sampai homing selesai.")
                    conveyor.stop()
            except Exception as e:
                print(f"[CONVEYOR] Gagal STOP saat re-homing: {e}")
            esp32.send_line("G28", force=True)
            robot_homed = False
            conveyor_started_after_homing = False
            manual_conveyor_stop = False
            conveyor_waiting_for_roi_clear = False
            pick_zone_clear_streak = 0
            conveyor_stopped_for_pick = False
            conveyor_settling = False
            conveyor_settle_until = None
            conveyor_stopped_at = None
            robot_homing_started_at = time.perf_counter()
        elif key in (ord("?"), ord("/")):
            print("[HOTKEY] ? -> HELP")
            esp32.send_line("HELP", force=True)
        elif key in (ord("m"), ord("M")):
            print("[HOTKEY] M -> M114")
            esp32.send_line("M114", force=True)
        elif key in (ord("l"), ord("L")):
            print("[HOTKEY] L -> M119")
            esp32.send_line("M119", force=True)
        elif key in (ord("i"), ord("I")):
            print("[HOTKEY] I -> INA")
            esp32.send_line("INA", force=True)
        elif key in (ord("y"), ord("Y")):
            print("[HOTKEY] Y -> YOLO")
            esp32.send_line("YOLO", force=True)
        elif key in (ord("c"), ord("C")):
            print("[HOTKEY] C -> conveyor START")
            if conveyor is not None:
                conveyor.start()
            manual_conveyor_stop = False
            conveyor_stopped_for_pick = False
            conveyor_settling = False
            conveyor_settle_until = None
            conveyor_stopped_at = None
        elif key in (ord("s"), ord("S")):
            print("[HOTKEY] S -> conveyor STOP")
            if conveyor is not None:
                conveyor.stop()
            manual_conveyor_stop = True
            conveyor_stopped_for_pick = True
            conveyor_waiting_for_roi_clear = False
            conveyor_settling = False
            conveyor_settle_until = None
            conveyor_stopped_at = time.perf_counter()
        elif key in (ord("o"), ord("O")):
            print("[HOTKEY] O -> PUMP ON")
            esp32.send_line("PUMP ON", force=True)
        elif key in (ord("p"), ord("P")):
            print("[HOTKEY] P -> PUMP OFF")
            esp32.send_line("PUMP OFF", force=True)

except KeyboardInterrupt:
    print("\n[CTRL+C] main.py dihentikan oleh user.")
finally:
    safe_shutdown(cam, esp32, conveyor)
