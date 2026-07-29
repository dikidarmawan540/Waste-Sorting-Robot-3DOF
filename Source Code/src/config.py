import os

# =========================
# MODEL
# =========================

_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_DIR = os.path.dirname(_BASE_DIR)

MODEL_PATH = r"D:\Documents\Yolov11-seg Skripsi\model\best.engine"

# =========================
# CAMERA
# =========================

# Kamera aplikasi berasal dari OBS Virtual Camera, bukan webcam fisik langsung.
# CAM_INDEX hanya dipakai sebagai cadangan apabila pygrabber belum terpasang.
# Pada umumnya webcam fisik berada di index 0 dan OBS Virtual Camera di index 1.
CAM_DEVICE_NAME = "OBS Virtual Camera"
CAM_INDEX = 1
CAM_REQUIRE_DEVICE_NAME = False
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 30


# =========================
# CONVEYOR / PLC MODBUS
# =========================
# Pilihan mode:
#   "AUTO"            = coba koneksi ESP32 dulu, jika gagal coba PLC TCP/RTU
#   "ESP32"           = conveyor via serial ESP32 lama saja
#   "PLC_MODBUS_TCP"  = conveyor via PLC Modbus TCP saja
#   "PLC_MODBUS_RTU"  = conveyor via PLC Modbus RTU/serial saja
#
# Dengan AUTO, sistem memilih koneksi yang berhasil dari urutan CONVEYOR_AUTO_PRIORITY.
# Jika ESP32 conveyor COM10 terhubung, ESP32 dipakai. Jika ESP32 tidak ada, coba PLC.
CONVEYOR_CONTROL_MODE = "AUTO"
CONVEYOR_AUTO_PRIORITY = "ESP32,PLC_MODBUS_TCP,PLC_MODBUS_RTU"

# Setting TCP PLC. Sesuaikan dengan IP PLC kamu. Jika AUTO memilih PLC, D0 ditulis sebagai register: D0=1 RUN, D0=0 STOP.
CONVEYOR_PLC_HOST = "192.168.1.10"
CONVEYOR_PLC_PORT = 502
CONVEYOR_PLC_UNIT_ID = 1

# Setting RTU PLC jika memakai RS485/serial. Dipakai hanya jika mode PLC_MODBUS_RTU.
CONVEYOR_PLC_RTU_PORT = "COM10"
CONVEYOR_PLC_RTU_BAUDRATE = 9600
CONVEYOR_PLC_RTU_PARITY = "N"
CONVEYOR_PLC_RTU_STOPBITS = 1
CONVEYOR_PLC_RTU_BYTESIZE = 8

# Mode D0: tulis holding register. Default address 0 = D0 pada banyak PLC.
# Jika mapping PLC berbeda, ubah address ini sesuai tabel Modbus PLC.
CONVEYOR_PLC_WRITE_MODE = "REGISTER"
CONVEYOR_PLC_RUN_REGISTER_ADDRESS = 0
CONVEYOR_PLC_RUN_VALUE = 1
CONVEYOR_PLC_STOP_VALUE = 0

# Pulse opsional untuk test dari console conveyor.py. Main program tetap memakai
# level-signal D0=1/D0=0 agar conveyor tidak miss trigger.
CONVEYOR_PLC_PULSE_DURATION_SEC = 0.20

# Setting serial ESP32 lama. Dipakai jika CONVEYOR_CONTROL_MODE="ESP32" atau AUTO memilih ESP32.
# ISI MANUAL sesuai Device Manager (lihat port yang aktif saat ESP32 conveyor dicolok).
CONVEYOR_SERIAL_PORT = "COM14"
CONVEYOR_SERIAL_BAUDRATE = 115200
CONVEYOR_SERIAL_TIMEOUT = 0.1
CONVEYOR_START_COMMAND = "R"
CONVEYOR_RUN_COMMAND = "START"
CONVEYOR_STOP_COMMAND = "STOP"

# =========================
# YOLO
# =========================

YOLO_WIDTH = 640
YOLO_HEIGHT = 640
CONF_THRESHOLD = 0.6
IOU_THRESHOLD = 0.5

# =========================
# DISPLAY
# =========================

WINDOW_MAIN = "YOLO Detection -> Robot Coordinate"

MASK_COLORS = [
    (255, 85, 0),
    (0, 200, 255),
    (170, 0, 255),
    (0, 255, 100),
    (255, 200, 0),
    (255, 0, 170),
    (50, 180, 255),
    (0, 255, 200),
]
MASK_ALPHA = 0.25

# Tampilan overlay live-view dibuat ringkas:
# - hanya FPS + info target pick-and-place
# - teks dipindah ke kiri bawah agar tidak menutup area atas conveyor/ROI
# - font tetap kecil agar tidak menutupi area kerja
# - ROI ditampilkan tipis sebagai referensi area deteksi
# - PICK ZONE tidak digambar sebagai kotak/polygon; hanya stop line kuning tipis
OVERLAY_TEXT_SCALE = 0.42
OVERLAY_TEXT_THICKNESS = 1
OVERLAY_TEXT_LINE_GAP = 20
OVERLAY_TEXT_POSITION = "BOTTOM_LEFT"  # BOTTOM_LEFT / TOP_LEFT
OVERLAY_TEXT_MARGIN_X = 10
OVERLAY_TEXT_MARGIN_BOTTOM = 12
SHOW_ROI_BORDER_ON_FRAME = True
ROI_BORDER_THICKNESS = 1
SHOW_PICK_ZONE_BORDER_ON_FRAME = False
SHOW_PICK_ZONE_LABEL_ON_FRAME = False
SHOW_PICK_ZONE_STOP_LINE_ON_FRAME = True
SHOW_PICK_ZONE_STOP_LINE_LABEL = False
PICK_ZONE_STOP_LINE_THICKNESS = 1
PICK_ZONE_STOP_LINE_MARKER_RADIUS = 3
SHOW_TARGET_ROBOT_COORD_ON_FRAME = False
SHOW_SYSTEM_STATUS_ON_FRAME = False


# =========================
# OBJECT ID / TRACKING
# =========================
# ID dipakai agar objek yang terdeteksi lebih dulu tetap punya prioritas.
# Jika masking/segmentation suatu ID hilang saat robot arm bekerja, ID tidak
# langsung di-skip; ByteTrack + occlusion hold menjaga conveyor tetap STOP.
TRACKING_ENABLE = True
TRACK_MAX_MATCH_DISTANCE_PX = 80.0
TRACK_MAX_MISSED_FRAMES = 8
TRACK_PURGE_MISSED_FRAMES = 90
TRACK_STABLE_MIN_FRAMES = 2
TRACK_TARGET_LOST_SKIP_FRAMES = 3
TRACK_DRAW_ID = True
ROBOT_BUSY_LOCK_ENABLE = True
ROBOT_BUSY_TIMEOUT_SEC = 35.0

# Backend tracking: "BYTETRACK" direkomendasikan untuk mengurangi flickering ID.
# Jika ByteTrack gagal pada versi Ultralytics tertentu, main.py otomatis fallback
# ke tracker centroid manual.
TRACKING_BACKEND = "BYTETRACK"
BYTETRACK_CONFIG_PATH = os.path.join(_BASE_DIR, "bytetrack_conveyor.yaml")
# ByteTrack butuh confidence input yang tidak terlalu tinggi agar track tidak
# mudah putus saat confidence turun sebentar. Filter target tetap dijaga oleh
# TRACK_STABLE_MIN_FRAMES dan class vote di tracking_utils.py.
BYTETRACK_YOLO_CONF_THRESHOLD = 0.25
TRACK_CLASS_UPDATE_MIN_VOTES = 3
TRACK_CONF_HISTORY_SIZE = 8

# Conveyor lock berbasis PICK ZONE, bukan ROI deteksi.
# ROI dan PICK ZONE sama-sama dikelola di roi.py agar penentuan titik cukup 2 tahap dalam satu file.
# Revisi: conveyor STOP dipicu oleh garis akhir/stop line di PICK ZONE,
# sedangkan robot boleh memilih target dari SELURUH area PICK ZONE.
PICK_ZONE_ENABLE = True
PICK_ZONE_POINTS_PATH = os.path.join(_PROJECT_DIR, "config", "pick_zone_points.json")
PICK_ZONE_DEFAULT_WIDTH_RATIO = 0.35
PICK_ZONE_DEFAULT_HEIGHT_RATIO = 0.35

# STOP LINE PICK ZONE
# Default LEFT = objek baru memicu STOP saat sisi/bounding box objek menyentuh
# garis paling kiri PICK ZONE. Jika arah conveyor berlawanan, ganti ke RIGHT/TOP/BOTTOM.
PICK_ZONE_STOP_LINE_ENABLE = True
PICK_ZONE_STOP_LINE_SIDE = "LEFT"       # LEFT, RIGHT, TOP, BOTTOM
PICK_ZONE_STOP_LINE_TOLERANCE_PX = 6     # toleransi sentuh garis agar tidak flicker
PICK_ZONE_STOP_LINE_HOLD_FRAMES = 3      # tahan trigger beberapa frame jika mask sempat flicker
PICK_ZONE_STOP_REQUIRE_CENTROID_IN_ZONE = True

# Kompatibilitas variabel lama. Tidak dipakai lagi sebagai trigger utama.
PICK_ZONE_CENTER_GATE_ENABLE = False
PICK_ZONE_CENTER_GATE_WIDTH_RATIO = 0.10
PICK_ZONE_CENTER_GATE_HEIGHT_RATIO = 0.10
PICK_ZONE_CENTER_GATE_MIN_SIZE_PX = 18
PICK_ZONE_CENTER_GATE_MAX_SIZE_PX = 45

CONVEYOR_BLOCK_WHILE_PICK_ZONE_OCCUPIED = True
PICK_ZONE_OCCLUSION_HOLD_FRAMES = 12
PICK_ZONE_CLEAR_STABLE_FRAMES = 3
PICK_ZONE_FILL_OVERLAY = False
PICK_ZONE_FILL_ALPHA = 0.12
SHOW_PICK_ZONE_OCCUPANCY_ON_FRAME = False
SHOW_PICK_ZONE_OBJECT_STATUS = False

# Opsi lama tetap disediakan sebagai kompatibilitas, tetapi main.py sekarang
# memakai PICK_ZONE_* sebagai syarat stop/start conveyor.
CONVEYOR_BLOCK_WHILE_ROI_OCCUPIED = False
ROI_OCCLUSION_HOLD_FRAMES = PICK_ZONE_OCCLUSION_HOLD_FRAMES
ROI_CLEAR_STABLE_FRAMES = PICK_ZONE_CLEAR_STABLE_FRAMES
CONVEYOR_IGNORE_SKIPPED_IDS_FOR_BLOCK = False
CONVEYOR_IGNORE_SENT_IDS_WHEN_NOT_VISIBLE = True
SENT_VISIBLE_RETRY_FRAMES = 10
SHOW_ROI_OCCUPANCY_ON_FRAME = False
CONVEYOR_STOP_SETTLE_SEC = 0.20
CONVEYOR_PICK_WAIT_TIMEOUT_SEC = 0.80

# =========================
# ROBOT HOMING / READY GATE
# =========================
# Pemilihan target baru untuk dikirim ke robot HANYA aktif
# setelah robot benar-benar selesai homing. Sebelum homing selesai, sistem
# dianggap "busy" (sama seperti robot_busy) sehingga tidak ada target yang
# dipilih/dikirim.
#
# Jika True, main.py otomatis mengirim G28 sesaat setelah program mulai dan
# menunggu firmware mengonfirmasi homing selesai sebelum mengizinkan deteksi.
ROBOT_HOMING_AUTO_ON_START = True

# Berapa lama menunggu konfirmasi homing sebelum menyerah dan tetap
# mengizinkan deteksi berjalan (safety agar program tidak macet total kalau
# firmware tidak pernah mengirim marker homing-selesai).
ROBOT_HOMING_WAIT_TIMEOUT_SEC = 30.0

# Potongan teks (huruf besar) pada baris serial dari firmware yang menandakan
# homing SUDAH SELESAI. Dicek dengan "substring in line", jadi tidak perlu
# exact match. Firmware Final_V1 mencetak "G28 done. Raw offset applied..."
# (tanpa prefix EVENT:), jadi markernya "G28 DONE".
ROBOT_HOMED_MARKERS = ["G28 DONE"]

# Setelah command SORT (dengan ID) dikirim, robot_busy (yang mematikan
# deteksi supaya gripper tidak ikut ke-deteksi sebagai objek) BARU aktif
# setelah firmware mengonfirmasi robot BENAR-BENAR:
#   1) sudah sampai di posisi target, DAN
#   2) sudah menerima ID yang dikirim, DAN
#   3) lengan robot sudah merespons (siap pick/place).
# Sebelum konfirmasi ini diterima, deteksi tetap berjalan normal (robot
# dianggap masih "in transit", bukan busy) -- hanya target BARU yang tidak
# dipilih selama satu target masih diproses.
#
# Firmware Final_V1 mencetak "EVENT:PUMP_ON_AT_PICK_XY" persis saat lengan
# sudah sampai XY pick dan mulai pump -- ini dipakai sebagai marker "arrived".
ROBOT_ARRIVED_MARKERS = ["EVENT:PUMP_ON_AT_PICK_XY"]

# Safety fallback: kalau tidak ada konfirmasi ARRIVED sama sekali dalam
# durasi ini setelah command dikirim, tetap anggap busy (asumsi lengan
# sudah masuk area ROI) supaya deteksi tidak salah menganggap gripper
# sebagai objek baru.
ROBOT_ARRIVAL_FALLBACK_SEC = 8.0

# Firmware revisi mencetak "EVENT:READY_FOR_NEXT_YOLO" di akhir setiap SORT.
# Untuk pengujian, jika INA tidak mendeteksi arus, robot tetap lanjut ke bin
# dan mengirim DATA:PICK_PLACE_CSV dengan status=FAIL. Marker READY dipakai
# sebagai sinyal PASTI untuk melepas robot_busy/target lock.
ROBOT_CYCLE_DONE_MARKERS = ["EVENT:READY_FOR_NEXT_YOLO"]

# Setelah satu siklus SORT selesai (READY_FOR_NEXT_YOLO), lengan robot masih
# butuh waktu singkat untuk benar-benar keluar dari bidang pandang kamera
# (mis. naik ke Z safety / bergerak menjauh dari ROI). Kalau deteksi langsung
# diaktifkan lagi persis di titik ini, lengan yang masih terlihat di kamera
# berisiko ke-deteksi YOLO sebagai "objek" baru (false positive) dan robot
# mengirim SORT ke posisi lengannya sendiri -> indikator INA bisa FAIL karena
# memang tidak ada objek di sana, walaupun robot tetap lanjut ke bin.
# ROBOT_POST_CYCLE_SETTLE_SEC menahan deteksi tetap mati sesaat setelah
# READY_FOR_NEXT_YOLO untuk menghindari ini. Sesuaikan dengan seberapa lama
# lengan butuh waktu benar-benar menjauh dari ROI setelah selesai.
ROBOT_POST_CYCLE_SETTLE_SEC = 0.35

# =========================
# SERIAL / ESP32
# =========================

SERIAL_PORT = "COM15"
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.3
SERIAL_RECONNECT_DELAY = 2.0
SERIAL_ENABLE = True


# =========================
# LOG PICK AND PLACE (VS Code)
# =========================
# Firmware ESP32 mengirim baris DATA:PICK_PLACE_CSV hanya saat siklus SORT/pick-place.
# main.py / esp32_comm.py otomatis menangkap baris itu dan menyimpan log ke CSV mentah saja.
# Revisi data:
# - Kolom dihapus: object_id, pick_z_req, bin_index, bin_x, bin_y, bin_z, ina_ready, offset_ok.
# - Status SUCCESS/FAIL diletakkan di kolom paling kanan.
# - ina_detected dan touch_current_A tetap disimpan untuk menjelaskan penyebab FAIL.
# - Latency dipisahkan dari cycle time:
#   yolo_latency_ms     = waktu inference/komputasi YOLO.
#   decision_latency_ms = waktu dari hasil YOLO sampai Python mengirim SORT.
#   serial_latency_ms   = waktu dari Python mengirim SORT sampai ESP32 membalas ACK koordinat.
#   total_latency_ms    = yolo + decision + serial.
#   cycle_time_ms       = durasi gerak fisik robot pick-and-place dari firmware.
# - MAE ditambahkan. actual_x/actual_y otomatis diambil dari koordinat objek pada tracking ID,
#   lalu DIKUNCI LANGSUNG saat robot akan bergerak ke XY objek / SORT dikirim.
# - CSV tetap 1 file saja. Setiap 8 percobaan ditambah blok/baris judul baru:
#   PERCOBAAN PERTAMA, PERCOBAAN KEDUA, dst.
# - trial_id di setiap blok PERCOBAAN mulai dari 1 lagi.
PICK_PLACE_LOG_ENABLE = True
PICK_PLACE_LOG_DIR = os.path.join(_PROJECT_DIR, "logs")
PICK_PLACE_LOG_CSV_BASE_NAME = "PERCOBAAN"
PICK_PLACE_LOG_CSV_SINGLE_FILE = "hasil_pick_place.csv"
PICK_PLACE_LOG_ROWS_PER_FILE = 8
PICK_PLACE_LOG_WRITE_TITLE_ROW = True
PICK_PLACE_LOG_WRITE_XLSX = False

# MAE koordinat XY pick.
# actual_x/actual_y otomatis diambil dari titik objek pada tracking ID kamera
# yang sudah dikonversi ke koordinat robot. Jadi tidak perlu input manual.
# Rumus:
#   mae_x_mm  = abs(pick_x - actual_x_tracking_id)
#   mae_y_mm  = abs(pick_y - actual_y_tracking_id)
#   mae_xy_mm = sqrt(mae_x_mm^2 + mae_y_mm^2)
PICK_PLACE_MAE_ENABLE = True
PICK_PLACE_MAE_ACTUAL_FROM_TRACKING = True
# actual_x/actual_y dikunci langsung pada momen robot akan bergerak ke XY objek / SORT dikirim.
# Revisi: tidak memakai median beberapa frame. Nilai actual adalah koordinat tracking ID
# pada frame target yang dipilih untuk SORT.
PICK_PLACE_ACTUAL_FREEZE_ON_SORT = True
PICK_PLACE_ACTUAL_MEDIAN_FRAMES = 1  # kompatibilitas lama; tidak dipakai untuk median.
PICK_PLACE_TRACKING_ACTUAL_MAX_AGE_SEC = 10.0

# Fallback opsional kalau tracking ID hilang terlalu lama. Biarkan None untuk default.
PICK_PLACE_MAE_REF_X_MM = None
PICK_PLACE_MAE_REF_Y_MM = None
PICK_PLACE_MAE_REFERENCE_BY_BIN_NAME = {}

# Anti-flood serial: minimal jarak waktu antar command
SEND_INTERVAL = 0.20

# Main.py command terminal: memungkinkan mengetik HELP/G28/M114/dll
# di terminal VS Code saat kamera YOLO sedang berjalan.
TERMINAL_COMMAND_ENABLE = True

# Supaya integrasi ESP32 tidak menjatuhkan FPS terlalu parah.
# Main.py hanya polling respons serial tiap interval ini.
SERIAL_READ_INTERVAL = 0.03
SERIAL_READ_DURATION = 0.001

# Rate-limit command YOLO -> ESP32 agar relay/pump tidak on/off terus dan FPS tidak drop.
YOLO_SEND_INTERVAL = 0.35
YOLO_RESEND_DISTANCE_MM = 10.0
YOLO_FORCE_RESEND_SEC = 1.20

# Saat keluar dengan Q/EXIT/CTRL+C, kirim PUMP OFF lalu tutup COM port.
PUMP_OFF_ON_EXIT = True

# Mode command ke ESP32:
#   "SORT" = kirim SORT X.. Y.. Z.. B..
#   "G1"   = kirim G1 X.. Y.. Z..
ESP32_COMMAND_MODE = "SORT"
ROBOT_PICK_Z_MM = 40.0

# =========================
# KELAS YOLO -> BIN ROBOT
# =========================
# Urutan wajib sesuai dataset/model:
#   nc: 4
#   names: ['Kaca', 'Kertas', 'Logam', 'Plastik']
WASTE_NC = 4
WASTE_CLASS_NAMES = ["Kaca", "Kertas", "Logam", "Plastik"]

# Mapping class-id YOLO ke indeks bin firmware ESP32.
# Firmware disusun dengan urutan sama:
#   B0=Kaca, B1=Kertas, B2=Logam, B3=Plastik
# Catatan place firmware:
#   - Kaca ditempatkan khusus di X280 Y0 Z40.
#   - Kertas/Logam/Plastik dilepas langsung pada Z safety.
WASTE_CLASS_ID_TO_BIN_INDEX = {
    0: 0,  # Kaca
    1: 1,  # Kertas
    2: 2,  # Logam
    3: 3,  # Plastik
}

# Fallback kalau class-id tidak dikenal. Normalnya tidak dipakai.
ROBOT_BIN_INDEX = 0
SHOW_TARGET_BIN_ON_FRAME = False

# =========================
# HOMOGRAPHY: PIXEL -> ROBOT XY
# =========================

# Default dibuat langsung ke koordinat robot.
# Jadi hasil homography.pixel_to_mm(px, py) = X_robot_mm, Y_robot_mm.
HOMOGRAPHY_MODE = "robot"  # "robot" direkomendasikan; "conveyor" hanya jika perlu debug konveyor
HOMOGRAPHY_PATH = os.path.join(_PROJECT_DIR, "config", "homography_robot.json")

# Titik kalibrasi robot dalam koordinat firmware ESP32.
# Urutan klik di kamera harus sama persis dengan urutan ini.
# Pilih 4 titik yang aman, reachable, tidak segaris, dan berada pada bidang pick.
# Contoh ini membentuk area kerja depan robot:
#   1 kiri-dekat, 2 kanan-dekat, 3 kanan-jauh, 4 kiri-jauh
ROBOT_CALIBRATION_POINTS_MM = [
    [-150.0, 180.0],
    [ 150.0, 180.0],
    [ 150.0, 240.0],
    [-150.0, 240.0],
]

# Batas valid target robot. Target di luar batas ini tidak dikirim ke ESP32.
ROBOT_WORKSPACE_X_MIN_MM = -250.0
ROBOT_WORKSPACE_X_MAX_MM =  250.0
ROBOT_WORKSPACE_Y_MIN_MM =  120.0
ROBOT_WORKSPACE_Y_MAX_MM =  360.0
WORKSPACE_MARGIN_MM = 5.0

# =========================
# ROI
# =========================
# ROI sekarang berbasis 4 TITIK (polygon bebas) yang dipilih manual lewat
# stream kamera live, lalu disimpan ke file JSON dan dipakai ulang oleh
# YOLO.py maupun main.py. Tidak ada lagi mode "homography"/"manual"/
# "interactive" seperti versi lama.
#
# Cara pilih/ubah ROI:
#   - Jalankan:  python roi.py   (buka stream, klik 4 titik, ENTER simpan)
#   - Atau tekan tombol "R" saat main.py berjalan.
ROI_POINTS_PATH = os.path.join(_PROJECT_DIR, "config", "roi_points.json")

# True: pixel di luar polygon 4 titik ROI di-hitamkan sebelum masuk YOLO.
ROI_MASK_OUTSIDE = True

# Jika True, ROI tidak di-resize paksa ke 640x640.
# ROI akan di-letterbox: aspek rasio objek tetap asli, sisa area diberi padding.
# Ini penting untuk YOLO segmentasi karena resize paksa ROI yang pipih/lebar
# dapat membuat bentuk kertas/kaca/logam/plastik berubah dan class mudah salah.
YOLO_USE_LETTERBOX_ROI = True
YOLO_LETTERBOX_COLOR = 114
