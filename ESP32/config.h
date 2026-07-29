#pragma once
#include <Arduino.h>

// =============================================================
// Firmware robot pemilah sampah ESP32 - parallel/parallelogram arm
// REVISI BASE-ORIGIN: titik koordinat Cartesian berada di poros/base robot
// =============================================================

#define FIRMWARE_NAME "PemilahSampah_ESP32_CONTINUE_BIN_LOG_PICKPLACE"
#define SERIAL_BAUDRATE 115200

// ===================== Integrasi YOLO (Python/VSCode) =================
#define SERIAL_READY_TOKEN "READY"
#define SERIAL_EVENT_PREFIX "EVENT:"

constexpr uint32_t SERIAL_BOOT_DELAY_MS = 2000;
constexpr uint32_t SERIAL_HEARTBEAT_MS = 5000;

// =====================================================================
// KONSEP KOORDINAT — TITIK NOL = BASE/POROS ROBOT
// =====================================================================
// Sistem koordinat mengikuti Cartesian Guide:
//   X=0 Y=0 Z=0  -> pusat poros/base robot.
//   +X           -> kanan robot pada top view.
//   +Y           -> arah depan/jangkauan lengan robot.
//   +Z           -> arah atas.
//
// Titik conveyor BUKAN nol. Semua nilai di bawah diukur dari pusat poros/base.
// Default fisik:
//   Conveyor center = X0, Y225
//   Safe pick/home  = 100 mm di atas belt = Z54
constexpr float CONVEYOR_CENTER_X_MM = 0.0f;
constexpr float CONVEYOR_CENTER_Y_MM = 225.0f;
constexpr float CONVEYOR_SURFACE_Z_MM = 5.0f;
constexpr float PICK_SAFE_CLEARANCE_MM = 100.0f;

// ===================== INA219 + touch probe ===================
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define INA219_I2C_ADDR 0x40
#define INA_TOUCH_THRESHOLD_A 0.90f
#define INA_TOUCH_SAMPLE_MS 5

// Parameter kalibrasi (CAL_OFFSET, CAL_SLOPE) ada di INA219.h — ubah di sana.
constexpr int INA_CAL_SAMPLES = 20;
constexpr uint32_t INA_CAL_SAMPLE_DELAY_MS = 10;

// "INA" command: 10 printed samples, each = average of INA_CAL_SAMPLES raw reads
constexpr int INA_STREAM_SAMPLE_COUNT = 10;

// ===================== Pump / Suction cup relay ================
#define PUMP_RELAY_PIN 33
#define PUMP_RELAY_ACTIVE_HIGH true

// ===================== Pick & Place parameters ==================
constexpr float PICK_SAFE_Z_MM = CONVEYOR_SURFACE_Z_MM + PICK_SAFE_CLEARANCE_MM;
constexpr float PICK_PROBE_SPEED_MM_S = 5.0f;

// ===================== XYZ calibration mode ===================
// Mode ini dibuka untuk kalibrasi presisi X/Y/Z manual.
// Saat true, perintah G0/G1 dan probe Z boleh menerima target Z negatif
// sampai XYZ_CALIBRATION_Z_MIN_MM. Setelah kalibrasi selesai dan robot dipakai
// normal/sort otomatis, kembalikan nilai ini ke false atau naikkan Z minimum
// ke batas aman mekanik.
constexpr bool XYZ_CALIBRATION_MODE = false;
constexpr float NORMAL_OPERATION_Z_MIN_MM = 6.0f;
constexpr float XYZ_CALIBRATION_Z_MIN_MM = -150.0f;
constexpr float PICK_PROBE_ZMIN_MM = XYZ_CALIBRATION_MODE ? XYZ_CALIBRATION_Z_MIN_MM : NORMAL_OPERATION_Z_MIN_MM;
// Setelah arus INA219 terkalibrasi regresi linear mencapai threshold,
// end-effector tetap turun lagi sebesar offset ini agar suction cup yang
// berspring benar-benar menekan objek.
//
// Catatan revisi: logic rising-edge, blind-zone, dan debounce dihapus.
// INA219 hanya dibaca sebagai nilai arus hasil regresi linear; 1 sampel
// calibratedCurrentA() >= threshold langsung dianggap deteksi arus.
// REVISI-5 (motion.h): offsetTargetZ (= Z_deteksi - PICK_AFTER_TOUCH_OFFSET_MM)
// sekarang di-clamp supaya tidak pernah diminta turun di bawah PICK_PROBE_ZMIN_MM.
// Tanpa clamp ini, kalau arus 0.9A baru terdeteksi saat Z sudah < (zMin + 20mm),
// syarat offset penuh jadi mustahil tercapai secara fisik (lengan sudah mentok
// di lantai zMin duluan) -> robot tidak pernah lanjut ke bin. Kalau ingin objek
// lebih "ditekan" sebelum dianggap tergenggam, kecilkan angka 20.0f di bawah ini
// supaya marginnya lebih longgar terhadap PICK_PROBE_ZMIN_MM.
constexpr float PICK_AFTER_TOUCH_OFFSET_MM = 5.0f;
constexpr uint32_t PICK_SUCTION_SETTLE_MS = 10;
constexpr uint32_t PLACE_RELEASE_DELAY_MS = 10;
constexpr float DEFAULT_PLACE_Z_MM = PICK_SAFE_Z_MM;  // default place: lepas objek di Z safety

constexpr float INA_OVERLOAD_THRESHOLD_A = 0.900f;

// ========================== Geometry ==========================
constexpr float LINKAGE1_MM = 150.0f;
constexpr float LINKAGE2_MM = 160.0f;
constexpr float CENTER_OFFSET_MM = 0.0f;
constexpr float HEAD_OFFSET_MM = 45.08f;

// BASE-ORIGIN: tidak ada translasi koordinat firmware.
// Offset harus 0 agar X/Y/Z langsung berarti jarak dari poros robot.
constexpr float SCARA_OFFSET_X_MM = 0.0f;
constexpr float SCARA_OFFSET_Y_MM = 0.0f;
constexpr float SCARA_OFFSET_Z_MM = 0.0f;

constexpr float AXIS_SCALE_X = 1.0f;
constexpr float AXIS_SCALE_Y = 1.0f;
constexpr float AXIS_SCALE_Z = 1.0f;

// Cabang IK. -1 dipakai untuk cabang yang sesuai reference firmware lama kamu.
constexpr int IK_ELBOW_SIGN = -1;

// Alias agar file kinematic/motion lama tetap compile.
constexpr float LOW_SHANK_LENGTH_MM = LINKAGE1_MM;
constexpr float HIGH_SHANK_LENGTH_MM = LINKAGE2_MM;
constexpr float END_EFFECTOR_OFFSET_MM = HEAD_OFFSET_MM;
constexpr float SHOULDER_Z_OFFSET_MM = SCARA_OFFSET_Z_MM;

// ----------------------- Joint limits -------------------------
// J1 (rotasi base): dibuka penuh sampai +/-179 derajat (hanya menghindari nilai
// tepat +/-180 karena tie-break atan2/normalize) agar Y bisa negatif (belakang
// robot). Proteksi terhadap tabrakan ke badan base BUKAN lewat sudut, tapi lewat
// R_MIN_MM (jarak radial minimum dari poros base) -- lihat bagian "Reachable
// envelope" di bawah.
constexpr float J1_MIN_DEG = -179.0f;
constexpr float J2_MIN_DEG = -170.0f;
constexpr float J3_MIN_DEG = -180.0f;

constexpr float J1_MAX_DEG = 179.0f;
constexpr float J2_MAX_DEG = 170.0f;
constexpr float J3_MAX_DEG = 180.0f;

// =====================================================================
// POSISI BOOT (BOOT_J*_DEG)
// =====================================================================
// Dipakai hanya sebagai estimasi internal saat board baru menyala.
// Status tetap belum homing sampai G28/G93 sukses.
constexpr float BOOT_J1_DEG = 0.0f;
constexpr float BOOT_J2_DEG = 90.0f;
constexpr float BOOT_J3_DEG = 0.0f;

// -------------------- Limit switch homing --------------------
constexpr int J1_LIMIT_PIN = 27;
constexpr int J2_LIMIT_PIN = 25;
constexpr int J3_LIMIT_PIN = 26;

constexpr bool LIMIT_ACTIVE_LOW = true;
constexpr bool LIMIT_USE_INTERNAL_PULLUP = true;

// Backward-compatible alias untuk kode lama.
constexpr bool LIMIT_SWITCH_ACTIVE_LOW = LIMIT_ACTIVE_LOW;

// =====================================================================
// ARAH HOMING — RAW DIR LEVEL, TERPISAH DARI DIR_INVERT
// =====================================================================
// J*_HOME_TO_LIMIT_DIR_LEVEL = level fisik pin DIR saat mencari limit.
// true  -> DIR pin HIGH
// false -> DIR pin LOW
// Nilai ini TIDAK melewati J*_DIR_INVERT.
constexpr bool J1_HOME_TO_LIMIT_DIR_LEVEL = true;
constexpr bool J2_HOME_TO_LIMIT_DIR_LEVEL = false;
constexpr bool J3_HOME_TO_LIMIT_DIR_LEVEL = true;

// =====================================================================
// POSISI HOME SETELAH G28 (J*_HOME_ANGLE_DEG)
// =====================================================================
// Sudut kinematic reference. BUKAN raw offset dari limit switch.
//
// Target revisi setelah G28 + M114:
//   J1=0.000 J2=90.000 J3=0.000 | X=0.00 Y=205.00 Z=150.00
//
// Artinya:
//   J1 = 0°  -> base menghadap depan / +Y.
//   J2 = 90° -> lower shank vertikal.
//   J3 = 0°  -> high shank horizontal.
//
// Nilai -187.920, 21.984, dan 0.0 tetap dipakai di HOME_OFFSET_DEG
// sebagai jarak RAW dari limit switch ke pose home fisik.
constexpr float J1_HOME_ANGLE_DEG = 0.0f;
constexpr float J2_HOME_ANGLE_DEG = 90.0f;
constexpr float J3_HOME_ANGLE_DEG = 0.0f;

// Alias agar kode lama tetap compile.
constexpr float J1_HOME_DEG = J1_HOME_ANGLE_DEG;
constexpr float J2_HOME_DEG = J2_HOME_ANGLE_DEG;
constexpr float J3_HOME_DEG = J3_HOME_ANGLE_DEG;

// ---- Offset derajat RAW setelah limit switch tersentuh ----
// positif -> motor bergerak dengan DIR pin HIGH
// negatif -> motor bergerak dengan DIR pin LOW
constexpr float J1_HOME_OFFSET_DEG = -186.0f;  // isi dari SAVEHOME J1
constexpr float J2_HOME_OFFSET_DEG = 18.5f;  // isi dari SAVEHOME J2
constexpr float J3_HOME_OFFSET_DEG = -1.3f;   // isi dari SAVEHOME J3

constexpr uint32_t HOMING_STEP_INTERVAL_US = 2000;
constexpr long HOMING_MAX_STEPS = 15000;
constexpr long HOMING_BACKOFF_STEPS = 100;  // 0 agar HOME_OFFSET dihitung langsung dari titik LS

// ------------------- Stepper calibration ----------------------
constexpr float MOTOR_STEPS_PER_REV = 200.0f;
constexpr float MICROSTEPS = 8.0f;

constexpr float J1_GEAR_RATIO = 5.00f;
constexpr float J2_GEAR_RATIO = 4.36f;
constexpr float J3_GEAR_RATIO = 4.36f;

constexpr float J1_STEPS_PER_DEG = (MOTOR_STEPS_PER_REV * MICROSTEPS * J1_GEAR_RATIO) / 360.0f;
constexpr float J2_STEPS_PER_DEG = (MOTOR_STEPS_PER_REV * MICROSTEPS * J2_GEAR_RATIO) / 360.0f;
constexpr float J3_STEPS_PER_DEG = (MOTOR_STEPS_PER_REV * MICROSTEPS * J3_GEAR_RATIO) / 360.0f;

// Arah motor untuk gerak normal (G0/G1, moveToAngles, IK Cartesian).
// DIR_INVERT TIDAK dipakai oleh G28/CALHOME dan tidak mempengaruhi arah limit switch.
constexpr bool J1_DIR_INVERT = false;
constexpr bool J2_DIR_INVERT = true;
constexpr bool J3_DIR_INVERT = false;

// Backward-compatible sign multiplier untuk motion lama.
constexpr int J1_MOTOR_DIR = J1_DIR_INVERT ? -1 : 1;
constexpr int J2_MOTOR_DIR = J2_DIR_INVERT ? -1 : 1;
constexpr int J3_MOTOR_DIR = J3_DIR_INVERT ? -1 : 1;

// ------------------------ ESP32 pins -------------------------
constexpr int J1_STEP_PIN = 5;
constexpr int J1_DIR_PIN = 17;
constexpr int J1_EN_PIN = -1;

constexpr int J2_STEP_PIN = 19;
constexpr int J2_DIR_PIN = 18;
constexpr int J2_EN_PIN = -1;

constexpr int J3_STEP_PIN = 16;
constexpr int J3_DIR_PIN = 4;
constexpr int J3_EN_PIN = -1;

constexpr bool ENABLE_ACTIVE_LOW = true;
constexpr bool STEP_ACTIVE_HIGH = true;

constexpr uint32_t STEP_PULSE_US = 4;
constexpr uint32_t DIR_SETUP_US = 5;
constexpr uint32_t MIN_STEP_INTERVAL_US = 450;

// Profil gerak AccelStepper normal.
constexpr float J1_MAX_SPEED_STEPS_PER_SEC = 2200.0f;  // FAST: dinaikkan dari 1400
constexpr float J2_MAX_SPEED_STEPS_PER_SEC = 1800.0f;  // FAST: dinaikkan dari 1200
constexpr float J3_MAX_SPEED_STEPS_PER_SEC = 1800.0f;  // FAST: dinaikkan dari 1200
constexpr float J1_ACCEL_STEPS_PER_SEC2 = 1000.0f;     // FAST: dinaikkan dari 900
constexpr float J2_ACCEL_STEPS_PER_SEC2 = 1000.0f;     // FAST: dinaikkan dari 800
constexpr float J3_ACCEL_STEPS_PER_SEC2 = 1000.0f;     // FAST: dinaikkan dari 800

constexpr float SEGMENTS_PER_MM = 1.0f;  
constexpr int MIN_CARTESIAN_SEGMENTS = 3;
constexpr int MAX_CARTESIAN_SEGMENTS = 25;

// Dipakai moveToCartesianLinear untuk menentukan jumlah segmen berdasarkan
// sudut sendi yang berubah (bukan jarak mm). Makin kecil nilainya -> makin
// halus/lurus tapi makin banyak segmen (lebih banyak stop-start kecil).
// Makin besar -> makin sedikit segmen, tiap segmen lebih panjang jadi motor
// sempat capai cruise speed (lebih cepat & mulus), dengan sedikit trade-off
// akurasi lintasan antar-waypoint. 3 derajat/segmen sudah menghasilkan
// deviasi sub-mm untuk panjang lengan robot ini.
constexpr float DEG_PER_CARTESIAN_SEGMENT = 3.0f;

// Motion blending untuk G1 linear Cartesian.
// Target segmen berikutnya diberikan sebelum segmen aktif benar-benar berhenti,
// sehingga gerakan lebih menyambung dan tidak terasa stop-start di tiap segmen.
constexpr bool ENABLE_MOTION_BLENDING = true;
constexpr float MOTION_BLEND_START_FRACTION = 0.55f;  // mulai blend saat sisa jarak joint ±55% segmen (dulu 0.35)
constexpr long MOTION_BLEND_MIN_WINDOW_STEPS = 14;     // dulu 8: window terlalu kecil bikin tiap segmen tetap rem penuh
constexpr long MOTION_BLEND_MAX_WINDOW_STEPS = 180;
constexpr uint32_t CARTESIAN_MOTION_TIMEOUT_MS = 60000UL;

constexpr float DEFAULT_FEEDRATE_MM_MIN = 4000.0f;  // default feedrate referensi

// Workspace envelope untuk IK.
// Catatan kalibrasi XYZ: Z_MIN_MM mengikuti PICK_PROBE_ZMIN_MM, sehingga saat
// XYZ_CALIBRATION_MODE=true robot boleh diuji ke Z negatif untuk mencari nol fisik.
// R_MIN_MM = jarak radial minimum dari poros/pusat base (berlaku ke segala arah,
// depan maupun belakang/Y-). Base robot berbentuk lingkaran cukup besar, jadi
// target yang lebih dekat dari ini berisiko nabrak badan base -> dianggap error
// di RobotKinematic::inverse().
constexpr float R_MIN_MM = 130.0f;
constexpr float R_MAX_MM = LINKAGE1_MM + LINKAGE2_MM + HEAD_OFFSET_MM - 5.0f;
constexpr float Z_MIN_MM = PICK_PROBE_ZMIN_MM;  // mode kalibrasi: IK menerima Z negatif sampai XYZ_CALIBRATION_Z_MIN_MM
constexpr float Z_MAX_MM = LINKAGE1_MM + LINKAGE2_MM + 40.0f;

// Pose default setelah homing/boot.
// Sesuai Cartesian Guide: lower arm vertikal dan high arm horizontal.
constexpr float HOME_90_X_MM = 0.0f;
constexpr float HOME_90_Y_MM = HIGH_SHANK_LENGTH_MM + END_EFFECTOR_OFFSET_MM;
constexpr float HOME_90_Z_MM = LOW_SHANK_LENGTH_MM;

constexpr float INITIAL_X_MM = HOME_90_X_MM;
constexpr float INITIAL_Y_MM = HOME_90_Y_MM;
constexpr float INITIAL_Z_MM = HOME_90_Z_MM;

// Kompatibilitas sequence lama.
constexpr float PICK_APPROACH_Z_OFFSET_MM = 45.0f;
constexpr float PICK_DESCEND_STEP_MM = 2.0f;  // sudah TIDAK dipakai lagi oleh probePickDown (lihat REVISI di bawah),
                                               // dibiarkan ada untuk kompatibilitas kalau ada kode lain yang refer.

// REVISI (fix "Z pick down lambat"): dulu probePickDown() memecah turun jadi
// puluhan langkah G1 terpisah sejauh PICK_DESCEND_STEP_MM (2mm), dan SETIAP
// langkah adalah gerakan blocking yang berhenti total (decel ke 0) sebelum
// baca arus INA219 lalu jalan lagi. Untuk jarak sependek 2mm, motor bahkan
// tidak pernah sampai ke MAX_SPEED yang sudah di-tuning (profilnya segitiga,
// bukan trapesium) -> tuning G1 MAX_SPEED tidak berpengaruh sama sekali di
// fase ini, dan overhead start-stop x ~70 langkah (150mm -> 5mm / 2mm) itulah
// yang bikin "Z pick down" terasa lambat/patah-patah.
//
// Perbaikan: turun dalam SATU gerakan kontinu ke Zmin, arus INA219 dipantau
// tiap PICK_MONITOR_SAMPLE_MS milidetik TANPA memberhentikan motor (non-blocking,
// tidak pakai delay() dari averageCurrentA). Begitu syarat terpenuhi
// (arus >= threshold DAN sudah turun offset tambahan), motor baru direm
// halus pakai AccelStepper::stop() (tetap pakai profil accel yang sudah
// di-tuning, bukan hard-stop).
constexpr float PICK_MONITOR_SAMPLE_MS = 15.0f;  // interval baca INA219 saat descend kontinu (non-blocking)

// REVISI-6 (fix "tidak mau turun sampai Z surface untuk objek pendek"):
// Sebelumnya rising-edge/blind-zone/debounce dihapus total (lihat komentar
// REVISI-5 di atas), jadi 1 sample arus >= threshold langsung dianggap
// "sudah menyentuh objek". Masalahnya: begitu descend mulai, motor J2/J3
// mengalami lonjakan arus akibat FASE AKSELERASI (bukan kontak fisik). Kalau
// sample pertama kebetulan diambil saat lonjakan itu, currentDetected jadi
// true padahal end-effector masih jauh di atas permukaan -- offsetTargetZ
// lalu dihitung dari posisi palsu itu, dan lengan cuma turun 20mm dari situ
// lalu berhenti (kelihatan seperti "menolak turun sampai Z_MIN").
//
// Untuk objek TINGGI ini nyaris tidak kelihatan (titik sentuh asli memang
// dekat awal descend). Untuk objek PENDEK/TIPIS (tutup botol dsb.) yang titik
// sentuh aslinya harus dekat Z_MIN, efeknya jadi jelas: lengan berhenti jauh
// di atas objek, tidak pernah benar-benar menyentuhnya.
//
// PICK_TOUCH_BLIND_TIME_MS     : abaikan semua sample arus dalam periode ini
//   sejak descend mulai (durasi kira-kira setara fase akselerasi motor),
//   supaya lonjakan start-up tidak pernah dihitung sebagai sentuhan.
// PICK_TOUCH_DEBOUNCE_SAMPLES  : jumlah sample BERURUTAN (interval
//   PICK_MONITOR_SAMPLE_MS) yang harus >= threshold sebelum sentuhan
//   dianggap valid -- menyaring noise/spike sesaat.
constexpr uint32_t PICK_TOUCH_BLIND_TIME_MS = 250;
constexpr int PICK_TOUCH_DEBOUNCE_SAMPLES = 3;

// REVISI-3: pilihan mode gerak untuk Z pick down (descendWithCurrentMonitor).
// true  = gaya G1: sekarang disegmentasi + blending PERSIS seperti
//         moveToCartesianLinear() (dipakai perintah manual "G1 Z.."), supaya
//         "Z pick down" gerak sama dengan G1 manual, bukan cuma sama config
//         speed/accel-nya saja.
// false = gaya G0 (restoreAllNormalSpeeds + moveTo langsung ke target akhir,
//         tiap sendi independen di MAX_SPEED-nya sendiri-sendiri).
constexpr bool PICK_DESCEND_USE_G1_SYNC = true;

constexpr uint32_t PUMP_SETTLE_MS = PICK_SUCTION_SETTLE_MS;
constexpr uint32_t DROP_SETTLE_MS = PLACE_RELEASE_DELAY_MS;

// -------------------- Bin / penampungan -----------------------
struct BinPoseConfig {
  const char* name;
  float x;
  float y;
  float z;
};

// Titik bin/penampungan berdasarkan class-id YOLO.
// Urutan wajib sama dengan Python/config.py:
//   B0=KACA, B1=KERTAS, B2=LOGAM, B3=PLASTIK
// Koordinat X/Y wajib reachable terhadap R_MIN_MM (jangan terlalu dekat poros,
// supaya tidak nabrak badan base) dan <= R_MAX_MM.
// Revisi place:
//   - KACA turun ke X280 Y0 Z40 sebelum PUMP OFF.
//   - Kelas lain langsung PUMP OFF pada Z safety di koordinat bin masing-masing.
constexpr float GLASS_PLACE_Z_MM = 40.0f;
static constexpr BinPoseConfig WASTE_BINS[] = {
  { "KACA",    280.0f,   0.0f, GLASS_PLACE_Z_MM },
  { "KERTAS",  100.0f, -200.0f, PICK_SAFE_Z_MM },
  { "LOGAM",   200.0f, -200.0f, PICK_SAFE_Z_MM },
  { "PLASTIK",-150.0f, -200.0f, PICK_SAFE_Z_MM }
};

constexpr int WASTE_BIN_COUNT = sizeof(WASTE_BINS) / sizeof(WASTE_BINS[0]);

// -------------------- Optional gripper -----------------------
#define USE_GRIPPER_SERVO 0
constexpr int GRIPPER_SERVO_PIN = -1;
