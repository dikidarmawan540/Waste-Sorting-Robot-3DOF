#pragma once
#include <Arduino.h>

#define FIRMWARE_NAME "PemilahSampah_ESP32_CONTINUE_BIN_LOG_PICKPLACE"
#define SERIAL_BAUDRATE 115200
#define SERIAL_READY_TOKEN "READY"
#define SERIAL_EVENT_PREFIX "EVENT:"

constexpr uint32_t SERIAL_BOOT_DELAY_MS = 2000;
constexpr uint32_t SERIAL_HEARTBEAT_MS = 5000;
constexpr float CONVEYOR_CENTER_X_MM = 0.0f;
constexpr float CONVEYOR_CENTER_Y_MM = 225.0f;
constexpr float CONVEYOR_SURFACE_Z_MM = 5.0f;
constexpr float PICK_SAFE_CLEARANCE_MM = 100.0f;

// INA219 + touch probe
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define INA219_I2C_ADDR 0x40
#define INA_TOUCH_THRESHOLD_A 0.90f
#define INA_TOUCH_SAMPLE_MS 5

// Parameter kalibrasi (CAL_OFFSET, CAL_SLOPE)
constexpr int INA_CAL_SAMPLES = 20;
constexpr uint32_t INA_CAL_SAMPLE_DELAY_MS = 10;
constexpr int INA_STREAM_SAMPLE_COUNT = 10;

// Pump / Suction cup relay
#define PUMP_RELAY_PIN 33
#define PUMP_RELAY_ACTIVE_HIGH true

// Pick & Place parameters
constexpr float PICK_SAFE_Z_MM = CONVEYOR_SURFACE_Z_MM + PICK_SAFE_CLEARANCE_MM;
constexpr float PICK_PROBE_SPEED_MM_S = 5.0f;

// XYZ calibration mode
constexpr bool XYZ_CALIBRATION_MODE = false;
constexpr float NORMAL_OPERATION_Z_MIN_MM = 6.0f;
constexpr float XYZ_CALIBRATION_Z_MIN_MM = -150.0f;
constexpr float PICK_PROBE_ZMIN_MM = XYZ_CALIBRATION_MODE ? XYZ_CALIBRATION_Z_MIN_MM : NORMAL_OPERATION_Z_MIN_MM;
constexpr float PICK_AFTER_TOUCH_OFFSET_MM = 5.0f;
constexpr uint32_t PICK_SUCTION_SETTLE_MS = 10;
constexpr uint32_t PLACE_RELEASE_DELAY_MS = 10;
constexpr float DEFAULT_PLACE_Z_MM = PICK_SAFE_Z_MM;  // default place: lepas objek di Z safety
constexpr float INA_OVERLOAD_THRESHOLD_A = 0.900f;

// Geometry
constexpr float LINKAGE1_MM = 150.0f;
constexpr float LINKAGE2_MM = 160.0f;
constexpr float CENTER_OFFSET_MM = 0.0f;
constexpr float HEAD_OFFSET_MM = 45.08f;
constexpr float SCARA_OFFSET_X_MM = 0.0f;
constexpr float SCARA_OFFSET_Y_MM = 0.0f;
constexpr float SCARA_OFFSET_Z_MM = 0.0f;
constexpr float AXIS_SCALE_X = 1.0f;
constexpr float AXIS_SCALE_Y = 1.0f;
constexpr float AXIS_SCALE_Z = 1.0f;
constexpr int IK_ELBOW_SIGN = -1;
constexpr float LOW_SHANK_LENGTH_MM = LINKAGE1_MM;
constexpr float HIGH_SHANK_LENGTH_MM = LINKAGE2_MM;
constexpr float END_EFFECTOR_OFFSET_MM = HEAD_OFFSET_MM;
constexpr float SHOULDER_Z_OFFSET_MM = SCARA_OFFSET_Z_MM;

// Joint limits
constexpr float J1_MIN_DEG = -179.0f;
constexpr float J2_MIN_DEG = -170.0f;
constexpr float J3_MIN_DEG = -180.0f;

constexpr float J1_MAX_DEG = 179.0f;
constexpr float J2_MAX_DEG = 170.0f;
constexpr float J3_MAX_DEG = 180.0f;

// ROBOT POSITION
constexpr float BOOT_J1_DEG = 0.0f;
constexpr float BOOT_J2_DEG = 90.0f;
constexpr float BOOT_J3_DEG = 0.0f;

// Limit switch homing
constexpr int J1_LIMIT_PIN = 27;
constexpr int J2_LIMIT_PIN = 25;
constexpr int J3_LIMIT_PIN = 26;
constexpr bool LIMIT_ACTIVE_LOW = true;
constexpr bool LIMIT_USE_INTERNAL_PULLUP = true;

// Arah homing (raw dir level)
constexpr bool J1_HOME_TO_LIMIT_DIR_LEVEL = true;
constexpr bool J2_HOME_TO_LIMIT_DIR_LEVEL = false;
constexpr bool J3_HOME_TO_LIMIT_DIR_LEVEL = true;

// Posisi home setelah G28 (sudut kinematic, bukan raw offset limit switch): target J1=0 J2=90 J3=0 -> X=0 Y=205 Z=150
constexpr float J1_HOME_ANGLE_DEG = 0.0f;
constexpr float J2_HOME_ANGLE_DEG = 90.0f;
constexpr float J3_HOME_ANGLE_DEG = 0.0f;

// Offset derajat RAW dari limit switch ke pose home fisik (positif = DIR HIGH, negatif = DIR LOW)
constexpr float J1_HOME_OFFSET_DEG = -186.0f;  // isi dari SAVEHOME J1
constexpr float J2_HOME_OFFSET_DEG = 18.5f;    // isi dari SAVEHOME J2
constexpr float J3_HOME_OFFSET_DEG = -1.3f;    // isi dari SAVEHOME J3

constexpr uint32_t HOMING_STEP_INTERVAL_US = 2000;
constexpr long HOMING_MAX_STEPS = 15000;
constexpr long HOMING_BACKOFF_STEPS = 100;  // 0 agar HOME_OFFSET dihitung langsung dari titik LS

// Stepper calibration
constexpr float MOTOR_STEPS_PER_REV = 200.0f;
constexpr float MICROSTEPS = 8.0f;

constexpr float J1_GEAR_RATIO = 5.00f;
constexpr float J2_GEAR_RATIO = 4.36f;
constexpr float J3_GEAR_RATIO = 4.36f;

constexpr float J1_STEPS_PER_DEG = (MOTOR_STEPS_PER_REV * MICROSTEPS * J1_GEAR_RATIO) / 360.0f;
constexpr float J2_STEPS_PER_DEG = (MOTOR_STEPS_PER_REV * MICROSTEPS * J2_GEAR_RATIO) / 360.0f;
constexpr float J3_STEPS_PER_DEG = (MOTOR_STEPS_PER_REV * MICROSTEPS * J3_GEAR_RATIO) / 360.0f;

// Arah motor untuk gerak normal (G0/G1, moveToAngles, IK Cartesian) — tidak dipakai oleh G28/CALHOME
constexpr bool J1_DIR_INVERT = false;
constexpr bool J2_DIR_INVERT = true;
constexpr bool J3_DIR_INVERT = false;

// ESP32 pins
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

// Profil gerak AccelStepper normal
constexpr float J1_MAX_SPEED_STEPS_PER_SEC = 2200.0f;
constexpr float J2_MAX_SPEED_STEPS_PER_SEC = 1800.0f;
constexpr float J3_MAX_SPEED_STEPS_PER_SEC = 1800.0f;
constexpr float J1_ACCEL_STEPS_PER_SEC2 = 1000.0f;
constexpr float J2_ACCEL_STEPS_PER_SEC2 = 1000.0f;
constexpr float J3_ACCEL_STEPS_PER_SEC2 = 1000.0f;

constexpr float SEGMENTS_PER_MM = 1.0f;
constexpr int MIN_CARTESIAN_SEGMENTS = 3;
constexpr int MAX_CARTESIAN_SEGMENTS = 25;

// Jumlah segmen moveToCartesianLinear berdasarkan sudut sendi yang berubah; makin kecil nilai = makin halus tapi lebih banyak segmen
constexpr float DEG_PER_CARTESIAN_SEGMENT = 3.0f;

// Motion blending G1 linear Cartesian: segmen berikutnya diberikan sebelum segmen aktif berhenti total, agar gerak tidak stop-start
constexpr bool ENABLE_MOTION_BLENDING = true;
constexpr float MOTION_BLEND_START_FRACTION = 0.55f;
constexpr long MOTION_BLEND_MIN_WINDOW_STEPS = 14;
constexpr long MOTION_BLEND_MAX_WINDOW_STEPS = 180;
constexpr uint32_t CARTESIAN_MOTION_TIMEOUT_MS = 60000UL;

constexpr float DEFAULT_FEEDRATE_MM_MIN = 4000.0f;

// Workspace envelope untuk IK; R_MIN_MM menghindari tabrakan ke badan base, Z_MIN_MM mengikuti mode kalibrasi
constexpr float R_MIN_MM = 130.0f;
constexpr float R_MAX_MM = LINKAGE1_MM + LINKAGE2_MM + HEAD_OFFSET_MM - 5.0f;
constexpr float Z_MIN_MM = PICK_PROBE_ZMIN_MM;
constexpr float Z_MAX_MM = LINKAGE1_MM + LINKAGE2_MM + 40.0f;

// Pose default setelah homing/boot: lower arm vertikal, high arm horizontal
constexpr float HOME_90_X_MM = 0.0f;
constexpr float HOME_90_Y_MM = HIGH_SHANK_LENGTH_MM + END_EFFECTOR_OFFSET_MM;
constexpr float HOME_90_Z_MM = LOW_SHANK_LENGTH_MM;

constexpr float INITIAL_X_MM = HOME_90_X_MM;
constexpr float INITIAL_Y_MM = HOME_90_Y_MM;
constexpr float INITIAL_Z_MM = HOME_90_Z_MM;

// Interval baca INA219 saat Z pick-down turun kontinu (non-blocking)
constexpr float PICK_MONITOR_SAMPLE_MS = 15.0f;

// Blind time + debounce untuk deteksi sentuh, agar lonjakan arus fase akselerasi awal tidak terbaca sebagai kontak fisik
constexpr uint32_t PICK_TOUCH_BLIND_TIME_MS = 250;
constexpr int PICK_TOUCH_DEBOUNCE_SAMPLES = 3;

// Mode gerak Z pick down: true = tersegmentasi + blending seperti G1 manual, false = gaya G0 (tiap sendi independen di MAX_SPEED)
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

// (B0=KACA, B1=KERTAS, B2=LOGAM, B3=PLASTIK)
constexpr float GLASS_PLACE_Z_MM = 40.0f;
static constexpr BinPoseConfig WASTE_BINS[] = {
  { "KACA",    280.0f,    0.0f, GLASS_PLACE_Z_MM },
  { "KERTAS",  100.0f, -200.0f, PICK_SAFE_Z_MM },
  { "LOGAM",   200.0f, -200.0f, PICK_SAFE_Z_MM },
  { "PLASTIK",-150.0f, -200.0f, PICK_SAFE_Z_MM }
};

constexpr int WASTE_BIN_COUNT = sizeof(WASTE_BINS) / sizeof(WASTE_BINS[0]);

