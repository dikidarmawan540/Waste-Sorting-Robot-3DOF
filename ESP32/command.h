#pragma once

#include <Arduino.h>
#include "config.h"
#include "pump.h"
#include "INA219.h"
#include "kineamtic.h"
#include "motion.h"

class CommandProcessor {
public:
  CommandProcessor(MotionController& motion,
                   RobotKinematic& kinematic,
                   PumpController& pump,
                   INA219Monitor& ina,
                   Stream& io)
    : _motion(motion), _kinematic(kinematic), _pump(pump), _ina(ina), _io(io) {}

  void begin() {
    _buffer.reserve(120);
    printHelp();
    printPickPlaceCsvHeader();
  }

  void update() {
    if (_streamingIna) {
      updateInaStream();
      return;
    }

    while (_io.available()) {
      const char c = static_cast<char>(_io.read());
      if (c == '\n' || c == '\r') {
        if (_buffer.length() > 0) {
          process(_buffer);
          _buffer = "";
        }
      } else {
        if (_buffer.length() < 118) _buffer += c;
      }
    }
  }

  void printHelp() {
    _io.println();
    _io.println(F("Pemilah Sampah ESP32 Commands"));
    _io.println(F("--------------------------------"));
    _io.println(F("HELP                         : show this menu"));
    _io.println(F("G28                          : home J3, J2, then J1"));
    _io.println(F("M119                         : show limit switch states"));
    _io.println(F("M114                         : show current joint and XYZ estimate"));
    _io.println(F("CALXYZ                       : show XYZ calibration parameters"));
    _io.println(F("INA                          : print 10 samples, each = average of 20 reads (2 decimals)"));
    _io.println(F("PUMP ON | PUMP OFF           : control suction relay"));
    _io.println(F("G0 X0 Y205 Z150              : rapid Cartesian endpoint move"));
    _io.println(F("G1 X200 Y205 Z150            : linear Cartesian move + motion blending agar lebih halus"));
    _io.println(F("MOVEJ A0 B90 C0              : joint move, A=J1 B=J2 C=J3 in deg"));
    _io.println(F("SORT X0 Y225 Z40 B0 [IDn]    : YOLO pick command; tetap lanjut ke bin walau INA tidak deteksi"));
    _io.println(F("YOLO                         : show last YOLO coordinate/status received by ESP32"));
    _io.println(F("HOMING REQUIRED              : MOVEJ/G0/G1/SORT ditolak sampai G28 sukses"));
    _io.println(F("BINS                         : list configured bin coordinates"));
    _io.println(F("YOLO ACK                     : after SORT/PICK, ESP32 replies:"));
    _io.println(F("                               EVENT:YOLO_COORD_RECEIVED X.. Y.. Z.. B.."));
    _io.println(F("                               EVENT:PUMP_ON_AT_PICK_XY"));
    _io.println(F("YOLO status check            : ketik YOLO untuk cek last coordinate dari Python/YOLO; jika belum ada, PUMP OFF"));
    _io.println(F("Safe sequence                : G0 Zsafe -> G0 XY -> PUMP ON -> G1 turun ke Zmin/INA -> BIN -> PUMP OFF"));
    _io.println(F("Pick result CSV              : DATA:PICK_PLACE_CSV,... hanya dikirim saat siklus SORT/pick-place"));
    _io.println(F("No home behavior             : SORT tetap ACK YOLO, tapi PUMP OFF dan motor tidak bergerak sebelum G28"));
    _io.println(F("Z calibration mode           : G0/G1 boleh Z negatif sampai XYZ_CALIBRATION_Z_MIN_MM"));
    _io.println(F("Note: +Y=front, origin is base/poros robot. M119 shows raw limit states."));
    _io.println();
  }

private:
  MotionController& _motion;
  RobotKinematic& _kinematic;
  PumpController& _pump;
  INA219Monitor& _ina;
  Stream& _io;
  String _buffer;

  // INA219 calibration streaming state
  bool _streamingIna = false;
  uint32_t _inaSampleIndex = 0;

  // YOLO receive/status state
  bool _hasYoloCoord = false;
  uint32_t _yoloReceiveCount = 0;
  uint32_t _lastYoloReceivedMs = 0;
  CartesianPose _lastYoloPick = {INITIAL_X_MM, INITIAL_Y_MM, DEFAULT_PLACE_Z_MM};
  int _lastYoloBin = 0;
  int _lastYoloObjectId = -1;
  bool _lastYoloWasHomed = false;
  bool _lastYoloWasAcceptedForMotion = false;
  uint32_t _pickPlaceTrialCount = 0;

  void startInaStream() {
    _streamingIna = true;
    _inaSampleIndex = 0;

    // flush any leftover input so a stray newline doesn't instantly stop the stream
    while (_io.available()) _io.read();
    _buffer = "";

    _io.println();
    _io.println(F("=== INA219 Calibration: 10 sample (tiap sample = rata-rata 20 pembacaan) ==="));
    _io.println(F("INAavg = raw rata-rata (untuk data regresi, bandingkan dengan multimeter)."));
    _io.println(F("INACLB = hasil regresi linear (aktif permanen; ubah CAL_OFFSET/CAL_SLOPE"));
    _io.println(F("         dan baris return diaktifkan di INA219.h::calibratedCurrentA)."));
    _io.println(F("(Kirim karakter apa saja untuk berhenti lebih awal.)"));
    _io.println(F("idx , INAavg   , INACLB"));
  }

  void stopInaStream() {
    _streamingIna = false;
    _io.println(F("=== Streaming INA219 selesai ==="));
    printHelp();
  }

  void updateInaStream() {
    // Any incoming byte stops the stream early (checked before each ~200ms average).
    if (_io.available()) {
      while (_io.available()) _io.read();
      stopInaStream();
      return;
    }

    // Blocks ~ INA_CAL_SAMPLES * INA_CAL_SAMPLE_DELAY_MS (default 20*10=200ms).
    // Acceptable here since "INA" is only run during idle calibration sessions.
    const float avgRawA = _ina.averageRawCurrentA();
    const float avgClbA = _ina.averageCurrentA();
    _io.print(_inaSampleIndex + 1);
    _io.print(F(" , "));
    _io.print(avgRawA, 2);
    _io.print(F(" , "));
    _io.println(avgClbA, 2);

    _inaSampleIndex++;
    if (_inaSampleIndex >= INA_STREAM_SAMPLE_COUNT) {
      stopInaStream();
    }
  }

  void process(String line) {
    line.trim();
    if (line.length() == 0) return;

    String upper = line;
    upper.toUpperCase();

    _io.print(F("> "));
    _io.println(line);

    if (upper == "HELP" || upper == "?") {
      printHelp();
      return;
    }

    if (upper.startsWith("G28")) {
      _motion.homeAll(_io);
      return;
    }

    if (upper.startsWith("M119")) {
      _motion.printLimitStatus(_io);
      return;
    }

    if (upper.startsWith("M114")) {
      _motion.printPosition(_io, _kinematic);
      _io.print(F("Pump="));
      _io.println(_pump.isOn() ? F("ON") : F("OFF"));
      return;
    }

    if (upper.startsWith("CALXYZ")) {
      printCalibrationParams();
      return;
    }

    if (upper.startsWith("INA")) {
      startInaStream();
      return;
    }

    if (upper == "YOLO" || upper == "YOLO?") {
      printYoloStatus();
      return;
    }

    if (upper.startsWith("PUMP")) {
      if (upper.indexOf("ON") >= 0) {
        _pump.on();
        _io.println(F("Pump ON"));
      } else if (upper.indexOf("OFF") >= 0) {
        _pump.off();
        _io.println(F("Pump OFF"));
      } else {
        _pump.toggle();
        _io.println(_pump.isOn() ? F("Pump ON") : F("Pump OFF"));
      }
      return;
    }

    if (upper.startsWith("BINS")) {
      printBins();
      return;
    }

    if (upper.startsWith("MOVEJ")) {
      handleMoveJ(upper);
      return;
    }

    if (upper.startsWith("G0") || upper.startsWith("G1")) {
      handleCartesianMove(upper, upper.startsWith("G1"));
      return;
    }

    if (upper.startsWith("SORT") || upper.startsWith("PICK")) {
      handleSort(upper);
      return;
    }

    _io.println(F("ERROR: unknown command. Type HELP."));
  }

  bool readFloatParam(const String& line, char key, float& value) {
    const int idx = line.indexOf(key);
    if (idx < 0) return false;

    int start = idx + 1;
    while (start < line.length() && (line[start] == ' ' || line[start] == '=' || line[start] == ':')) start++;

    int end = start;
    while (end < line.length()) {
      const char c = line[end];
      if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.') end++;
      else break;
    }

    if (end <= start) return false;
    value = line.substring(start, end).toFloat();
    return true;
  }

  bool readIntParam(const String& line, char key, int& value) {
    float f = 0.0f;
    if (!readFloatParam(line, key, f)) return false;
    value = static_cast<int>(f);
    return true;
  }

  bool readObjectIdParam(const String& line, int& value) {
    const int idx = line.indexOf("ID");
    if (idx < 0) return false;

    int start = idx + 2;
    while (start < line.length() && (line[start] == ' ' || line[start] == '=' || line[start] == ':')) start++;

    int end = start;
    while (end < line.length()) {
      const char c = line[end];
      if ((c >= '0' && c <= '9') || c == '-' || c == '+') end++;
      else break;
    }

    if (end <= start) return false;
    value = line.substring(start, end).toInt();
    return true;
  }

  void printPickPlaceCsvHeader() {
    _io.println(F("DATA:PICK_PLACE_CSV_HEADER,trial_id,object_id,started_ms,ended_ms,duration_ms,status,reason,pick_x,pick_y,pick_z_req,bin_index,bin_name,bin_x,bin_y,bin_z,ina_ready,ina_detected,offset_ok,touch_z,touch_current_A"));
  }

  void emitPickPlaceCsv(uint32_t trialId,
                        int objectId,
                        uint32_t startedMs,
                        const char* status,
                        const char* reason,
                        const CartesianPose& pick,
                        int binIndex,
                        const BinPoseConfig& bin,
                        bool inaReady,
                        bool currentDetected,
                        bool offsetSatisfied,
                        float touchZ,
                        float touchCurrentA) {
    const uint32_t endedMs = millis();
    const uint32_t durationMs = endedMs - startedMs;

    _io.print(F("EVENT:PICK_PLACE_RESULT trial="));
    _io.print(trialId);
    _io.print(F(" id="));
    _io.print(objectId);
    _io.print(F(" status="));
    _io.print(status);
    _io.print(F(" reason="));
    _io.print(reason);
    _io.print(F(" ina_detected="));
    _io.print(currentDetected ? F("YES") : F("NO"));
    _io.print(F(" continued_to_bin="));
    _io.println(F("YES"));

    _io.print(F("DATA:PICK_PLACE_CSV,"));
    _io.print(trialId); _io.print(',');
    _io.print(objectId); _io.print(',');
    _io.print(startedMs); _io.print(',');
    _io.print(endedMs); _io.print(',');
    _io.print(durationMs); _io.print(',');
    _io.print(status); _io.print(',');
    _io.print(reason); _io.print(',');
    _io.print(pick.x, 2); _io.print(',');
    _io.print(pick.y, 2); _io.print(',');
    _io.print(pick.z, 2); _io.print(',');
    _io.print(binIndex); _io.print(',');
    _io.print(bin.name); _io.print(',');
    _io.print(bin.x, 2); _io.print(',');
    _io.print(bin.y, 2); _io.print(',');
    _io.print(bin.z, 2); _io.print(',');
    _io.print(inaReady ? 1 : 0); _io.print(',');
    _io.print(currentDetected ? 1 : 0); _io.print(',');
    _io.print(offsetSatisfied ? 1 : 0); _io.print(',');
    _io.print(touchZ, 2); _io.print(',');
    _io.println(touchCurrentA, 3);
  }

  bool requireHomedForMotion(const __FlashStringHelper* commandName) {
    if (_motion.isHomed()) return true;

    _io.print(F("ERROR:"));
    _io.print(commandName);
    _io.println(F(" rejected. Robot belum homing. Kirim G28 dulu."));
    _io.print(F("EVENT:"));
    _io.print(commandName);
    _io.println(F("_REJECTED_NOT_HOMED"));
    return false;
  }

  void handleCartesianMove(const String& line, bool linearMove) {
    if (!requireHomedForMotion(linearMove ? F("G1") : F("G0"))) return;

    const JointPose nowJ = _motion.currentJointPose();
    CartesianPose target = _kinematic.forward(nowJ);

    readFloatParam(line, 'X', target.x);
    readFloatParam(line, 'Y', target.y);
    readFloatParam(line, 'Z', target.z);

    if (linearMove) {
      _motion.moveToCartesianLinear(target, _kinematic, _io);
    } else {
      _motion.moveToCartesian(target, _kinematic, _io, true);
    }
  }

  void handleMoveJ(const String& line) {
    if (!requireHomedForMotion(F("MOVEJ"))) return;

    JointPose target = _motion.currentJointPose();

    readFloatParam(line, 'A', target.j1);
    readFloatParam(line, 'B', target.j2);
    readFloatParam(line, 'C', target.j3);

    _motion.moveToJointAngles(target, _io, true);
  }

  void handleSort(const String& line) {
    CartesianPose pick = {INITIAL_X_MM, INITIAL_Y_MM, DEFAULT_PLACE_Z_MM};
    int binIndex = 0;
    int objectId = -1;

    const bool hasX = readFloatParam(line, 'X', pick.x);
    const bool hasY = readFloatParam(line, 'Y', pick.y);
    readFloatParam(line, 'Z', pick.z);
    readIntParam(line, 'B', binIndex);
    readObjectIdParam(line, objectId);

    if (!hasX || !hasY) {
      _hasYoloCoord = false;
      _lastYoloWasAcceptedForMotion = false;
      _pump.off();
      _io.println(F("ERROR:SORT rejected. Koordinat YOLO tidak lengkap. Wajib ada X dan Y."));
      _io.println(F("EVENT:YOLO_COORD_MISSING"));
      _io.println(F("EVENT:PUMP_OFF_NO_YOLO_COORD"));
      return;
    }

    if (pick.z < PICK_PROBE_ZMIN_MM) {
      _io.print(F("WARNING: requested pick Z < limit, clamped to Z="));
      _io.println(PICK_PROBE_ZMIN_MM, 2);
      pick.z = PICK_PROBE_ZMIN_MM;
    }

    if (binIndex < 0 || binIndex >= WASTE_BIN_COUNT) {
      _io.println(F("ERROR: invalid bin index."));
      printBins();
      return;
    }

    recordYoloReceived(pick, binIndex, objectId);

    if (!_motion.isHomed()) {
      _lastYoloWasAcceptedForMotion = false;
      _pump.off();
      _io.println(F("ERROR:SORT rejected. Robot belum homing. Kirim G28 dulu."));
      _io.println(F("EVENT:SORT_REJECTED_NOT_HOMED"));
      _io.println(F("EVENT:PUMP_OFF_NOT_HOMED"));
      _io.println(F("ACTION_REQUIRED:G28"));
      return;
    }

    _lastYoloWasAcceptedForMotion = true;

    // Safety revision:
    // Pump tidak dinyalakan di sini. Pump baru ON setelah robot benar-benar
    // sampai di koordinat XY pick pada Z safety (lihat sortSequence()).
    //
    // REVISI (auto-recover untuk YOLO berikutnya): kalau pump masih ON dari
    // percobaan SORT sebelumnya yang gagal di tengah jalan (objek masih
    // dihisap tapi belum sempat sampai bin), JANGAN langsung _pump.off() di
    // posisi lengan saat itu juga -- itu berarti menjatuhkan objek di
    // sembarang tempat. Angkat dulu ke Z safety pada XY saat ini, baru lepas.
    // Setelah itu robot langsung lanjut proses SORT baru dari YOLO ini tanpa
    // perlu intervensi manual.
    if (_pump.isOn()) {
      if (!raiseCurrentXYToSafe()) {
        _io.println(F("EVENT:WARNING_RAISE_BEFORE_FORCED_OFF_FAILED"));
      }
      _pump.off();
      _io.println(F("EVENT:PUMP_FORCED_OFF_BEFORE_PICK_XY"));
    }
    _io.println(F("EVENT:SORT_MOTION_ACCEPTED_PUMP_WAIT_XY"));
    sortSequence(pick, WASTE_BINS[binIndex], binIndex, objectId);
    _io.println(F("EVENT:READY_FOR_NEXT_YOLO"));
  }

  float clampZMin(float z) const {
    return (z < PICK_PROBE_ZMIN_MM) ? PICK_PROBE_ZMIN_MM : z;
  }

  void printYoloReceivedEvent(const CartesianPose& pick, int binIndex, int objectId) {
    _io.print(F("EVENT:YOLO_COORD_RECEIVED X="));
    _io.print(pick.x, 2);
    _io.print(F(" Y="));
    _io.print(pick.y, 2);
    _io.print(F(" Z="));
    _io.print(pick.z, 2);
    _io.print(F(" B="));
    _io.print(binIndex);
    _io.print(F(" ID="));
    _io.println(objectId);
  }

  void recordYoloReceived(const CartesianPose& pick, int binIndex, int objectId) {
    _hasYoloCoord = true;
    _yoloReceiveCount++;
    _lastYoloReceivedMs = millis();
    _lastYoloPick = pick;
    _lastYoloBin = binIndex;
    _lastYoloObjectId = objectId;
    _lastYoloWasHomed = _motion.isHomed();
    _lastYoloWasAcceptedForMotion = false;

    printYoloReceivedEvent(pick, binIndex, objectId);
    _io.println(F("EVENT:YOLO_COORD_STORED"));
  }

  void printYoloStatus() {
    _io.println(F("YOLO STATUS"));
    _io.println(F("-----------"));
    if (!_hasYoloCoord) {
      _pump.off();
      _io.println(F("EVENT:YOLO_STATUS RECEIVED=NO"));
      _io.println(F("EVENT:PUMP_OFF_NO_YOLO_COORD"));
      _io.print(F("Homed="));
      _io.println(_motion.isHomed() ? F("YES") : F("NO"));
      _io.print(F("Pump="));
      _io.println(_pump.isOn() ? F("ON") : F("OFF"));
      return;
    }

    _io.print(F("EVENT:YOLO_STATUS RECEIVED=YES COUNT="));
    _io.print(_yoloReceiveCount);
    _io.print(F(" AGE_MS="));
    _io.print(millis() - _lastYoloReceivedMs);
    _io.print(F(" X="));
    _io.print(_lastYoloPick.x, 2);
    _io.print(F(" Y="));
    _io.print(_lastYoloPick.y, 2);
    _io.print(F(" Z="));
    _io.print(_lastYoloPick.z, 2);
    _io.print(F(" B="));
    _io.print(_lastYoloBin);
    _io.print(F(" ID="));
    _io.println(_lastYoloObjectId);

    _io.print(F("HomedAtReceive="));
    _io.println(_lastYoloWasHomed ? F("YES") : F("NO"));
    _io.print(F("HomedNow="));
    _io.println(_motion.isHomed() ? F("YES") : F("NO"));
    _io.print(F("MotionAccepted="));
    _io.println(_lastYoloWasAcceptedForMotion ? F("YES") : F("NO"));
    _io.print(F("Pump="));
    _io.println(_pump.isOn() ? F("ON") : F("OFF"));
  }

  bool rapidMove(const CartesianPose& target, const __FlashStringHelper* label) {
    CartesianPose safeTarget = target;
    safeTarget.z = clampZMin(safeTarget.z);

    _io.print(F("G0 "));
    _io.print(label);
    _io.print(F(" X=")); _io.print(safeTarget.x, 2);
    _io.print(F(" Y=")); _io.print(safeTarget.y, 2);
    _io.print(F(" Z=")); _io.println(safeTarget.z, 2);

    return _motion.moveToCartesian(safeTarget, _kinematic, _io, true, false);
  }

  bool linearMove(const CartesianPose& target, const __FlashStringHelper* label) {
    CartesianPose safeTarget = target;
    safeTarget.z = clampZMin(safeTarget.z);

    _io.print(F("G1 "));
    _io.print(label);
    _io.print(F(" X=")); _io.print(safeTarget.x, 2);
    _io.print(F(" Y=")); _io.print(safeTarget.y, 2);
    _io.print(F(" Z=")); _io.println(safeTarget.z, 2);

    return _motion.moveToCartesianLinear(safeTarget, _kinematic, _io);
  }

  bool raiseCurrentXYToSafe() {
    const CartesianPose now = _kinematic.forward(_motion.currentJointPose());
    const CartesianPose safeNow = {now.x, now.y, PICK_SAFE_Z_MM};
    return rapidMove(safeNow, F("SAFE_Z_CURRENT_XY"));
  }

  // REVISI (fix "Z pick down lambat"): sebelumnya fungsi ini memecah turun
  // jadi puluhan langkah G1 terpisah sejauh PICK_DESCEND_STEP_MM (2mm), dan
  // SETIAP langkah adalah gerakan blocking yang berhenti total sebelum baca
  // arus INA219 lalu jalan lagi dari diam. Untuk jarak 2mm, motor bahkan tidak
  // pernah sempat mencapai MAX_SPEED yang sudah dituning (profil geraknya
  // segitiga, bukan trapesium) -- jadi menaikkan MAX_SPEED G1 tidak berefek
  // di fase ini, dan overhead start-stop x ~70 langkah itulah yang membuat
  // "menuju XY (G0, cepat)" terasa jauh lebih gesit dibanding "Z pick down".
  //
  // Sekarang turun dalam SATU gerakan kontinu ke Zmin lewat
  // MotionController::descendWithCurrentMonitor(), arus dipantau tiap
  // PICK_MONITOR_SAMPLE_MS tanpa menghentikan motor. Logika deteksi
  // (arus >= threshold && sudah turun offset tambahan) tetap sama persis.
  // REVISI: untuk kebutuhan pengujian, gagal deteksi INA TIDAK lagi
  // membatalkan siklus. Robot tetap melanjutkan place ke bin, lalu firmware
  // mengirim indikator kegagalan ke VSCode lewat EVENT:PICK_PLACE_RESULT dan
  // DATA:PICK_PLACE_CSV. Return false hanya dipakai untuk kegagalan motion.
  bool probePickDown(const CartesianPose& pick,
                     float& finalZ,
                     float& finalCurrentA,
                     bool& inaReady,
                     bool& currentDetected,
                     bool& offsetSatisfied) {
    currentDetected = false;
    offsetSatisfied = false;
    inaReady = _ina.isReady();

    const bool moved = _motion.descendWithCurrentMonitor(
      pick,
      PICK_PROBE_ZMIN_MM,
      INA_OVERLOAD_THRESHOLD_A,
      PICK_AFTER_TOUCH_OFFSET_MM,
      PICK_MONITOR_SAMPLE_MS,
      PICK_DESCEND_USE_G1_SYNC,
      _kinematic,
      _ina,
      _io,
      finalZ,
      finalCurrentA,
      currentDetected,
      offsetSatisfied
    );

    if (!moved) {
      _io.println(F("EVENT:SORT_ERROR PICK_PROBE_MOTION_FAILED"));
      return false;
    }

    if (!inaReady) {
      _io.println(F("EVENT:PICK_FAIL_INA_NOT_READY_CONTINUE_TO_BIN"));
      _io.println(F("WARNING: INA219 tidak ready; siklus tetap lanjut ke bin untuk data uji."));
    } else if (!currentDetected) {
      _io.println(F("EVENT:PICK_FAIL_NO_INA_CONTINUE_TO_BIN"));
      _io.println(F("WARNING: INA threshold tidak terdeteksi; siklus tetap lanjut ke bin untuk data uji."));
    } else if (!offsetSatisfied) {
      _io.println(F("EVENT:PICK_OFFSET_NOT_FULL_BUT_CURRENT_OK (object pendek / dekat zMin, tetap lanjut)"));
    }

    return true;
  }

  bool sortSequence(const CartesianPose& pick, const BinPoseConfig& bin, int binIndex, int objectId) {
    _io.print(F("EVENT:SORT_START X=")); _io.print(pick.x, 2);
    _io.print(F(" Y=")); _io.print(pick.y, 2);
    _io.print(F(" Zreq=")); _io.print(pick.z, 2);
    _io.print(F(" bin=")); _io.print(bin.name);
    _io.print(F(" ID=")); _io.println(objectId);

    const uint32_t trialId = ++_pickPlaceTrialCount;
    const uint32_t trialStartedMs = millis();
    bool inaReady = _ina.isReady();
    bool currentDetected = false;
    bool offsetSatisfied = false;

    const CartesianPose pickSafe = {pick.x, pick.y, PICK_SAFE_Z_MM};

    // 1) Semua perpindahan jauh/XY memakai G0 pada Z safety.
    if (!raiseCurrentXYToSafe()) {
      _io.println(F("EVENT:SORT_ERROR RAISE_TO_SAFE_FAILED"));
      _pump.off();
      emitPickPlaceCsv(trialId, objectId, trialStartedMs, "ERROR", "RAISE_TO_SAFE_FAILED", pick, binIndex, bin, inaReady, currentDetected, offsetSatisfied, pick.z, 0.0f);
      return false;
    }

    if (!rapidMove(pickSafe, F("PICK_XY_SAFE"))) {
      _io.println(F("EVENT:SORT_ERROR MOVE_TO_PICK_XY_FAILED"));
      _pump.off();
      emitPickPlaceCsv(trialId, objectId, trialStartedMs, "ERROR", "MOVE_TO_PICK_XY_FAILED", pick, binIndex, bin, inaReady, currentDetected, offsetSatisfied, pick.z, 0.0f);
      return false;
    }

    // 2) Pump ON hanya setelah robot sudah mencapai koordinat XY pick
    //    pada Z safety. Setelah itu Z turun/probing sampai surface belt conveyor
    //    memakai batas arus INA219.
    _pump.on();
    _io.println(F("EVENT:PUMP_ON_AT_PICK_XY"));
    delay(PUMP_SETTLE_MS);

    float touchZ = PICK_SAFE_Z_MM;
    float touchCurrentA = 0.0f;
    if (!probePickDown(pick, touchZ, touchCurrentA, inaReady, currentDetected, offsetSatisfied)) {
      _pump.off();
      _io.println(F("EVENT:PUMP_OFF_PICK_PROBE_MOTION_FAILED"));
      if (!rapidMove(pickSafe, F("LIFT_PROBE_ERROR_SAFE"))) {
        _io.println(F("EVENT:SORT_ERROR LIFT_PROBE_ERROR_FAILED"));
      }
      emitPickPlaceCsv(trialId, objectId, trialStartedMs, "ERROR", "PICK_PROBE_MOTION_FAILED", pick, binIndex, bin, inaReady, currentDetected, offsetSatisfied, touchZ, touchCurrentA);
      return false;
    }

    if (!currentDetected) {
      _io.println(F("EVENT:PICK_TEST_CONTINUE_TO_BIN_WITHOUT_INA"));
    }

    delay(PUMP_SETTLE_MS);

    // 3) Angkat lagi ke Z safety dengan G0 di XY yang sama.
    // REVISI: pump TIDAK dimatikan di sini walau gerakan gagal. Objek sudah
    // dalam kondisi dihisap (probePickDown sukses) -- mematikan pump di titik
    // ini berarti menjatuhkan objek di tengah jalan sebelum sampai bin.
    // Sesuai requirement: pump hanya boleh mati ketika benar-benar sudah
    // sampai bin; kegagalan gerak lain diabaikan (di-log saja) supaya pump
    // tetap ON dan siap dicoba lagi / dilanjutkan secara manual.
    if (!rapidMove(pickSafe, F("LIFT_PICK_SAFE"))) {
      _io.println(F("EVENT:SORT_ERROR LIFT_PICK_FAILED_PUMP_KEPT_ON"));
      raiseCurrentXYToSafe();
      emitPickPlaceCsv(trialId, objectId, trialStartedMs, "ERROR", "LIFT_PICK_FAILED", pick, binIndex, bin, inaReady, currentDetected, offsetSatisfied, touchZ, touchCurrentA);
      return false;
    }

    // 4) Pindah ke koordinat penampungan sesuai kelas dengan G0 pada Z safety.
    //    Jika Z bin lebih rendah dari Z safety (khusus KACA: Z40), robot turun dulu.
    //    Untuk kelas lain, Z bin = PICK_SAFE_Z_MM sehingga langsung PUMP OFF di Z safety.
    const float binDropZ = clampZMin(bin.z);
    const CartesianPose binSafe = {bin.x, bin.y, PICK_SAFE_Z_MM};
    const CartesianPose binDrop = {bin.x, bin.y, binDropZ};

    // REVISI: sama seperti LIFT_PICK_FAILED di atas -- objek masih dihisap,
    // jadi pump TIDAK dimatikan walau gerakan menuju bin gagal. Pump baru
    // mati di EVENT:PUMP_OFF setelah benar-benar sampai bin (di bawah).
    if (!rapidMove(binSafe, F("BIN_XY_SAFE"))) {
      _io.println(F("EVENT:SORT_ERROR MOVE_TO_BIN_XY_FAILED_PUMP_KEPT_ON"));
      raiseCurrentXYToSafe();
      emitPickPlaceCsv(trialId, objectId, trialStartedMs, "ERROR", "MOVE_TO_BIN_XY_FAILED", pick, binIndex, bin, inaReady, currentDetected, offsetSatisfied, touchZ, touchCurrentA);
      return false;
    }

    if (binDropZ < (PICK_SAFE_Z_MM - 0.01f)) {
      if (!linearMove(binDrop, F("PLACE_DOWN"))) {
        _io.println(F("EVENT:SORT_ERROR PLACE_DOWN_FAILED_PUMP_KEPT_ON"));
        raiseCurrentXYToSafe();
        emitPickPlaceCsv(trialId, objectId, trialStartedMs, "ERROR", "PLACE_DOWN_FAILED", pick, binIndex, bin, inaReady, currentDetected, offsetSatisfied, touchZ, touchCurrentA);
        return false;
      }
    }

    _pump.off();
    _io.println(F("EVENT:PUMP_OFF"));
    delay(DROP_SETTLE_MS);

    rapidMove(binSafe, F("LIFT_AFTER_PLACE"));

    const char* status = currentDetected ? "SUCCESS" : "FAIL";
    const char* reason = currentDetected ? "OK" : (inaReady ? "INA_NOT_DETECTED_CONTINUED_TO_BIN" : "INA_NOT_READY_CONTINUED_TO_BIN");

    _io.print(F("EVENT:SORT_DONE touch_z="));
    _io.print(touchZ, 2);
    _io.print(F(" touch_current_A="));
    _io.print(touchCurrentA, 3);
    _io.print(F(" pick_result="));
    _io.print(status);
    _io.print(F(" reason="));
    _io.println(reason);

    emitPickPlaceCsv(trialId, objectId, trialStartedMs, status, reason, pick, binIndex, bin, inaReady, currentDetected, offsetSatisfied, touchZ, touchCurrentA);
    return true;
  }


  void printCalibrationParams() {
    _io.println(F("XYZ CALIBRATION PARAMETERS"));
    _io.println(F("--------------------------"));

    _io.print(F("Firmware=")); _io.println(FIRMWARE_NAME);
    _io.print(F("XYZ_CALIBRATION_MODE=")); _io.println(XYZ_CALIBRATION_MODE ? F("TRUE") : F("FALSE"));
    _io.print(F("Z_MIN_MM=")); _io.println(Z_MIN_MM, 2);
    _io.print(F("Z_MAX_MM=")); _io.println(Z_MAX_MM, 2);
    _io.print(F("NORMAL_OPERATION_Z_MIN_MM=")); _io.println(NORMAL_OPERATION_Z_MIN_MM, 2);
    _io.print(F("XYZ_CALIBRATION_Z_MIN_MM=")); _io.println(XYZ_CALIBRATION_Z_MIN_MM, 2);
    _io.print(F("PICK_PROBE_ZMIN_MM=")); _io.println(PICK_PROBE_ZMIN_MM, 2);

    _io.println(F("\nKoordinat dan offset Cartesian:"));
    _io.print(F("SCARA_OFFSET_X_MM=")); _io.println(SCARA_OFFSET_X_MM, 3);
    _io.print(F("SCARA_OFFSET_Y_MM=")); _io.println(SCARA_OFFSET_Y_MM, 3);
    _io.print(F("SCARA_OFFSET_Z_MM=")); _io.println(SCARA_OFFSET_Z_MM, 3);
    _io.print(F("AXIS_SCALE_X=")); _io.println(AXIS_SCALE_X, 6);
    _io.print(F("AXIS_SCALE_Y=")); _io.println(AXIS_SCALE_Y, 6);
    _io.print(F("AXIS_SCALE_Z=")); _io.println(AXIS_SCALE_Z, 6);

    _io.println(F("\nGeometri IK/FK:"));
    _io.print(F("LINKAGE1_MM=")); _io.println(LINKAGE1_MM, 3);
    _io.print(F("LINKAGE2_MM=")); _io.println(LINKAGE2_MM, 3);
    _io.print(F("HEAD_OFFSET_MM=")); _io.println(HEAD_OFFSET_MM, 3);
    _io.print(F("CENTER_OFFSET_MM=")); _io.println(CENTER_OFFSET_MM, 3);
    _io.print(F("R_MIN_MM=")); _io.println(R_MIN_MM, 3);
    _io.print(F("R_MAX_MM=")); _io.println(R_MAX_MM, 3);
    _io.print(F("IK_ELBOW_SIGN=")); _io.println(IK_ELBOW_SIGN);

    _io.println(F("\nHome dan zero joint:"));
    _io.print(F("J1_HOME_ANGLE_DEG=")); _io.println(J1_HOME_ANGLE_DEG, 3);
    _io.print(F("J2_HOME_ANGLE_DEG=")); _io.println(J2_HOME_ANGLE_DEG, 3);
    _io.print(F("J3_HOME_ANGLE_DEG=")); _io.println(J3_HOME_ANGLE_DEG, 3);
    _io.print(F("J1_HOME_OFFSET_DEG=")); _io.println(J1_HOME_OFFSET_DEG, 3);
    _io.print(F("J2_HOME_OFFSET_DEG=")); _io.println(J2_HOME_OFFSET_DEG, 3);
    _io.print(F("J3_HOME_OFFSET_DEG=")); _io.println(J3_HOME_OFFSET_DEG, 3);

    _io.println(F("\nStepper dan arah motor:"));
    _io.print(F("MOTOR_STEPS_PER_REV=")); _io.println(MOTOR_STEPS_PER_REV, 3);
    _io.print(F("MICROSTEPS=")); _io.println(MICROSTEPS, 3);
    _io.print(F("J1_GEAR_RATIO=")); _io.println(J1_GEAR_RATIO, 4);
    _io.print(F("J2_GEAR_RATIO=")); _io.println(J2_GEAR_RATIO, 4);
    _io.print(F("J3_GEAR_RATIO=")); _io.println(J3_GEAR_RATIO, 4);
    _io.print(F("J1_STEPS_PER_DEG=")); _io.println(J1_STEPS_PER_DEG, 6);
    _io.print(F("J2_STEPS_PER_DEG=")); _io.println(J2_STEPS_PER_DEG, 6);
    _io.print(F("J3_STEPS_PER_DEG=")); _io.println(J3_STEPS_PER_DEG, 6);
    _io.print(F("J1_DIR_INVERT=")); _io.println(J1_DIR_INVERT ? F("TRUE") : F("FALSE"));
    _io.print(F("J2_DIR_INVERT=")); _io.println(J2_DIR_INVERT ? F("TRUE") : F("FALSE"));
    _io.print(F("J3_DIR_INVERT=")); _io.println(J3_DIR_INVERT ? F("TRUE") : F("FALSE"));

    _io.println(F("\nGerak G0/G1:"));
    _io.print(F("DEFAULT_FEEDRATE_MM_MIN=")); _io.println(DEFAULT_FEEDRATE_MM_MIN, 2);
    _io.print(F("DEG_PER_CARTESIAN_SEGMENT=")); _io.println(DEG_PER_CARTESIAN_SEGMENT, 3);
    _io.print(F("ENABLE_MOTION_BLENDING=")); _io.println(ENABLE_MOTION_BLENDING ? F("TRUE") : F("FALSE"));
    _io.print(F("MOTION_BLEND_START_FRACTION=")); _io.println(MOTION_BLEND_START_FRACTION, 3);
    _io.print(F("CARTESIAN_MOTION_TIMEOUT_MS=")); _io.println(CARTESIAN_MOTION_TIMEOUT_MS);

    _io.println(F("\nPick/INA:"));
    _io.print(F("PICK_SAFE_Z_MM=")); _io.println(PICK_SAFE_Z_MM, 2);
    _io.print(F("PICK_AFTER_TOUCH_OFFSET_MM=")); _io.println(PICK_AFTER_TOUCH_OFFSET_MM, 2);
    _io.print(F("INA_OVERLOAD_THRESHOLD_A=")); _io.println(INA_OVERLOAD_THRESHOLD_A, 4);
    _io.print(F("PICK_TOUCH_BLIND_TIME_MS=")); _io.println(PICK_TOUCH_BLIND_TIME_MS);
    _io.print(F("PICK_TOUCH_DEBOUNCE_SAMPLES=")); _io.println(PICK_TOUCH_DEBOUNCE_SAMPLES);
    _io.print(F("CAL_OFFSET=")); _io.println(CAL_OFFSET, 6);
    _io.print(F("CAL_SLOPE=")); _io.println(CAL_SLOPE, 6);
  }

  void printBins() {
    _io.println(F("Configured bins:"));
    for (int i = 0; i < WASTE_BIN_COUNT; i++) {
      _io.print(F("B")); _io.print(i);
      _io.print(F(" ")); _io.print(WASTE_BINS[i].name);
      _io.print(F(" X=")); _io.print(WASTE_BINS[i].x, 2);
      _io.print(F(" Y=")); _io.print(WASTE_BINS[i].y, 2);
      _io.print(F(" Z=")); _io.println(WASTE_BINS[i].z, 2);
    }
  }
};
