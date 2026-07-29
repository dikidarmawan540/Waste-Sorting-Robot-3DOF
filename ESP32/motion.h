#pragma once

#include <Arduino.h>
#include <AccelStepper.h>
#include "config.h"
#include "kineamtic.h"
#include "INA219.h"

class MotionController {
public:
  MotionController()
    : _j1(AccelStepper::DRIVER, J1_STEP_PIN, J1_DIR_PIN),
      _j2(AccelStepper::DRIVER, J2_STEP_PIN, J2_DIR_PIN),
      _j3(AccelStepper::DRIVER, J3_STEP_PIN, J3_DIR_PIN),
      _homed(false) {}

  void begin() {
    pinMode(J1_STEP_PIN, OUTPUT);
    pinMode(J1_DIR_PIN, OUTPUT);
    pinMode(J2_STEP_PIN, OUTPUT);
    pinMode(J2_DIR_PIN, OUTPUT);
    pinMode(J3_STEP_PIN, OUTPUT);
    pinMode(J3_DIR_PIN, OUTPUT);

    digitalWrite(J1_STEP_PIN, STEP_ACTIVE_HIGH ? LOW : HIGH);
    digitalWrite(J2_STEP_PIN, STEP_ACTIVE_HIGH ? LOW : HIGH);
    digitalWrite(J3_STEP_PIN, STEP_ACTIVE_HIGH ? LOW : HIGH);

    configureStepper(_j1, J1_EN_PIN, J1_MAX_SPEED_STEPS_PER_SEC, J1_ACCEL_STEPS_PER_SEC2);
    configureStepper(_j2, J2_EN_PIN, J2_MAX_SPEED_STEPS_PER_SEC, J2_ACCEL_STEPS_PER_SEC2);
    configureStepper(_j3, J3_EN_PIN, J3_MAX_SPEED_STEPS_PER_SEC, J3_ACCEL_STEPS_PER_SEC2);

    configureLimitPin(J1_LIMIT_PIN);
    configureLimitPin(J2_LIMIT_PIN);
    configureLimitPin(J3_LIMIT_PIN);

    // Estimasi internal saat baru boot. Status tetap belum homing.
    _j1.setCurrentPosition(degToSteps(1, BOOT_J1_DEG));
    _j2.setCurrentPosition(degToSteps(2, BOOT_J2_DEG));
    _j3.setCurrentPosition(degToSteps(3, BOOT_J3_DEG));
  }

  void run() {
    _j1.run();
    _j2.run();
    _j3.run();
  }

  bool isBusy() {
    return (_j1.distanceToGo() != 0) || (_j2.distanceToGo() != 0) || (_j3.distanceToGo() != 0);
  }

  bool runToIdle(uint32_t timeoutMs = 60000UL) {
    const uint32_t startMs = millis();
    while (isBusy()) {
      run();
      yield();
      if ((millis() - startMs) > timeoutMs) {
        stopAll();
        return false;
      }
    }
    return true;
  }

  void stopAll() {
    _j1.stop();
    _j2.stop();
    _j3.stop();
  }

  bool isHomed() const {
    return _homed;
  }

  bool limitActive(int pin) const {
    const int raw = digitalRead(pin);
    return LIMIT_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
  }

  void printLimitStatus(Stream& out) const {
    const int j1Raw = digitalRead(J1_LIMIT_PIN);
    const int j2Raw = digitalRead(J2_LIMIT_PIN);
    const int j3Raw = digitalRead(J3_LIMIT_PIN);

    out.print(F("J1 raw=")); out.print(j1Raw);
    out.print(F(" status=")); out.print(limitActive(J1_LIMIT_PIN) ? F("TRIGGERED") : F("open"));

    out.print(F(" | J2 raw=")); out.print(j2Raw);
    out.print(F(" status=")); out.print(limitActive(J2_LIMIT_PIN) ? F("TRIGGERED") : F("open"));

    out.print(F(" | J3 raw=")); out.print(j3Raw);
    out.print(F(" status=")); out.println(limitActive(J3_LIMIT_PIN) ? F("TRIGGERED") : F("open"));
  }

  bool homeAll(Stream& out) {
    out.println(F("G28: raw homing J3 elbow..."));
    if (!homeAxisRaw(3, _j3, J3_STEP_PIN, J3_DIR_PIN, J3_LIMIT_PIN, J3_HOME_TO_LIMIT_DIR_LEVEL, J3_HOME_OFFSET_DEG, out)) return false;

    out.println(F("G28: raw homing J2 shoulder..."));
    if (!homeAxisRaw(2, _j2, J2_STEP_PIN, J2_DIR_PIN, J2_LIMIT_PIN, J2_HOME_TO_LIMIT_DIR_LEVEL, J2_HOME_OFFSET_DEG, out)) return false;

    out.println(F("G28: raw homing J1 base..."));
    if (!homeAxisRaw(1, _j1, J1_STEP_PIN, J1_DIR_PIN, J1_LIMIT_PIN, J1_HOME_TO_LIMIT_DIR_LEVEL, J1_HOME_OFFSET_DEG, out)) return false;

    _homed = true;
    out.println(F("G28 done. Raw offset applied and kinematic home angle set."));
    return true;
  }

  bool moveToJointAngles(const JointPose& target, Stream& out, bool blocking = true) {
    if (!targetInRange(target, out)) return false;

    // G0/MOVEJ selalu rapid: pastikan speed/accel full config, bukan sisa
    // hasil skala sinkronisasi dari G1 linear sebelumnya.
    restoreAllNormalSpeeds();

    _j1.moveTo(degToSteps(1, target.j1));
    _j2.moveTo(degToSteps(2, target.j2));
    _j3.moveTo(degToSteps(3, target.j3));

    if (blocking) {
      if (!runToIdle()) {
        out.println(F("ERROR: motion timeout"));
        return false;
      }
    }
    return true;
  }

  bool moveToCartesian(const CartesianPose& target, const RobotKinematic& kinematic, Stream& out, bool blocking = true, bool verbose = true) {
    KinematicResult ik = kinematic.inverse(target);
    if (!ik.ok) {
      out.print(F("IK ERROR: "));
      out.println(ik.message);
      return false;
    }

    if (verbose) {
      out.print(F("IK J1=")); out.print(ik.joints.j1, 2);
      out.print(F(" J2=")); out.print(ik.joints.j2, 2);
      out.print(F(" J3=")); out.println(ik.joints.j3, 2);
    }

    return moveToJointAngles(ik.joints, out, blocking);
  }

  bool moveToCartesianLinear(const CartesianPose& target, const RobotKinematic& kinematic, Stream& out) {
    const JointPose startJ = currentJointPose();
    const CartesianPose start = kinematic.forward(startJ);

    const float dx = target.x - start.x;
    const float dy = target.y - start.y;
    const float dz = target.z - start.z;
    const float distance = sqrtf((dx * dx) + (dy * dy) + (dz * dz));

    // Segmentasi berbasis SUDUT sendi yang berubah, bukan jarak Cartesian.
    // Deviasi chord-vs-arc (seberapa jauh X/Y/Z "meleset" di antara waypoint)
    // ditentukan oleh besar sudut per segmen, bukan oleh jarak mm. Contoh: G1
    // Z murni bisa berjarak >100mm padahal sudut J2/J3 yang berubah kecil
    // -> kalau segmentasi dipaksa ikut jarak mm, kita bikin puluhan segmen
    // super pendek (cuma belasan step) yang bikin motor terus-menerus
    // rem/gas tiap segmen -> lambat & kurang mulus. Dengan basis sudut,
    // jumlah segmen otomatis menyesuaikan seberapa "melengkung" gerakan itu
    // sebenarnya perlu didekati.
    KinematicResult ikEnd = kinematic.inverse(target);
    if (!ikEnd.ok) {
      out.print(F("IK ERROR: "));
      out.println(ikEnd.message);
      return false;
    }

    const float dJ1 = fabsf(ikEnd.joints.j1 - startJ.j1);
    const float dJ2 = fabsf(ikEnd.joints.j2 - startJ.j2);
    const float dJ3 = fabsf(ikEnd.joints.j3 - startJ.j3);
    float maxJointDeltaDeg = dJ1;
    if (dJ2 > maxJointDeltaDeg) maxJointDeltaDeg = dJ2;
    if (dJ3 > maxJointDeltaDeg) maxJointDeltaDeg = dJ3;

    const int segmentsByAngle = static_cast<int>(ceilf(maxJointDeltaDeg / DEG_PER_CARTESIAN_SEGMENT));
    const int segmentsByDistance = static_cast<int>(ceilf(distance * SEGMENTS_PER_MM));
    // Pakai basis sudut sebagai penggerak utama, jarak cuma jadi lantai
    // ringan (jaga-jaga untuk gerakan lurus tapi sudutnya kebetulan kecil).
    int segments = (segmentsByAngle > segmentsByDistance) ? segmentsByAngle : segmentsByDistance;
    if (segments < MIN_CARTESIAN_SEGMENTS) segments = MIN_CARTESIAN_SEGMENTS;
    if (segments > MAX_CARTESIAN_SEGMENTS) segments = MAX_CARTESIAN_SEGMENTS;

    out.print(F("G1 linear Cartesian BLEND: segments="));
    out.print(segments);
    out.print(F(" (angle-based, maxJointDelta="));
    out.print(maxJointDeltaDeg, 2);
    out.print(F("deg)"));
    out.print(F(" blending="));
    out.print(ENABLE_MOTION_BLENDING ? F("ON") : F("OFF"));
    out.print(F(" target X=")); out.print(target.x, 2);
    out.print(F(" Y=")); out.print(target.y, 2);
    out.print(F(" Z=")); out.println(target.z, 2);

    JointStepTarget stepTargets[MAX_CARTESIAN_SEGMENTS];

    for (int i = 1; i <= segments; i++) {
      // REVISI: segmen terakhir (i == segments) memakai koordinat target
      // ASLI (bukan start + dz*t) supaya tidak ada drift floating-point.
      // Interpolasi start+dz*t seharusnya sama dengan target di t=1, tapi
      // akumulasi pembulatan float bisa membuatnya meleset ~0.0001mm --
      // cukup untuk gagal validasi kalau target persis di batas Z_MIN/MAX.
      CartesianPose stepTarget;
      if (i == segments) {
        stepTarget = target;
      } else {
        const float t = static_cast<float>(i) / static_cast<float>(segments);
        stepTarget = {
          start.x + (dx * t),
          start.y + (dy * t),
          start.z + (dz * t)
        };
      }

      KinematicResult ik = kinematic.inverse(stepTarget);
      if (!ik.ok) {
        out.print(F("IK ERROR at segment "));
        out.print(i);
        out.print(F("/"));
        out.print(segments);
        out.print(F(": "));
        out.println(ik.message);
        return false;
      }

      if (!targetInRange(ik.joints, out)) {
        out.print(F("ERROR: linear move joint limit at segment "));
        out.print(i);
        out.print(F("/"));
        out.println(segments);
        return false;
      }

      stepTargets[i - 1] = jointPoseToSteps(ik.joints);

      if (i == segments) {
        out.print(F("Final IK J1=")); out.print(ik.joints.j1, 2);
        out.print(F(" J2=")); out.print(ik.joints.j2, 2);
        out.print(F(" J3=")); out.println(ik.joints.j3, 2);
      }
    }

    if (ENABLE_MOTION_BLENDING) {
      return runStepTargetsBlended(stepTargets, segments, out);
    }

    for (int i = 0; i < segments; i++) {
      commandStepTarget(stepTargets[i]);
      if (!runToIdle(CARTESIAN_MOTION_TIMEOUT_MS)) {
        out.println(F("ERROR: linear motion timeout"));
        return false;
      }
    }

    return true;
  }

  // REVISI: pengganti loop "turun 2mm -> stop total -> baca INA -> ulangi"
  // di command.h::probePickDown(). Di sini robot turun dalam SATU gerakan
  // linear kontinu menuju zMin (X/Y tetap), dan arus INA219 dipantau tiap
  // sampleIntervalMs TANPA memberhentikan motor (tidak ada delay() di sini,
  // hanya cek millis()). Begitu arus >= overloadThresholdA dan sudah turun
  // afterTouchOffsetMm tambahan, motor direm halus (AccelStepper::stop(),
  // masih pakai profil accel J2/J3 yang sudah di-tuning) baru berhenti.
  //
  // Ini menghilangkan puluhan siklus accel/decel penuh yang sebelumnya
  // membuat MAX_SPEED G1 tidak pernah tercapai (gerak tiap step terlalu
  // pendek untuk keluar dari fase akselerasi) -- itulah sumber "Z pick down
  // lambat" meski G1 sudah dituning.
  // REVISI-4: sebelumnya fungsi ini pakai SATU commandStepTarget() untuk
  // seluruh jarak turun (safe Z -> zMin sekaligus). Itu membuat motor
  // akselerasi sekali lalu cruise di MAX_SPEED penuh untuk seluruh ~145mm
  // TANPA jeda -- beda dengan perintah manual "G1 Z.." yang lewat
  // moveToCartesianLinear(): disegmentasi tiap DEG_PER_CARTESIAN_SEGMENT
  // dengan blending antar-segmen. Sama-sama pakai config G1 yang sama, tapi
  // arsitektur eksekusinya beda -> kerasa lebih cepat/kurang terkendali.
  //
  // Sekarang descend memakai SEGMENTASI + BLENDING YANG SAMA PERSIS dengan
  // moveToCartesianLinear (baca komentar di fungsi itu untuk detail kenapa
  // basis sudut dipakai), supaya "Z pick down" = "G1 manual" secara gerak.
  // Arus INA219 dipantau tiap sampleIntervalMs di sela-sela loop blending.
  // Tidak ada rising-edge, blind-zone, atau debounce: 1 sampel arus
  // terkalibrasi regresi linear >= threshold langsung dianggap deteksi.
  bool descendWithCurrentMonitor(const CartesianPose& pick,
                                  float zMin,
                                  float overloadThresholdA,
                                  float afterTouchOffsetMm,
                                  float sampleIntervalMs,
                                  bool useG1Sync,
                                  const RobotKinematic& kinematic,
                                  INA219Monitor& ina,
                                  Stream& out,
                                  float& outFinalZ,
                                  float& outFinalCurrentA,
                                  bool& outCurrentDetected,
                                  bool& outOffsetSatisfied) {
    outCurrentDetected = false;
    outOffsetSatisfied = false;
    outFinalCurrentA = 0.0f;
    outFinalZ = pick.z;

    CartesianPose target = pick;
    target.z = zMin;

    const JointPose startJ = currentJointPose();
    const CartesianPose start = kinematic.forward(startJ);

    KinematicResult ikEnd = kinematic.inverse(target);
    if (!ikEnd.ok) {
      out.print(F("IK ERROR: "));
      out.println(ikEnd.message);
      return false;
    }
    if (!targetInRange(ikEnd.joints, out)) {
      out.println(F("ERROR: descend monitor target joint limit"));
      return false;
    }

    if (!useG1Sync) {
      // Gaya G0: satu gerakan langsung, tiap sendi independen di
      // MAX_SPEED/ACCEL config-nya masing-masing (sama seperti G0/MOVEJ).
      restoreAllNormalSpeeds();
      _j1.moveTo(degToSteps(1, ikEnd.joints.j1));
      _j2.moveTo(degToSteps(2, ikEnd.joints.j2));
      _j3.moveTo(degToSteps(3, ikEnd.joints.j3));

      out.println(F("EVENT:PICK_PROBE_START(monitor,G0-independent)"));
      return runDescendMonitorLoop(zMin, overloadThresholdA, afterTouchOffsetMm, sampleIntervalMs,
                                    kinematic, ina, out,
                                    outFinalZ, outFinalCurrentA, outCurrentDetected, outOffsetSatisfied);
    }

    // Gaya G1: segmentasi berbasis sudut, identik dengan moveToCartesianLinear().
    const float dx = target.x - start.x;
    const float dy = target.y - start.y;
    const float dz = target.z - start.z;
    const float distance = sqrtf((dx * dx) + (dy * dy) + (dz * dz));

    const float dJ1 = fabsf(ikEnd.joints.j1 - startJ.j1);
    const float dJ2 = fabsf(ikEnd.joints.j2 - startJ.j2);
    const float dJ3 = fabsf(ikEnd.joints.j3 - startJ.j3);
    float maxJointDeltaDeg = dJ1;
    if (dJ2 > maxJointDeltaDeg) maxJointDeltaDeg = dJ2;
    if (dJ3 > maxJointDeltaDeg) maxJointDeltaDeg = dJ3;

    const int segmentsByAngle = static_cast<int>(ceilf(maxJointDeltaDeg / DEG_PER_CARTESIAN_SEGMENT));
    const int segmentsByDistance = static_cast<int>(ceilf(distance * SEGMENTS_PER_MM));
    int segments = (segmentsByAngle > segmentsByDistance) ? segmentsByAngle : segmentsByDistance;
    if (segments < MIN_CARTESIAN_SEGMENTS) segments = MIN_CARTESIAN_SEGMENTS;
    if (segments > MAX_CARTESIAN_SEGMENTS) segments = MAX_CARTESIAN_SEGMENTS;

    out.print(F("EVENT:PICK_PROBE_START(monitor,G1-synced) segments="));
    out.println(segments);

    JointStepTarget stepTargets[MAX_CARTESIAN_SEGMENTS];
    for (int i = 1; i <= segments; i++) {
      // REVISI: sama seperti moveToCartesianLinear() -- segmen terakhir
      // (biasanya persis di zMin/Z_MIN_MM) memakai koordinat target ASLI
      // supaya tidak gagal validasi hanya karena drift floating-point dari
      // akumulasi start + dz*t. Ini akar penyebab "IK ERROR at segment
      // N/N: Z out of range" saat probe turun ke batas Z persis.
      CartesianPose stepTarget;
      if (i == segments) {
        stepTarget = target;
      } else {
        const float t = static_cast<float>(i) / static_cast<float>(segments);
        stepTarget = {
          start.x + (dx * t),
          start.y + (dy * t),
          start.z + (dz * t)
        };
      }

      KinematicResult ik = kinematic.inverse(stepTarget);
      if (!ik.ok) {
        out.print(F("IK ERROR at segment "));
        out.print(i);
        out.print(F("/"));
        out.print(segments);
        out.print(F(": "));
        out.println(ik.message);
        return false;
      }
      if (!targetInRange(ik.joints, out)) {
        out.print(F("ERROR: descend joint limit at segment "));
        out.print(i);
        out.print(F("/"));
        out.println(segments);
        return false;
      }

      stepTargets[i - 1] = jointPoseToSteps(ik.joints);
    }

    return runDescendBlendedWithMonitor(stepTargets, segments, zMin,
                                         overloadThresholdA, afterTouchOffsetMm, sampleIntervalMs,
                                         kinematic, ina, out,
                                         outFinalZ, outFinalCurrentA, outCurrentDetected, outOffsetSatisfied);
  }

  JointPose currentJointPose() {
    return {
      stepsToDeg(1, _j1.currentPosition()),
      stepsToDeg(2, _j2.currentPosition()),
      stepsToDeg(3, _j3.currentPosition())
    };
  }

  void printPosition(Stream& out, const RobotKinematic& kinematic) {
    const JointPose joints = currentJointPose();
    const CartesianPose xyz = kinematic.forward(joints);

    out.print(F("J1=")); out.print(joints.j1, 3);
    out.print(F(" J2=")); out.print(joints.j2, 3);
    out.print(F(" J3=")); out.print(joints.j3, 3);
    out.print(F(" | X=")); out.print(xyz.x, 2);
    out.print(F(" Y=")); out.print(xyz.y, 2);
    out.print(F(" Z=")); out.print(xyz.z, 2);
    out.print(F(" | homed=")); out.println(_homed ? F("YES") : F("NO"));
  }

private:
  AccelStepper _j1;
  AccelStepper _j2;
  AccelStepper _j3;
  bool _homed;

  struct JointStepTarget {
    long j1;
    long j2;
    long j3;
  };

  JointStepTarget jointPoseToSteps(const JointPose& pose) const {
    return {
      degToSteps(1, pose.j1),
      degToSteps(2, pose.j2),
      degToSteps(3, pose.j3)
    };
  }

  JointStepTarget currentStepTarget() {
    return {
      _j1.currentPosition(),
      _j2.currentPosition(),
      _j3.currentPosition()
    };
  }

  // Coordinated move: sebelum moveTo(), skalakan maxSpeed/accel tiap sendi
  // proporsional terhadap jarak (steps) sendi itu dibanding sendi yang
  // menempuh jarak PALING JAUH pada segmen ini ("dominant axis").
  //
  // Kenapa ini yang bikin G1 lebih LURUS: kalau J2/J3 dibiarkan jalan di
  // maxSpeed config masing-masing (independen), sendi yang jaraknya lebih
  // pendek akan sampai duluan lalu diam, sementara sendi lain masih
  // bergerak -> lintasan antar-segmen melengkung, terlihat jelas pada G1
  // yang hanya mengubah Z (J1 diam, J2/J3 jarang punya rasio jarak sama).
  // Dengan skala proporsional ini, waktu tempuh trapezoid tiap sendi jadi
  // identik (matematis: t_accel dan t_total tidak berubah karena speed &
  // accel diskalakan dengan rasio yang sama), jadi ketiganya start/stop
  // bersamaan -> path jadi garis lurus di ruang Cartesian.
  //
  // Kenapa tidak lebih LAMBAT: sendi dominan (jarak terjauh) rasionya = 1,
  // jadi tetap dijalankan di kecepatan penuh seperti sebelumnya. Total
  // waktu segmen tetap ditentukan oleh sendi dominan itu.
  void commandStepTarget(const JointStepTarget& target) {
    const long d1 = target.j1 - _j1.currentPosition();
    const long d2 = target.j2 - _j2.currentPosition();
    const long d3 = target.j3 - _j3.currentPosition();
    const long dominant = maxAbs3(d1, d2, d3);

    if (dominant > 0) {
      applySyncedSpeed(_j1, d1, dominant, J1_MAX_SPEED_STEPS_PER_SEC, J1_ACCEL_STEPS_PER_SEC2);
      applySyncedSpeed(_j2, d2, dominant, J2_MAX_SPEED_STEPS_PER_SEC, J2_ACCEL_STEPS_PER_SEC2);
      applySyncedSpeed(_j3, d3, dominant, J3_MAX_SPEED_STEPS_PER_SEC, J3_ACCEL_STEPS_PER_SEC2);
    }

    _j1.moveTo(target.j1);
    _j2.moveTo(target.j2);
    _j3.moveTo(target.j3);
  }

  void applySyncedSpeed(AccelStepper& stepper, long axisDelta, long dominantDelta, float fullSpeed, float fullAccel) {
    float ratio = static_cast<float>(labs(axisDelta)) / static_cast<float>(dominantDelta);
    if (ratio > 1.0f) ratio = 1.0f;
    // Floor kecil supaya AccelStepper tidak diberi speed persis 0 saat axis
    // ini tetap harus menempuh beberapa step (mencegah stuck di speed=0).
    if (ratio < 0.02f && axisDelta != 0) ratio = 0.02f;

    stepper.setMaxSpeed(fullSpeed * ratio);
    stepper.setAcceleration(fullAccel * ratio);
  }

  void restoreAllNormalSpeeds() {
    restoreNormalSpeed(1, _j1);
    restoreNormalSpeed(2, _j2);
    restoreNormalSpeed(3, _j3);
  }

  // Monitor arus non-blocking untuk gaya G0 (satu gerakan langsung, sudah
  // di-moveTo() oleh pemanggil). Tidak ada rising-edge, blind-zone, atau
  // debounce: 1 sampel calibratedCurrentA() >= threshold langsung valid.
  bool runDescendMonitorLoop(float zMin, float overloadThresholdA, float afterTouchOffsetMm,
                              float sampleIntervalMs, const RobotKinematic& kinematic,
                              INA219Monitor& ina, Stream& out,
                              float& outFinalZ, float& outFinalCurrentA,
                              bool& outCurrentDetected, bool& outOffsetSatisfied) {
    float offsetTargetZ = zMin;
    int consecutiveOverThreshold = 0;

    uint32_t lastSampleMs = millis();
    const uint32_t startMs = lastSampleMs;
    const uint32_t sampleIntervalMsU = (sampleIntervalMs > 1.0f) ? static_cast<uint32_t>(sampleIntervalMs) : 1;

    while (isBusy()) {
      run();
      yield();

      const uint32_t now = millis();
      if ((now - lastSampleMs) >= sampleIntervalMsU) {
        lastSampleMs = now;

        const float currentA = ina.isReady() ? ina.calibratedCurrentA() : 0.0f;
        outFinalCurrentA = currentA;

        const CartesianPose nowPose = kinematic.forward(currentJointPose());
        outFinalZ = nowPose.z;

        // Blind-time: abaikan sample selama fase akselerasi awal supaya
        // lonjakan arus start-up tidak dihitung sebagai sentuhan objek.
        const bool pastBlindTime = (now - startMs) >= PICK_TOUCH_BLIND_TIME_MS;

        if (!outCurrentDetected && ina.isReady() && pastBlindTime) {
          if (currentA >= overloadThresholdA) {
            consecutiveOverThreshold++;
          } else {
            consecutiveOverThreshold = 0;
          }
        }

        if (!outCurrentDetected && consecutiveOverThreshold >= PICK_TOUCH_DEBOUNCE_SAMPLES) {
          outCurrentDetected = true;
          // Syarat stop tetap LOGIC AND:
          // 1) arus INA >= threshold, dan
          // 2) Z sudah turun sebesar afterTouchOffsetMm dari titik deteksi
          //    (atau sudah mentok di lantai mekanis zMin, mana yang lebih
          //    dulu tercapai).
          //
          // REVISI-5 (fix "tidak pernah pindah ke bin"): offsetTargetZ SEKARANG
          // di-clamp ke zMin. Sebelumnya offsetTargetZ TIDAK di-clamp, sehingga
          // kalau arus 0.9A baru terdeteksi saat Z sudah dekat zMin (< zMin +
          // afterTouchOffsetMm), offsetTargetZ jatuh DI BAWAH zMin -- padahal
          // gerak descend memang di-hard-limit berhenti paling jauh di zMin.
          // Akibatnya syarat "Z <= offsetTargetZ" MUSTAHIL tercapai secara
          // fisik, outOffsetSatisfied selalu false, dan sortSequence() selalu
          // abort (EVENT:SORT_ABORT_OFFSET_NOT_MET) walau lengan sudah turun
          // penuh sampai lantai. Dengan clamp ini, begitu lengan mencapai
          // zMin DAN arus sudah >= threshold, syarat AND dianggap terpenuhi
          // (menekan objek semaksimal yang diizinkan hard limit).
          offsetTargetZ = nowPose.z - afterTouchOffsetMm;
          if (offsetTargetZ < zMin) {
            offsetTargetZ = zMin;
          }

          out.print(F("EVENT:PICK_TOUCH_CURRENT_DETECTED start_z="));
          out.print(nowPose.z, 2);
          out.print(F(" target_z_after_offset="));
          out.print(offsetTargetZ, 2);
          out.print(F(" I="));
          out.println(currentA, 3);
        }

        if (outCurrentDetected && nowPose.z <= offsetTargetZ) {
          outOffsetSatisfied = true;
          out.print(F("EVENT:PICK_TOUCH_AND_OFFSET_OK Z="));
          out.print(nowPose.z, 2);
          out.print(F(" I="));
          out.print(currentA, 3);
          out.println(F(" condition=INA>=threshold&&Z_OFFSET"));
          stopAll();
          break;
        }
      }

      if ((millis() - startMs) > CARTESIAN_MOTION_TIMEOUT_MS) {
        stopAll();
        out.println(F("ERROR: descend monitor timeout"));
        return false;
      }
    }

    runToIdle(2000);
    finishDescendReport(zMin, offsetTargetZ, kinematic, out, outFinalZ, outCurrentDetected, outOffsetSatisfied);
    return true;
  }

  // Monitor arus non-blocking untuk gaya G1: menjalankan segmen + blending
  // PERSIS seperti runStepTargetsBlended(), ditambah sampling INA219 di
  // sela-sela loop tanpa menghentikan motor. Tidak ada debounce/rising-edge.
  bool runDescendBlendedWithMonitor(const JointStepTarget* targets, int count,
                                     float zMin,
                                     float overloadThresholdA, float afterTouchOffsetMm,
                                     float sampleIntervalMs, const RobotKinematic& kinematic,
                                     INA219Monitor& ina, Stream& out,
                                     float& outFinalZ, float& outFinalCurrentA,
                                     bool& outCurrentDetected, bool& outOffsetSatisfied) {
    if (count <= 0) return true;

    JointStepTarget segmentStart = currentStepTarget();
    int active = 0;
    commandStepTarget(targets[active]);

    float offsetTargetZ = -1.0e9f;  // belum valid sampai touch terdeteksi
    int consecutiveOverThreshold = 0;

    uint32_t lastSampleMs = millis();
    const uint32_t startMs = lastSampleMs;
    const uint32_t sampleIntervalMsU = (sampleIntervalMs > 1.0f) ? static_cast<uint32_t>(sampleIntervalMs) : 1;

    while (true) {
      run();
      yield();

      if (active < (count - 1)) {
        const long segmentDelta = maxSegmentDeltaAbs(segmentStart, targets[active]);
        const long blendWindow = blendWindowForSegment(segmentDelta);

        if (blendWindow > 0 && maxDistanceToGoAbs() <= blendWindow) {
          segmentStart = targets[active];
          active++;
          commandStepTarget(targets[active]);
        } else if (blendWindow == 0 && !isBusy()) {
          segmentStart = targets[active];
          active++;
          commandStepTarget(targets[active]);
        }
      } else if (!isBusy()) {
        break;
      }

      const uint32_t now = millis();
      if ((now - lastSampleMs) >= sampleIntervalMsU) {
        lastSampleMs = now;

        const float currentA = ina.isReady() ? ina.calibratedCurrentA() : 0.0f;
        outFinalCurrentA = currentA;

        const CartesianPose nowPose = kinematic.forward(currentJointPose());
        outFinalZ = nowPose.z;

        // Blind-time: abaikan sample selama fase akselerasi awal supaya
        // lonjakan arus start-up tidak dihitung sebagai sentuhan objek.
        const bool pastBlindTime = (now - startMs) >= PICK_TOUCH_BLIND_TIME_MS;

        if (!outCurrentDetected && ina.isReady() && pastBlindTime) {
          if (currentA >= overloadThresholdA) {
            consecutiveOverThreshold++;
          } else {
            consecutiveOverThreshold = 0;
          }
        }

        if (!outCurrentDetected && consecutiveOverThreshold >= PICK_TOUCH_DEBOUNCE_SAMPLES) {
          outCurrentDetected = true;
          // REVISI-5: clamp offsetTargetZ ke zMin (lihat penjelasan lengkap di
          // runDescendMonitorLoop()). Tanpa clamp ini, deteksi arus yang
          // terjadi mendekati lantai (Z < zMin + afterTouchOffsetMm) membuat
          // offsetTargetZ jatuh di bawah zMin -- target yang mustahil dicapai
          // karena gerak descend memang di-hard-limit berhenti di zMin -- dan
          // outOffsetSatisfied tidak akan pernah true, sehingga sortSequence()
          // selalu abort dan robot tidak pernah pindah ke bin.
          offsetTargetZ = nowPose.z - afterTouchOffsetMm;
          if (offsetTargetZ < zMin) {
            offsetTargetZ = zMin;
          }

          out.print(F("EVENT:PICK_TOUCH_CURRENT_DETECTED start_z="));
          out.print(nowPose.z, 2);
          out.print(F(" target_z_after_offset="));
          out.print(offsetTargetZ, 2);
          out.print(F(" I="));
          out.println(currentA, 3);
        }

        if (outCurrentDetected && nowPose.z <= offsetTargetZ) {
          outOffsetSatisfied = true;
          out.print(F("EVENT:PICK_TOUCH_AND_OFFSET_OK Z="));
          out.print(nowPose.z, 2);
          out.print(F(" I="));
          out.print(currentA, 3);
          out.println(F(" condition=INA>=threshold&&Z_OFFSET"));
          stopAll();
          break;
        }
      }

      if ((millis() - startMs) > CARTESIAN_MOTION_TIMEOUT_MS) {
        stopAll();
        out.println(F("ERROR: descend blended monitor timeout"));
        return false;
      }
    }

    runToIdle(2000);

    const CartesianPose finalPose = kinematic.forward(currentJointPose());
    outFinalZ = finalPose.z;

    if (!outCurrentDetected) {
      out.println(F("EVENT:PICK_TOUCH_NOT_DETECTED (reached final segment)"));
      out.println(F("WARNING: INA threshold tidak terdeteksi; Z tetap dibatasi minimum."));
    } else if (!outOffsetSatisfied) {
      // Dengan clamp REVISI-5, cabang ini seharusnya hanya kejadian kalau
      // motor berhenti/timeout SEBELUM benar-benar mencapai offsetTargetZ
      // (mis. gangguan mekanis), bukan lagi karena target di bawah zMin.
      out.print(F("EVENT:PICK_OFFSET_NOT_FULL reached_z="));
      out.print(outFinalZ, 2);
      out.print(F(" target_z="));
      out.println(offsetTargetZ, 2);
      out.println(F("WARNING: Lengan berhenti sebelum offsetTargetZ tercapai."));
    }

    return true;
  }

  void finishDescendReport(float zMin, float offsetTargetZ, const RobotKinematic& kinematic, Stream& out,
                            float& outFinalZ, bool& outCurrentDetected, bool& outOffsetSatisfied) {
    const CartesianPose finalPose = kinematic.forward(currentJointPose());
    outFinalZ = finalPose.z;

    if (!outCurrentDetected) {
      out.print(F("EVENT:PICK_TOUCH_NOT_DETECTED reached_zmin="));
      out.println(zMin, 2);
      out.println(F("WARNING: INA threshold tidak terdeteksi; Z tetap dibatasi minimum."));
    } else if (!outOffsetSatisfied) {
      // Dengan clamp REVISI-5, cabang ini seharusnya hanya kejadian kalau
      // motor berhenti/timeout SEBELUM benar-benar mencapai offsetTargetZ
      // (mis. gangguan mekanis), bukan lagi karena target di bawah zMin.
      out.print(F("EVENT:PICK_OFFSET_NOT_FULL reached_z="));
      out.print(outFinalZ, 2);
      out.print(F(" target_z="));
      out.println(offsetTargetZ, 2);
      out.println(F("WARNING: Lengan berhenti sebelum offsetTargetZ tercapai."));
    }
  }

  long maxAbs3(long a, long b, long c) const {
    long aa = labs(a);
    long bb = labs(b);
    long cc = labs(c);
    long m = (aa > bb) ? aa : bb;
    return (m > cc) ? m : cc;
  }

  long maxDistanceToGoAbs() {
    return maxAbs3(_j1.distanceToGo(), _j2.distanceToGo(), _j3.distanceToGo());
  }

  long maxSegmentDeltaAbs(const JointStepTarget& from, const JointStepTarget& to) const {
    return maxAbs3(to.j1 - from.j1, to.j2 - from.j2, to.j3 - from.j3);
  }

  long blendWindowForSegment(long maxSegmentDelta) const {
    if (maxSegmentDelta <= 4) return 0;

    long window = lroundf(static_cast<float>(maxSegmentDelta) * MOTION_BLEND_START_FRACTION);
    if (window < MOTION_BLEND_MIN_WINDOW_STEPS) window = MOTION_BLEND_MIN_WINDOW_STEPS;
    if (window > MOTION_BLEND_MAX_WINDOW_STEPS) window = MOTION_BLEND_MAX_WINDOW_STEPS;

    // Jangan sampai target segmen berikutnya masuk terlalu awal pada segmen pendek.
    const long maxAllowed = maxSegmentDelta - 2;
    if (window > maxAllowed) window = maxAllowed;
    if (window < 0) window = 0;
    return window;
  }

  bool runStepTargetsBlended(const JointStepTarget* targets, int count, Stream& out) {
    if (count <= 0) return true;

    JointStepTarget segmentStart = currentStepTarget();
    int active = 0;
    commandStepTarget(targets[active]);

    const uint32_t startMs = millis();
    while (true) {
      run();
      yield();

      if (active < (count - 1)) {
        const long segmentDelta = maxSegmentDeltaAbs(segmentStart, targets[active]);
        const long blendWindow = blendWindowForSegment(segmentDelta);

        // Blending: sebelum segmen aktif berhenti total, target berikutnya sudah diberikan.
        // Ini mengurangi efek stop-start pada G1 segmented.
        if (blendWindow > 0 && maxDistanceToGoAbs() <= blendWindow) {
          segmentStart = targets[active];
          active++;
          commandStepTarget(targets[active]);
        } else if (blendWindow == 0 && !isBusy()) {
          segmentStart = targets[active];
          active++;
          commandStepTarget(targets[active]);
        }
      } else if (!isBusy()) {
        break;
      }

      if ((millis() - startMs) > CARTESIAN_MOTION_TIMEOUT_MS) {
        stopAll();
        out.println(F("ERROR: blended linear motion timeout"));
        return false;
      }
    }

    return true;
  }

  void configureLimitPin(int pin) {
    if (LIMIT_USE_INTERNAL_PULLUP) {
      if (LIMIT_ACTIVE_LOW) {
        pinMode(pin, INPUT_PULLUP);
      } else {
        #if defined(INPUT_PULLDOWN)
          pinMode(pin, INPUT_PULLDOWN);
        #else
          pinMode(pin, INPUT);
        #endif
      }
    } else {
      pinMode(pin, INPUT);
    }
  }

  void configureStepper(AccelStepper& stepper, int enPin, float maxSpeed, float accel) {
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(accel);
    stepper.setPinsInverted(false, !STEP_ACTIVE_HIGH, ENABLE_ACTIVE_LOW);

    if (enPin >= 0) {
      stepper.setEnablePin(enPin);
      stepper.enableOutputs();
    }
  }

  float stepsPerDeg(uint8_t axis) const {
    if (axis == 1) return J1_STEPS_PER_DEG;
    if (axis == 2) return J2_STEPS_PER_DEG;
    return J3_STEPS_PER_DEG;
  }

  int normalDirSign(uint8_t axis) const {
    if (axis == 1) return J1_DIR_INVERT ? -1 : 1;
    if (axis == 2) return J2_DIR_INVERT ? -1 : 1;
    return J3_DIR_INVERT ? -1 : 1;
  }

  float homeDeg(uint8_t axis) const {
    if (axis == 1) return J1_HOME_ANGLE_DEG;
    if (axis == 2) return J2_HOME_ANGLE_DEG;
    return J3_HOME_ANGLE_DEG;
  }

  long degToSteps(uint8_t axis, float deg) const {
    const float deltaDeg = deg - homeDeg(axis);
    return lroundf(deltaDeg * stepsPerDeg(axis) * normalDirSign(axis));
  }

  float stepsToDeg(uint8_t axis, long steps) const {
    return homeDeg(axis) + (static_cast<float>(steps) / (stepsPerDeg(axis) * normalDirSign(axis)));
  }

  long absDegToSteps(uint8_t axis, float deg) const {
    return labs(lroundf(fabsf(deg) * stepsPerDeg(axis)));
  }

  bool targetInRange(const JointPose& target, Stream& out) const {
    if (target.j1 < J1_MIN_DEG || target.j1 > J1_MAX_DEG) {
      out.print(F("ERROR: J1 out of range: ")); out.println(target.j1, 2);
      return false;
    }
    if (target.j2 < J2_MIN_DEG || target.j2 > J2_MAX_DEG) {
      out.print(F("ERROR: J2 out of range: ")); out.println(target.j2, 2);
      return false;
    }
    if (target.j3 < J3_MIN_DEG || target.j3 > J3_MAX_DEG) {
      out.print(F("ERROR: J3 out of range: ")); out.println(target.j3, 2);
      return false;
    }
    return true;
  }

  void setRawDir(int dirPin, bool dirLevel) {
    digitalWrite(dirPin, dirLevel ? HIGH : LOW);
    delayMicroseconds(DIR_SETUP_US);
  }

  void pulseRawStep(int stepPin) {
    if (STEP_ACTIVE_HIGH) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(STEP_PULSE_US);
      digitalWrite(stepPin, LOW);
    } else {
      digitalWrite(stepPin, LOW);
      delayMicroseconds(STEP_PULSE_US);
      digitalWrite(stepPin, HIGH);
    }
  }

  void waitStepInterval() {
    const uint32_t rest = (HOMING_STEP_INTERVAL_US > STEP_PULSE_US) ? (HOMING_STEP_INTERVAL_US - STEP_PULSE_US) : 1;
    delayMicroseconds(rest);
  }

  bool rawStepCount(int stepPin, int dirPin, bool dirLevel, long steps, int limitPin, bool stopIfLimit, Stream& out, const __FlashStringHelper* label) {
    setRawDir(dirPin, dirLevel);

    for (long i = 0; i < steps; i++) {
      if (stopIfLimit && limitActive(limitPin)) {
        return true;
      }

      pulseRawStep(stepPin);
      waitStepInterval();
      yield();
    }

    if (stopIfLimit && !limitActive(limitPin)) {
      out.print(F("ERROR: "));
      out.print(label);
      out.println(F(" limit not triggered within max steps"));
      return false;
    }

    return true;
  }

  bool homeAxisRaw(uint8_t axis,
                   AccelStepper& stepper,
                   int stepPin,
                   int dirPin,
                   int limitPin,
                   bool dirLevelToLimit,
                   float homeOffsetDeg,
                   Stream& out) {
    restoreNormalSpeed(axis, stepper);

    out.print(F("J")); out.print(axis);
    out.print(F(" limit before: raw=")); out.print(digitalRead(limitPin));
    out.print(F(" status=")); out.println(limitActive(limitPin) ? F("TRIGGERED") : F("open"));

    if (!limitActive(limitPin)) {
      if (!rawStepCount(stepPin, dirPin, dirLevelToLimit, HOMING_MAX_STEPS, limitPin, true, out, F("homing seek"))) {
        return false;
      }
    } else {
      out.print(F("J")); out.print(axis);
      out.println(F(" already on limit, applying offset directly."));
    }

    // Titik limit switch dianggap raw zero.
    stepper.setCurrentPosition(0);

    if (HOMING_BACKOFF_STEPS > 0) {
      rawStepCount(stepPin, dirPin, !dirLevelToLimit, HOMING_BACKOFF_STEPS, limitPin, false, out, F("backoff"));
      delay(80);
      rawStepCount(stepPin, dirPin, dirLevelToLimit, HOMING_MAX_STEPS, limitPin, true, out, F("slow seek"));
      stepper.setCurrentPosition(0);
    }

    const long offsetSteps = absDegToSteps(axis, homeOffsetDeg);
    if (offsetSteps > 0) {
      const bool offsetDirLevel = (homeOffsetDeg >= 0.0f);
      out.print(F("J")); out.print(axis);
      out.print(F(" applying raw HOME_OFFSET deg=")); out.print(homeOffsetDeg, 3);
      out.print(F(" steps=")); out.println(offsetSteps);
      rawStepCount(stepPin, dirPin, offsetDirLevel, offsetSteps, limitPin, false, out, F("home offset"));
    }

    // Setelah offset, posisi fisik ini dianggap kinematic home angle.
    stepper.setCurrentPosition(0);
    restoreNormalSpeed(axis, stepper);

    out.print(F("J")); out.print(axis);
    out.print(F(" homed. Kinematic angle=")); out.println(homeDeg(axis), 3);
    return true;
  }

  void restoreNormalSpeed(uint8_t axis, AccelStepper& stepper) {
    if (axis == 1) {
      stepper.setMaxSpeed(J1_MAX_SPEED_STEPS_PER_SEC);
      stepper.setAcceleration(J1_ACCEL_STEPS_PER_SEC2);
    } else if (axis == 2) {
      stepper.setMaxSpeed(J2_MAX_SPEED_STEPS_PER_SEC);
      stepper.setAcceleration(J2_ACCEL_STEPS_PER_SEC2);
    } else {
      stepper.setMaxSpeed(J3_MAX_SPEED_STEPS_PER_SEC);
      stepper.setAcceleration(J3_ACCEL_STEPS_PER_SEC2);
    }
  }
};
