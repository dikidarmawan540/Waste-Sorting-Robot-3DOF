#pragma once

/*
  kineamtic.h
  Nama file sengaja mengikuti permintaan awal: kineamtic.h.

  Konvensi koordinat:
  - X/Y/Z adalah koordinat Cartesian dalam milimeter.
  - Origin X/Y berada di pusat rotasi base.
  - Z = 0 mengikuti referensi poros shoulder/J2 sesuai guide.
  - +Y = depan robot.
  - -Y = belakang robot, dipakai untuk bin/tempat sampah.
  - +X = kanan robot dari top view.
  - +Z = atas.

  Konvensi sudut joint revisi:
  - J1 = 0 derajat menghadap +Y/depan.
  - J1 = +90 derajat menghadap +X/kanan.
  - J1 = +/-180 derajat menghadap -Y/belakang.
  - J2 adalah sudut lower shank terhadap bidang horizontal/radial.
    J2 = 90 derajat berarti lower shank vertikal.
  - J3 adalah sudut absolut high shank terhadap bidang horizontal/radial.
    J3 = 0 derajat berarti high shank horizontal ke depan.

  Dengan konvensi ini, pose home 90 derajat adalah:
  J1=0, J2=90, J3=0
  sehingga lower shank vertikal dan high shank horizontal.
*/

#include <Arduino.h>
#include <math.h>
#include "config.h"

struct CartesianPose {
  float x;
  float y;
  float z;
};

struct JointPose {
  float j1;
  float j2;
  float j3;
};

struct KinematicResult {
  bool ok;
  JointPose joints;
  char message[96];
};

class RobotKinematic {
public:
  static float degToRad(float deg) {
    return deg * PI / 180.0f;
  }

  static float radToDeg(float rad) {
    return rad * 180.0f / PI;
  }

  static float clampFloat(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
  }

  static float normalizeDeg180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
  }

  // Toleransi kecil untuk menyerap error pembulatan floating-point saat
  // target berada TEPAT di batas (Z_MIN_MM/Z_MAX_MM/dll). Tanpa ini,
  // segmen G1 terakhir yang menuju batas persis (mis. probe turun sampai
  // Z_MIN_MM) bisa gagal validasi hanya karena selisih 0.0001mm hasil
  // akumulasi start + dz*t, padahal target itu sendiri valid.
  static constexpr float BOUNDARY_EPSILON = 0.01f;

  static bool isWithin(float value, float low, float high) {
    return value >= (low - BOUNDARY_EPSILON) && value <= (high + BOUNDARY_EPSILON);
  }

  KinematicResult inverse(const CartesianPose& target) const {
    KinematicResult result;
    result.ok = false;
    result.joints = {0.0f, 0.0f, 0.0f};
    result.message[0] = '\0';

    // R adalah jarak horizontal target dari poros base.
    const float r = sqrtf((target.x * target.x) + (target.y * target.y));

    if (r < (R_MIN_MM - BOUNDARY_EPSILON) || r > (R_MAX_MM + BOUNDARY_EPSILON)) {
      snprintf(result.message, sizeof(result.message), "R out of range: %.2f mm", r);
      return result;
    }

    if (target.z < (Z_MIN_MM - BOUNDARY_EPSILON) || target.z > (Z_MAX_MM + BOUNDARY_EPSILON)) {
      snprintf(result.message, sizeof(result.message), "Z out of range: %.2f mm", target.z);
      return result;
    }

    // Sudut base.
    // atan2(X,Y) dipakai agar 0 derajat menghadap +Y/depan, bukan +X.
    float j1 = radToDeg(atan2f(target.x, target.y));
    j1 = normalizeDeg180(j1);

    // Hanya jaga nilai tepat +/-180 (kasus tie-breaking atan2/normalize), JANGAN
    // clamp ke J1_MIN/MAX_DEG di sini -- kalau di-clamp duluan, pengecekan
    // isWithin() di bawah jadi tidak pernah bisa mendeteksi out-of-range.
    if (j1 >= 179.999f) j1 = 179.999f;
    if (j1 <= -179.999f) j1 = -179.999f;

    if (!isWithin(j1, J1_MIN_DEG, J1_MAX_DEG)) {
      snprintf(result.message, sizeof(result.message), "J1 out of range: %.2f deg (limit +/-%.0f)", j1, J1_MAX_DEG);
      return result;
    }

    // Inverse kinematic 2-link pada bidang radial-Z.
    // Titik target suction cup dikurangi END_EFFECTOR_OFFSET agar menjadi titik wrist.
    // Karena Z=0 mengikuti shoulder guide, SHOULDER_Z_OFFSET_MM normalnya 0.
    const float wristR = r - END_EFFECTOR_OFFSET_MM;
    const float wristZ = target.z - SHOULDER_Z_OFFSET_MM;

    if (wristR < 0.0f) {
      snprintf(result.message, sizeof(result.message), "Wrist R negative: %.2f mm", wristR);
      return result;
    }

    const float L1 = LOW_SHANK_LENGTH_MM;
    const float L2 = HIGH_SHANK_LENGTH_MM;
    const float d2 = (wristR * wristR) + (wristZ * wristZ);
    const float d = sqrtf(d2);

    if (d > (L1 + L2) || d < fabsf(L1 - L2)) {
      snprintf(result.message, sizeof(result.message), "IK unreachable d=%.2f mm", d);
      return result;
    }

    float cosElbow = (d2 - (L1 * L1) - (L2 * L2)) / (2.0f * L1 * L2);
    cosElbow = clampFloat(cosElbow, -1.0f, 1.0f);

    // Hitung sudut relatif elbow lebih dulu.
    // Cabang negatif dipilih supaya pose HOME_90 menghasilkan:
    //   J2 = 90 derajat
    //   J3 absolut = 0 derajat
    const float elbowRelativeRad = -acosf(cosElbow);

    const float j2Rad = atan2f(wristZ, wristR)
                      - atan2f(L2 * sinf(elbowRelativeRad),
                               L1 + (L2 * cosf(elbowRelativeRad)));

    // J3 pada firmware ini adalah sudut absolut high shank terhadap horizontal,
    // bukan sudut relatif terhadap lower shank.
    const float j3AbsRad = j2Rad + elbowRelativeRad;

    const float j2 = radToDeg(j2Rad);
    const float j3 = radToDeg(j3AbsRad);

    if (!isWithin(j2, J2_MIN_DEG, J2_MAX_DEG)) {
      snprintf(result.message, sizeof(result.message), "J2 out of range: %.2f deg", j2);
      return result;
    }

    if (!isWithin(j3, J3_MIN_DEG, J3_MAX_DEG)) {
      snprintf(result.message, sizeof(result.message), "J3 out of range: %.2f deg", j3);
      return result;
    }

    result.ok = true;
    result.joints = {j1, j2, j3};
    snprintf(result.message, sizeof(result.message), "OK");
    return result;
  }

  CartesianPose forward(const JointPose& joints) const {
    const float L1 = LOW_SHANK_LENGTH_MM;
    const float L2 = HIGH_SHANK_LENGTH_MM;

    const float j1Rad = degToRad(joints.j1);
    const float j2Rad = degToRad(joints.j2);
    const float j3Rad = degToRad(joints.j3);

    // Posisi wrist pada bidang radial-Z.
    // J2 = sudut absolut lower shank terhadap horizontal.
    // J3 = sudut absolut high shank terhadap horizontal.
    const float wristR = (L1 * cosf(j2Rad)) + (L2 * cosf(j3Rad));
    const float z = SHOULDER_Z_OFFSET_MM + (L1 * sinf(j2Rad)) + (L2 * sinf(j3Rad));
    const float r = wristR + END_EFFECTOR_OFFSET_MM;

    // Kebalikan dari atan2(X,Y):
    // X = r*sin(J1), Y = r*cos(J1).
    const float x = r * sinf(j1Rad);
    const float y = r * cosf(j1Rad);

    return {x, y, z};
  }
};
