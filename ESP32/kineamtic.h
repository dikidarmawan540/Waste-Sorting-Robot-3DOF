#pragma once

// kineamtic.h (filename intentionally kept as originally requested: kineamtic.h)
// Coordinates: X/Y/Z in mm, X/Y origin at the base rotation center, Z=0 at the shoulder/J2 axis per the guide. +Y=front, -Y=back (bin direction), +X=right (top view), +Z=up.
// Joint angles: J1=0 faces +Y/front, J1=+90 faces +X/right, J1=+/-180 faces -Y/back. J2=lower shank angle vs horizontal (90=vertical). J3=absolute high shank angle vs horizontal (0=horizontal forward).
// 90-degree home pose: J1=0, J2=90, J3=0 (lower shank vertical, high shank horizontal).

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

  // Small tolerance to absorb floating-point rounding error when the target is exactly at a boundary (Z_MIN_MM/Z_MAX_MM/etc.)
  static constexpr float BOUNDARY_EPSILON = 0.01f;

  static bool isWithin(float value, float low, float high) {
    return value >= (low - BOUNDARY_EPSILON) && value <= (high + BOUNDARY_EPSILON);
  }

  KinematicResult inverse(const CartesianPose& target) const {
    KinematicResult result;
    result.ok = false;
    result.joints = {0.0f, 0.0f, 0.0f};
    result.message[0] = '\0';

    // R is the horizontal distance from the target to the base axis.
    const float r = sqrtf((target.x * target.x) + (target.y * target.y));

    if (r < (R_MIN_MM - BOUNDARY_EPSILON) || r > (R_MAX_MM + BOUNDARY_EPSILON)) {
      snprintf(result.message, sizeof(result.message), "R out of range: %.2f mm", r);
      return result;
    }

    if (target.z < (Z_MIN_MM - BOUNDARY_EPSILON) || target.z > (Z_MAX_MM + BOUNDARY_EPSILON)) {
      snprintf(result.message, sizeof(result.message), "Z out of range: %.2f mm", target.z);
      return result;
    }

    // Base angle; atan2(X,Y) is used so 0 degrees faces +Y/front instead of +X.
    float j1 = radToDeg(atan2f(target.x, target.y));
    j1 = normalizeDeg180(j1);

    // Keep values exactly at +/-180 (atan2/normalize tie-breaking) without clamping to J1_MIN/MAX_DEG here, so isWithin() can still detect out-of-range.
    if (j1 >= 179.999f) j1 = 179.999f;
    if (j1 <= -179.999f) j1 = -179.999f;

    if (!isWithin(j1, J1_MIN_DEG, J1_MAX_DEG)) {
      snprintf(result.message, sizeof(result.message), "J1 out of range: %.2f deg (limit +/-%.0f)", j1, J1_MAX_DEG);
      return result;
    }

    // 2-link inverse kinematics on the radial-Z plane; the suction cup target point minus END_EFFECTOR_OFFSET gives the wrist point (SHOULDER_Z_OFFSET_MM is normally 0 since Z=0 is at the shoulder).
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

    // The negative elbow branch is chosen so the HOME_90 pose yields J2=90 degrees and absolute J3=0 degrees.
    const float elbowRelativeRad = -acosf(cosElbow);

    const float j2Rad = atan2f(wristZ, wristR)
                      - atan2f(L2 * sinf(elbowRelativeRad),
                               L1 + (L2 * cosf(elbowRelativeRad)));

    // J3 in this firmware is the absolute high shank angle relative to horizontal, not the angle relative to the lower shank.
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

    // Wrist position on the radial-Z plane; J2 = absolute lower shank angle vs horizontal, J3 = absolute high shank angle vs horizontal.
    const float wristR = (L1 * cosf(j2Rad)) + (L2 * cosf(j3Rad));
    const float z = SHOULDER_Z_OFFSET_MM + (L1 * sinf(j2Rad)) + (L2 * sinf(j3Rad));
    const float r = wristR + END_EFFECTOR_OFFSET_MM;

    // Inverse of atan2(X,Y): X = r*sin(J1), Y = r*cos(J1).
    const float x = r * sinf(j1Rad);
    const float y = r * cosf(j1Rad);

    return {x, y, z};
  }
};
