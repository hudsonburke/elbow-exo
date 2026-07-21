#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ============================================================
// Teensy two-bus BNO055 elbow flexion + forearm pronation test
//
// No I2C multiplexer is used. Each BNO055 is connected to a separate
// hardware I2C controller on the Teensy, so both sensors may use address 0x28.
//   Upper-arm IMU: Wire
//   Forearm IMU:   Wire1
//
// Controls:
//   r = capture the current pose as zero
//   m = print the menu
//
// Output:
//   qUpperZeroed
//   qForearmZeroed
//   qRelativeRaw
//   qJointZeroed
//   qFlexion
//   qPronation
//   ElbowFlexDeg
//   ForearmPronDeg
//
// IMPORTANT AXIS SETTINGS
// The forearm's longitudinal axis was previously found to align with
// local +X, so FOREARM_LONG_AXIS defaults to (1, 0, 0).
// FLEXION_AXIS must be adjusted to match the calibrated elbow hinge axis
// in the upper-arm/joint coordinate frame.
// ============================================================

Adafruit_BNO055 bnoUpper(0, 0x28, &Wire);
Adafruit_BNO055 bnoForearm(1, 0x28, &Wire1);

const unsigned long SAMPLE_US = 10000;  // 100 Hz IMU/math update
const unsigned long PRINT_MS  = 100;    // 10 Hz serial output

// Anatomical axes expressed in the zeroed joint frame.
// Keep FOREARM_LONG_AXIS at +X if that matches the physical IMU mounting.
struct Vec3 {
  float x;
  float y;
  float z;
};

const Vec3 FOREARM_LONG_AXIS = {1.0f, 0.0f, 0.0f};
const Vec3 FLEXION_AXIS      = {0.0f, 0.0f, -1.0f};  // Change if needed

bool imuUpperOk = false;
bool imuForearmOk = false;
bool zeroCaptured = false;

unsigned long lastSampleUs = 0;
unsigned long lastPrintMs = 0;

imu::Quaternion qUpperRaw(1, 0, 0, 0);
imu::Quaternion qForearmRaw(1, 0, 0, 0);
imu::Quaternion qUpperZero(1, 0, 0, 0);
imu::Quaternion qForearmZero(1, 0, 0, 0);
imu::Quaternion qRelativeZero(1, 0, 0, 0);

imu::Quaternion qUpperZeroed(1, 0, 0, 0);
imu::Quaternion qForearmZeroed(1, 0, 0, 0);
imu::Quaternion qRelativeRaw(1, 0, 0, 0);
imu::Quaternion qJointZeroed(1, 0, 0, 0);
imu::Quaternion qFlexion(1, 0, 0, 0);
imu::Quaternion qPronation(1, 0, 0, 0);

float elbowFlexDeg = 0.0f;
float forearmPronDeg = 0.0f;

// ---------------- Vector helpers ----------------

float dot3(const Vec3& a, const Vec3& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3 cross3(const Vec3& a, const Vec3& b) {
  return {
    a.y*b.z - a.z*b.y,
    a.z*b.x - a.x*b.z,
    a.x*b.y - a.y*b.x
  };
}

float norm3(const Vec3& v) {
  return sqrtf(dot3(v, v));
}

Vec3 scale3(const Vec3& v, float s) {
  return {v.x*s, v.y*s, v.z*s};
}

Vec3 sub3(const Vec3& a, const Vec3& b) {
  return {a.x-b.x, a.y-b.y, a.z-b.z};
}

Vec3 unit3(const Vec3& v) {
  float n = norm3(v);
  if (!isfinite(n) || n < 1.0e-6f) {
    return {1.0f, 0.0f, 0.0f};
  }
  return scale3(v, 1.0f/n);
}

// Remove the component parallel to axis.
Vec3 rejectFromAxis(const Vec3& v, const Vec3& axisUnit) {
  return sub3(v, scale3(axisUnit, dot3(v, axisUnit)));
}

// ---------------- Quaternion helpers ----------------

bool quatValid(const imu::Quaternion& q) {
  float w = q.w();
  float x = q.x();
  float y = q.y();
  float z = q.z();

  if (!isfinite(w) || !isfinite(x) || !isfinite(y) || !isfinite(z)) {
    return false;
  }
  float mag2 = w*w + x*x + y*y + z*z;
  return isfinite(mag2) && mag2 > 1.0e-8f;
}

imu::Quaternion unitQ(imu::Quaternion q) {
  q.normalize();
  return q;
}

imu::Quaternion relativeQ(const imu::Quaternion& parent,
                          const imu::Quaternion& child) {
  return unitQ(parent.conjugate() * child);
}

imu::Quaternion zeroAgainst(const imu::Quaternion& raw,
                            const imu::Quaternion& zero) {
  return unitQ(zero.conjugate() * raw);
}

imu::Quaternion quatFromAxisAngle(const Vec3& axis, float angleRad) {
  Vec3 a = unit3(axis);
  float half = 0.5f * angleRad;
  float s = sinf(half);
  return unitQ(imu::Quaternion(cosf(half), a.x*s, a.y*s, a.z*s));
}

Vec3 rotateVector(const Vec3& v, const imu::Quaternion& q) {
  imu::Quaternion vq(0.0, v.x, v.y, v.z);
  imu::Quaternion rq = q * vq * q.conjugate();
  return {rq.x(), rq.y(), rq.z()};
}

// Signed angle from 'from' to 'to' about 'axis'.
// Both vectors are projected onto the plane perpendicular to axis first.
float signedAngleAboutAxis(const Vec3& from,
                           const Vec3& to,
                           const Vec3& axis) {
  Vec3 a = unit3(axis);
  Vec3 f = unit3(rejectFromAxis(from, a));
  Vec3 t = unit3(rejectFromAxis(to, a));

  float sinTerm = dot3(a, cross3(f, t));
  float cosTerm = constrain(dot3(f, t), -1.0f, 1.0f);
  return atan2f(sinTerm, cosTerm);
}

// Extract the signed twist angle represented by q about a known unit axis.
float signedTwistAngle(const imu::Quaternion& q, const Vec3& axis) {
  Vec3 a = unit3(axis);
  float projected = q.x()*a.x + q.y()*a.y + q.z()*a.z;
  return 2.0f * atan2f(projected, q.w());
}

float wrapDeg180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

void printQuat(const char* label, const imu::Quaternion& q) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print(q.w(), 6);
  Serial.print(", ");
  Serial.print(q.x(), 6);
  Serial.print(", ");
  Serial.print(q.y(), 6);
  Serial.print(", ");
  Serial.println(q.z(), 6);
}

// ---------------- IMU and decomposition ----------------

bool readRawQuaternions(imu::Quaternion& upper, imu::Quaternion& forearm) {
  if (!imuUpperOk || !imuForearmOk) {
    return false;
  }

  imu::Quaternion upperCandidate = bnoUpper.getQuat();
  imu::Quaternion forearmCandidate = bnoForearm.getQuat();

  if (!quatValid(upperCandidate) || !quatValid(forearmCandidate)) {
    return false;
  }

  upper = unitQ(upperCandidate);
  forearm = unitQ(forearmCandidate);
  return true;
}

void captureZero() {
  imu::Quaternion upper;
  imu::Quaternion forearm;

  if (!readRawQuaternions(upper, forearm)) {
    Serial.println("Zero failed: invalid IMU quaternion sample.");
    return;
  }

  qUpperRaw = upper;
  qForearmRaw = forearm;
  qUpperZero = upper;
  qForearmZero = forearm;
  qRelativeZero = relativeQ(upper, forearm);

  qUpperZeroed = imu::Quaternion(1, 0, 0, 0);
  qForearmZeroed = imu::Quaternion(1, 0, 0, 0);
  qRelativeRaw = qRelativeZero;
  qJointZeroed = imu::Quaternion(1, 0, 0, 0);
  qFlexion = imu::Quaternion(1, 0, 0, 0);
  qPronation = imu::Quaternion(1, 0, 0, 0);
  elbowFlexDeg = 0.0f;
  forearmPronDeg = 0.0f;
  zeroCaptured = true;

  Serial.println("Zero captured. Current pose is flexion = 0 deg, pronation = 0 deg.");
}

void updateKinematics() {
  imu::Quaternion upper;
  imu::Quaternion forearm;

  // Ignore a brief invalid sample and retain the last valid outputs.
  if (!readRawQuaternions(upper, forearm)) {
    return;
  }

  qUpperRaw = upper;
  qForearmRaw = forearm;

  if (!zeroCaptured) {
    return;
  }

  // Individual zeroed orientations for telemetry.
  qUpperZeroed = zeroAgainst(qUpperRaw, qUpperZero);
  qForearmZeroed = zeroAgainst(qForearmRaw, qForearmZero);

  // Relative orientation and zeroed joint orientation.
  qRelativeRaw = relativeQ(qUpperRaw, qForearmRaw);
  qJointZeroed = zeroAgainst(qRelativeRaw, qRelativeZero);

  Vec3 longAxis = unit3(FOREARM_LONG_AXIS);
  Vec3 flexAxis = unit3(FLEXION_AXIS);

  // Pronation does not change the forearm longitudinal direction.
  // Therefore qJointZeroed * longAxis reveals flexion independent of pronation.
  Vec3 currentLongAxis = unit3(rotateVector(longAxis, qJointZeroed));

  float flexRad = signedAngleAboutAxis(longAxis, currentLongAxis, flexAxis);
  qFlexion = quatFromAxisAngle(flexAxis, flexRad);

  // Model: qJointZeroed = qFlexion * qPronation.
  // Removing flexion leaves the local forearm-axis rotation.
  qPronation = unitQ(qFlexion.conjugate() * qJointZeroed);
  float pronRad = signedTwistAngle(qPronation, longAxis);

  // Replace residual with its pure modeled twist quaternion for clean output.
  qPronation = quatFromAxisAngle(longAxis, pronRad);

  elbowFlexDeg = wrapDeg180(flexRad * 180.0f / PI);
  forearmPronDeg = wrapDeg180(pronRad * 180.0f / PI);
}

void printOutput() {
  if (!zeroCaptured) {
    return;
  }

  Serial.println();
 

  Serial.print("ElbowFlexDeg: ");
  Serial.print(elbowFlexDeg, 2);
  Serial.print(" | ForearmPronDeg: ");
  Serial.println(forearmPronDeg, 2);
}

void printMenu() {
  Serial.println();
  Serial.println("Two-DOF elbow/pronation test");
  Serial.println("r = zero current pose");
  Serial.println("m = print menu");
  Serial.println("Configured forearm longitudinal axis: +X");
  Serial.println("Configured elbow flexion axis: +Z (change FLEXION_AXIS if needed)");
  Serial.println();
}

void checkSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') continue;

    if (c == 'r' || c == 'R') {
      captureZero();
    } else if (c == 'm' || c == 'M') {
      printMenu();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  // Start the Teensy's two independent hardware I2C buses.
  Wire.begin();
  Wire1.begin();

  Serial.println("Starting BNO055 IMUs in default fusion mode...");

  imuUpperOk = bnoUpper.begin();
  imuForearmOk = bnoForearm.begin();

  Serial.println("Upper-arm IMU is on Wire; forearm IMU is on Wire1.");

  Serial.print("Upper IMU: ");
  Serial.println(imuUpperOk ? "OK" : "NOT FOUND");
  Serial.print("Forearm IMU: ");
  Serial.println(imuForearmOk ? "OK" : "NOT FOUND");

  printMenu();
  Serial.println("Place the arm in the desired neutral pose, then press r.");

  lastSampleUs = micros();
}

void loop() {
  checkSerial();

  unsigned long nowUs = micros();
  if (nowUs - lastSampleUs >= SAMPLE_US) {
    lastSampleUs = nowUs;
    updateKinematics();
  }

  unsigned long nowMs = millis();
  if (nowMs - lastPrintMs >= PRINT_MS) {
    lastPrintMs = nowMs;
    printOutput();
  }
}