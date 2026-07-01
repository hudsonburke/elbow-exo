#include <Arduino.h>
#include <math.h>
#include <ctype.h>

#include <Wire.h>
#include <Encoder.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// =========================
// IMU SETUP
// =========================
// Two BNO055s on separate Teensy I2C buses.
// Upper-arm/base IMU:  Wire
// Forearm/moving IMU: Wire1
//
// Joint angle is computed from the zeroed relative quaternion:
//   qRelRaw   = conjugate(qUpperRaw) * qForearmRaw
//   qJoint    = conjugate(qRelZero)  * qRelRaw
//   joint deg = magnitude of qJoint
//
// After imuZero(), qUpper, qForearm, qRel, and qJoint all report identity
// at the zero pose: (1, 0, 0, 0). The raw sensor quaternions are still read
// from the BNO055s, but the printed/control quaternions are zeroed values.

Adafruit_BNO055 bnoUpper(0, 0x28, &Wire);
Adafruit_BNO055 bnoForearm(1, 0x28, &Wire1);

bool imuUpperOk = false;
bool imuForearmOk = false;

imu::Quaternion qUpperRaw(1.0, 0.0, 0.0, 0.0);
imu::Quaternion qForearmRaw(1.0, 0.0, 0.0, 0.0);

imu::Quaternion qUpperZero(1.0, 0.0, 0.0, 0.0);
imu::Quaternion qForearmZero(1.0, 0.0, 0.0, 0.0);
imu::Quaternion qRelZero(1.0, 0.0, 0.0, 0.0);

// Zeroed telemetry/control quaternions.
imu::Quaternion qUpper(1.0, 0.0, 0.0, 0.0);
imu::Quaternion qForearm(1.0, 0.0, 0.0, 0.0);
imu::Quaternion qRel(1.0, 0.0, 0.0, 0.0);
imu::Quaternion qJoint(1.0, 0.0, 0.0, 0.0);

float jointRawDeg = 0.0;
float jointAngleDeg = 0.0;

// IMU fault guard: brief bad/NaN reads are ignored so PID keeps using
// the most recent valid joint angle instead of reacting to a corrupt sample.
bool imuLastReadValid = true;
unsigned long imuBadReadCount = 0;


// =========================
// MOTOR PINS
// =========================

// Motor 1
#define M1_IN1 4
#define M1_IN2 5
#define M1_ENC_A 30
#define M1_ENC_B 31

// Motor 2
#define M2_IN1 2
#define M2_IN2 3
#define M2_ENC_A 28
#define M2_ENC_B 29

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);


// =========================
// SETTINGS
// =========================

const int FWD = 1;
const int REV = -1;

const int M1_ENC_SIGN = 1;
const int M2_ENC_SIGN = 1;

const int M1_MOT_SIGN = 1;
const int M2_MOT_SIGN = 1;

const int M1_ROLL_SIGN = 1;
const int M2_ROLL_SIGN = -1;

const int MIN_PWM = 125;
const int MAX_PWM = 255;
const int CMD_PWM = 220;
const int Man_speed = 125;

const unsigned long CTRL_US = 10000;   // 10 ms
const unsigned long PRINT_MS = 200;

const float KEY_TARGETS[10] = {
  0.0, 10.0, 20.0, 30.0, 40.0,
  50.0, 60.0, 70.0, 80.0, 90.0
};


// =========================
// MOTOR STRUCT
// =========================

struct Motor {
  const char* name;

  int in1;
  int in2;
  Encoder* enc;

  int encSign;
  int motSign;
  int rollSign;

  float kp;
  float kd;
  float ki;

  int minPwm;
  int maxPwm;
  int slowPwm;

  float tol;
  float slowZone;
  unsigned long timeout;

  float target;
  float lastErr;
  float sumErr;

  bool active;
  bool holding;
  bool printed;

  unsigned long startMs;
};

Motor m1 = {
  "Motor 1",
  M1_IN1, M1_IN2, &enc1,
  M1_ENC_SIGN, M1_MOT_SIGN, M1_ROLL_SIGN,
  1.75, 0.0, 0.125,
  150, 225, 150,
  1.0, 5.0, 10000,
  0.0, 0.0, 0.0,
  false, false, false,
  0
};

Motor m2 = {
  "Motor 2",
  M2_IN1, M2_IN2, &enc2,
  M2_ENC_SIGN, M2_MOT_SIGN, M2_ROLL_SIGN,
  1.75, 0.0, 0.125,
  150, 225, 150,
  1.0, 5.0, 10000,
  0.0, 0.0, 0.0,
  false, false, false,
  0
};

unsigned long lastCtrlUs = 0;
unsigned long lastPrintMs = 0;

bool manualMode = false;
int manualDir = 0;

int escState = 0;


// =========================
// FUNCTION DECLARATIONS
// =========================

void imuStart();
void imuRead();
void imuZero();
bool imuReady();

bool finiteFloat(float v);
bool validQ(const imu::Quaternion& q);
float quatNorm(const imu::Quaternion& q);
imu::Quaternion unitQ(imu::Quaternion q);
imu::Quaternion zeroAgainst(const imu::Quaternion& raw, const imu::Quaternion& zero);
imu::Quaternion relativeQ(const imu::Quaternion& upper, const imu::Quaternion& forearm);
float relativeAngleMagnitudeDeg(const imu::Quaternion& q);
void printQuat(const char* label, const imu::Quaternion& q);

float angDiff(float nowAng, float zeroAng);
float errDeg(float target, float current);
float axisVal();
float axisRaw();
const char* axisName();

void waitCal();
void menu();

void updateAll();
void pid(Motor& m, float dt, bool enabled);
bool motorEnabled(Motor& m);

void setTarget(Motor& m, float targetDeg, int pwm);
void resetMotor(Motor& m);
void resetTargets();
void stopManual();
void startManual(int dir);

long counts(Motor& m);

void drive(Motor& m, int dir, int pwm);
void off(Motor& m);
void hold(Motor& m);

void serialCheck();
void handleCmd(char c);
void handleArrow(char arrowCode);
void printData();


// =========================
// SETUP / LOOP
// =========================

void setup() {
  Serial.begin(9600);
  delay(2000);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  off(m1);
  off(m2);

  enc1.write(0);
  enc2.write(0);

  imuStart();

  Serial.println();
  Serial.println("System ready.");
  Serial.println("Place the mechanism at the zero position.");
  Serial.println("Type r and press Enter to zero the IMUs and joint angle.");
  Serial.println();

  waitCal();
  menu();

  lastCtrlUs = micros();
}

void loop() {
  updateAll();
}


// =========================
// IMU FUNCTIONS
// =========================

void imuStart() {
  Serial.println("Starting two BNO055 IMUs in default fusion mode...");

  Wire.begin();
  Wire1.begin();

  imuUpperOk = bnoUpper.begin();
  if (imuUpperOk) {
    Serial.println("Upper-arm BNO055 detected on Wire.");
    delay(500);
  } else {
    Serial.println("Upper-arm BNO055 NOT detected on Wire.");
  }

  imuForearmOk = bnoForearm.begin();
  if (imuForearmOk) {
    Serial.println("Forearm BNO055 detected on Wire1.");
    delay(500);
  } else {
    Serial.println("Forearm BNO055 NOT detected on Wire1.");
  }
}

bool imuReady() {
  return imuUpperOk && imuForearmOk;
}

bool finiteFloat(float v) {
  return !isnan(v) && !isinf(v);
}

float quatNorm(const imu::Quaternion& q) {
  return sqrt(
    q.w() * q.w() +
    q.x() * q.x() +
    q.y() * q.y() +
    q.z() * q.z()
  );
}

bool validQ(const imu::Quaternion& q) {
  if (!finiteFloat(q.w()) || !finiteFloat(q.x()) ||
      !finiteFloat(q.y()) || !finiteFloat(q.z())) {
    return false;
  }

  float n = quatNorm(q);
  return finiteFloat(n) && n > 0.000001;
}

imu::Quaternion unitQ(imu::Quaternion q) {
  q.normalize();
  return q;
}

imu::Quaternion zeroAgainst(const imu::Quaternion& raw, const imu::Quaternion& zero) {
  return unitQ(zero.conjugate() * raw);
}

imu::Quaternion relativeQ(const imu::Quaternion& upper, const imu::Quaternion& forearm) {
  return unitQ(upper.conjugate() * forearm);
}

float relativeAngleMagnitudeDeg(const imu::Quaternion& q) {
  if (!validQ(q)) {
    return jointAngleDeg;
  }

  float w = constrain(q.w(), -1.0, 1.0);
  float angleRad = 2.0 * acos(fabs(w));
  float angleDeg = angleRad * 180.0 / PI;

  if (!finiteFloat(angleDeg)) {
    return jointAngleDeg;
  }

  if (angleDeg < 0.0001) {
    angleDeg = 0.0;
  }

  return angleDeg;
}

void imuRead() {
  if (!imuReady()) {
    imuLastReadValid = false;
    imuBadReadCount++;
    return;
  }

  imu::Quaternion upperRawNew = bnoUpper.getQuat();
  imu::Quaternion forearmRawNew = bnoForearm.getQuat();

  if (!validQ(upperRawNew) || !validQ(forearmRawNew)) {
    // Ignore this sample and keep the previous valid quaternion/angle values.
    imuLastReadValid = false;
    imuBadReadCount++;
    return;
  }

  upperRawNew = unitQ(upperRawNew);
  forearmRawNew = unitQ(forearmRawNew);

  imu::Quaternion upperNew = zeroAgainst(upperRawNew, qUpperZero);
  imu::Quaternion forearmNew = zeroAgainst(forearmRawNew, qForearmZero);
  imu::Quaternion relRawNew = relativeQ(upperRawNew, forearmRawNew);
  imu::Quaternion relNew = zeroAgainst(relRawNew, qRelZero);
  imu::Quaternion jointNew = relNew;

  if (!validQ(upperNew) || !validQ(forearmNew) ||
      !validQ(relNew) || !validQ(jointNew)) {
    imuLastReadValid = false;
    imuBadReadCount++;
    return;
  }

  float angleNew = relativeAngleMagnitudeDeg(jointNew);
  if (!finiteFloat(angleNew)) {
    imuLastReadValid = false;
    imuBadReadCount++;
    return;
  }

  qUpperRaw = upperRawNew;
  qForearmRaw = forearmRawNew;
  qUpper = upperNew;
  qForearm = forearmNew;
  qRel = relNew;
  qJoint = jointNew;

  jointRawDeg = angleNew;
  jointAngleDeg = angleNew;
  imuLastReadValid = true;
}

void imuZero() {
  if (!imuReady()) {
    Serial.println("Cannot zero: one or both IMUs are not ready.");
    return;
  }

  imu::Quaternion upperRawNew = bnoUpper.getQuat();
  imu::Quaternion forearmRawNew = bnoForearm.getQuat();

  if (!validQ(upperRawNew) || !validQ(forearmRawNew)) {
    Serial.println("Cannot zero: invalid IMU quaternion sample.");
    imuLastReadValid = false;
    imuBadReadCount++;
    return;
  }

  qUpperRaw = unitQ(upperRawNew);
  qForearmRaw = unitQ(forearmRawNew);

  qUpperZero = qUpperRaw;
  qForearmZero = qForearmRaw;
  qRelZero = relativeQ(qUpperRaw, qForearmRaw);

  qUpper = imu::Quaternion(1.0, 0.0, 0.0, 0.0);
  qForearm = imu::Quaternion(1.0, 0.0, 0.0, 0.0);
  qRel = imu::Quaternion(1.0, 0.0, 0.0, 0.0);
  qJoint = imu::Quaternion(1.0, 0.0, 0.0, 0.0);

  jointRawDeg = 0.0;
  jointAngleDeg = 0.0;
  imuLastReadValid = true;

  stopManual();
  resetMotor(m1);
  resetMotor(m2);
  m1.target = 0.0;
  m2.target = 0.0;
  off(m1);
  off(m2);

  Serial.println("IMUs zeroed. Joint angle and zeroed quaternion outputs are now reset.");
  menu();
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

float angDiff(float nowAng, float zeroAng) {
  float diff = nowAng - zeroAng;

  while (diff > 180.0) {
    diff -= 360.0;
  }

  while (diff < -180.0) {
    diff += 360.0;
  }

  return diff;
}

float errDeg(float target, float current) {
  return angDiff(target, current);
}

float axisVal() {
  return jointAngleDeg;
}

float axisRaw() {
  return jointRawDeg;
}

const char* axisName() {
  return "zeroed relative quaternion angle magnitude";
}


// =========================
// MENU FUNCTIONS
// =========================

void waitCal() {
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        continue;
      }

      if (c == 'r' || c == 'R') {
        imuZero();
        Serial.println("Zero complete.");
        Serial.println();
        return;
      }

      if (c == 'm' || c == 'M') {
        menu();
        continue;
      }

      Serial.println("Please type r and press Enter to zero first.");
    }
  }
}

void menu() {
  Serial.println();
  Serial.println("Serial control mode is ON.");
  Serial.print("Active measurement: ");
  Serial.println(axisName());
  Serial.println("0-9 = target from 0 to 90 degrees");
  Serial.println("Left arrow = manual Motor 2 reverse");
  Serial.println("Right arrow = manual Motor 2 forward");
  Serial.println("s = stop both motors");
  Serial.println("r = reset");
  Serial.println("m = print menu");
  Serial.println();
}


// =========================
// PID CONTROL
// =========================

void updateAll() {
  serialCheck();

  unsigned long now = micros();

  if (now - lastCtrlUs < CTRL_US) {
    return;
  }

  float dt = (now - lastCtrlUs) / 1000000.0;
  lastCtrlUs = now;

  if (dt <= 0) {
    dt = 0.001;
  }

  imuRead();

  if (manualMode) {
    off(m1);
    drive(m2, manualDir * m2.motSign, Man_speed);
    printData();
    return;
  }

  pid(m1, dt, motorEnabled(m1));
  pid(m2, dt, motorEnabled(m2));

  if (m1.active || m2.active) {
    printData();
  }
}

bool motorEnabled(Motor& m) {
  if (&m == &m1) {
    return false;
  }

  return true;
}

void pid(Motor& m, float dt, bool enabled) {
  if (!imuReady() || !enabled) {
    off(m);
    resetMotor(m);
    return;
  }

  // No command yet: motor fully off.
  if (!m.active) {
    off(m);
    return;
  }

  float error = errDeg(m.target, axisVal());
  float absErr = fabs(error);

  // At target: active hold/brake.
  if (absErr <= m.tol) {
    hold(m);

    m.sumErr = 0.0;
    m.lastErr = error;
    m.holding = true;

    // Refresh timeout while holding.
    m.startMs = millis();

    if (!m.printed) {
      Serial.println();
      Serial.print(m.name);
      Serial.println(" reached target. PID hold is active.");

      Serial.print("Measurement: ");
      Serial.print(axisName());

      Serial.print(" | TargetDeg: ");
      Serial.print(m.target, 2);

      Serial.print(" | CurrentDeg: ");
      Serial.print(axisVal(), 2);

      Serial.print(" | ErrorDeg: ");
      Serial.print(error, 2);

      Serial.print(" | M1Counts: ");
      Serial.print(counts(m1));

      Serial.print(" | M2Counts: ");
      Serial.println(counts(m2));

      Serial.println("If the mechanism is pushed away, PID will correct it.");
      Serial.println();

      m.printed = true;
    }

    return;
  }

  // If it was holding and got pushed away, restart correction.
  if (m.holding) {
    m.holding = false;
    m.printed = false;
    m.sumErr = 0.0;
    m.lastErr = error;
    m.startMs = millis();

    Serial.println();
    Serial.print(m.name);
    Serial.println(" moved away from target. PID is correcting.");
    Serial.println();
  }

  // Timeout: fully off, not hold.
  // TIMEOUT TEMP COMMENTED OUT UNCOMMENT IF STATEMENT TO ENABLE
  // if (millis() - m.startMs > m.timeout) {
  //   off(m);
  //   resetMotor(m);

  //   Serial.println();
  //   Serial.print(m.name);
  //   Serial.println(" move timed out. Motor is fully OFF.");

  //   Serial.print("Measurement: ");
  //   Serial.print(axisName());

  //   Serial.print(" | TargetDeg: ");
  //   Serial.print(m.target, 2);

  //   Serial.print(" | CurrentDeg: ");
  //   Serial.print(axisVal(), 2);

  //   Serial.print(" | ErrorDeg: ");
  //   Serial.println(errDeg(m.target, axisVal()), 2);

  //   Serial.println("Choose another target or zero with r.");
  //   Serial.println();

  //   return;
  // }

  float dErr = (error - m.lastErr) / dt;

  m.sumErr += error * dt;
  m.sumErr = constrain(m.sumErr, -100.0, 100.0);

  float out = m.kp * error
            + m.kd * dErr
            + m.ki * m.sumErr;

  int pwm = abs((int)out);

  int pwmLimit = m.maxPwm;

  if (absErr <= m.slowZone) {
    pwmLimit = m.slowPwm;
  }

  pwm = constrain(pwm, 0, pwmLimit);

  if (pwm > 0 && pwm < m.minPwm) {
    pwm = m.minPwm;
  }

  int dir = FWD;

  if (out < 0) {
    dir = REV;
  }

  dir *= m.motSign;

  drive(m, dir, pwm);

  m.lastErr = error;
}


// =========================
// MOTOR STATE
// =========================

void setTarget(Motor& m, float targetDeg, int pwm) {
  imuRead();

  m.target = targetDeg;
  m.maxPwm = constrain(pwm, MIN_PWM, MAX_PWM);

  m.sumErr = 0.0;
  m.lastErr = errDeg(m.target, axisVal());

  m.active = true;
  m.holding = false;
  m.printed = false;
  m.startMs = millis();

  Serial.println();
  Serial.print("Moving ");
  Serial.print(m.name);
  Serial.print(" to ");
  Serial.print(m.target, 2);
  Serial.print(" degrees using ");
  Serial.print(axisName());
  Serial.println(".");
  Serial.println();
}

void resetMotor(Motor& m) {
  m.active = false;
  m.holding = false;
  m.printed = false;
  m.sumErr = 0.0;
  m.lastErr = 0.0;
}

void resetTargets() {
  stopManual();
  imuRead();

  m1.target = axisVal();
  m2.target = axisVal();

  resetMotor(m1);
  resetMotor(m2);

  off(m1);
  off(m2);
}

void stopManual() {
  manualMode = false;
  manualDir = 0;
}

void startManual(int dir) {
  resetMotor(m1);
  resetMotor(m2);
  off(m1);

  manualMode = true;
  manualDir = dir;

  Serial.println();
  Serial.print("Manual mode: Motor 2 turning ");
  Serial.println(dir == FWD ? "forward." : "reverse.");
  Serial.println("Press s to stop, r to zero, or 0-9 to return to PID target mode.");
  Serial.println();
}

long counts(Motor& m) {
  return m.enc->read() * m.encSign;
}


// =========================
// MOTOR DRIVER
// =========================

void drive(Motor& m, int dir, int pwm) {
  pwm = constrain(pwm, 0, 255);

  if (dir == FWD) {
    analogWrite(m.in1, pwm);
    analogWrite(m.in2, 0);
  }
  else if (dir == REV) {
    analogWrite(m.in1, 0);
    analogWrite(m.in2, pwm);
  }
  else {
    off(m);
  }
}

// Fully off / coast.
// Both IN pins are LOW.
void off(Motor& m) {
  analogWrite(m.in1, 0);
  analogWrite(m.in2, 0);

  digitalWrite(m.in1, LOW);
  digitalWrite(m.in2, LOW);
}

// Active hold / brake.
// Both IN pins are HIGH.
void hold(Motor& m) {
  analogWrite(m.in1, 255);
  analogWrite(m.in2, 255);
}


// =========================
// SERIAL INPUT / OUTPUT
// =========================

void serialCheck() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (escState == 0 && c == 27) {
      escState = 1;
      continue;
    }

    if (escState == 1) {
      escState = (c == '[') ? 2 : 0;
      continue;
    }

    if (escState == 2) {
      handleArrow(c);
      escState = 0;
      continue;
    }

    if (c == '\n' || c == '\r') {
      continue;
    }

    handleCmd(c);
  }
}

void handleArrow(char arrowCode) {
  if (arrowCode == 'D') {       // left arrow
    startManual(REV);
  }
  else if (arrowCode == 'C') {  // right arrow
    startManual(FWD);
  }
}

void handleCmd(char c) {
  if (c >= '0' && c <= '9') {
    int key = c - '0';
    float target = KEY_TARGETS[key];

    stopManual();
    imuRead();

    off(m1);
    resetMotor(m1);
    m1.target = axisVal();

    setTarget(m2, target, CMD_PWM);

    Serial.println("Joint-angle PID mode active: Motor 2 is controlling the quaternion-derived joint angle. Motor 1 is stopped.");
    Serial.println();
    return;
  }

  if (c == 'r' || c == 'R') {
    imuZero();
    return;
  }

  if (c == 's' || c == 'S') {
    resetTargets();

    Serial.println();
    Serial.print("Motor 1 and Motor 2 stopped. Current ");
    Serial.print(axisName());
    Serial.print(": ");
    Serial.println(axisVal(), 2);
    Serial.println("Choose another target.");
    Serial.println();
    return;
  }

  if (c == 'm' || c == 'M') {
    menu();
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(c);
}

void printData() {
  if (millis() - lastPrintMs >= PRINT_MS) {
    lastPrintMs = millis();

    float targetDeg = manualMode ? axisVal() : m2.target;
    float currentDeg = axisVal();
    float errorDeg = manualMode ? 0.0 : errDeg(targetDeg, currentDeg);





    Serial.print("\nTargetDeg: ");
    Serial.print(targetDeg, 2);

    Serial.print(" | CurrentDeg: ");
    Serial.print(currentDeg, 2);

    Serial.print(" | ErrorDeg: ");
    Serial.print(errorDeg, 2);

    Serial.print("\nM1Counts: ");
    Serial.print(counts(m1));

    Serial.print(" | M2Counts: ");
    Serial.println(counts(m2));

    printQuat("qUpperZeroed", qUpper);
    printQuat("qForearmZeroed", qForearm);
    printQuat("qJointZeroed", qJoint);
  }
}
