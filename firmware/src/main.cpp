#include <Arduino.h>
#include <math.h>
#include <ctype.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ======================================================
// Two-BNO055 IMU + Motor 2 PID Control + PWMNorm Output
// ======================================================
//
// Keyboard commands through Python visualizer:
//   0-9        target angle = digit * 10 degrees
//   left       manual motor reverse
//   right      manual motor forward
//   p or s     stop motor
//   r          recalibrate / zero IMUs
//   m          print menu
//
// Important:
//   Keep only this file active in src/main.cpp.
//   Do not keep another .cpp with setup() and loop() in src.

// =====================
// IMU setup
// =====================

// Upper arm IMU on Wire
Adafruit_BNO055 bnoUpper(0, 0x28, &Wire);

// Forearm IMU on Wire1
Adafruit_BNO055 bnoForearm(1, 0x28, &Wire1);

// Simple quaternion struct so we do not depend too much on imu::Quaternion math.
struct Quat {
  float w;
  float x;
  float y;
  float z;
};

Quat qUpperRaw     = {1.0, 0.0, 0.0, 0.0};
Quat qForearmRaw   = {1.0, 0.0, 0.0, 0.0};

Quat qUpperZero    = {1.0, 0.0, 0.0, 0.0};
Quat qForearmZero  = {1.0, 0.0, 0.0, 0.0};

Quat qUpperZeroed   = {1.0, 0.0, 0.0, 0.0};
Quat qForearmZeroed = {1.0, 0.0, 0.0, 0.0};
Quat qJointZeroed   = {1.0, 0.0, 0.0, 0.0};

float jointAngleDeg = 0.0;
bool imuOk = false;

// =====================
// Motor pins
// =====================

// Motor 1 pins
const int M1_IN1 = 4;
const int M1_IN2 = 5;
const int M1_ENC_A = 30;
const int M1_ENC_B = 31;

// Motor 2 pins
const int M2_IN1 = 2;
const int M2_IN2 = 3;
const int M2_ENC_A = 28;
const int M2_ENC_B = 29;

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);

// =====================
// General constants
// =====================

const int FWD = 1;
const int REV = -1;

const int M1_ENC_SIGN = 1;
const int M2_ENC_SIGN = 1;

const int M1_MOT_SIGN = 1;
const int M2_MOT_SIGN = 1;

// If roll direction is backwards, flip these.
const int M1_ROLL_SIGN = 1;
const int M2_ROLL_SIGN = -1;

// Manual test speed
const int Man_speed = 125;

// Control timing
const unsigned long CTRL_US = 10000;   // 10 ms control loop
const unsigned long PRINT_MS = 200;    // print every 200 ms

// Keyboard targets
const float KEY_TARGETS[10] = {
  0.0, 10.0, 20.0, 30.0, 40.0,
  50.0, 60.0, 70.0, 80.0, 90.0
};

// =====================
// Motor struct
// =====================

struct Motor {
  const char* name;

  int in1;
  int in2;
  Encoder* enc;

  int encSign;
  int motSign;
  int rollSign;

  // PID gains
  float kp;
  float kd;
  float ki;

  // uFull is the PID output magnitude that equals full normalized command.
  // Smaller uFull = reaches max PWM faster.
  // Larger uFull = smoother/weaker response.
  float uFull;

  // PWM limits
  int minPwm;
  int maxPwm;
  int slowPwm;

  // Angle control settings
  float tol;
  float slowZone;
  unsigned long timeout;

  // PID state
  float target;
  float lastErr;
  float sumErr;
  float lastMeas;
  float dErrFilt;
  float lastOut;

  // Values for visualizer
  float lastUNorm;
  int lastPwm;

  bool active;
  bool holding;
  bool printed;

  unsigned long startMs;
};

// Motor 1 exists, but right now it is disabled in motorEnabled().
Motor m1 = {
  "M1",

  M1_IN1, M1_IN2, &enc1,

  M1_ENC_SIGN,
  M1_MOT_SIGN,
  M1_ROLL_SIGN,

  // kp, kd, ki
  1.75, 0.0, 0.125,

  // uFull
  100.0,

  // minPwm, maxPwm, slowPwm
  150, 225, 150,

  // tol, slowZone, timeout
  1.0, 5.0, 10000,

  // target, lastErr, sumErr
  0.0, 0.0, 0.0,

  // lastMeas, dErrFilt, lastOut
  0.0, 0.0, 0.0,

  // lastUNorm, lastPwm
  0.0, 0,

  // active, holding, printed
  false, false, false,

  // startMs
  0
};

// Motor 2 is the active motor.
Motor m2 = {
  "M2",

  M2_IN1, M2_IN2, &enc2,

  M2_ENC_SIGN,
  M2_MOT_SIGN,
  M2_ROLL_SIGN,

  // kp, kd, ki
  1.75, 0.0, 0.125,

  // uFull
  100.0,

  // minPwm, maxPwm, slowPwm
  150, 225, 150,

  // tol, slowZone, timeout
  1.0, 5.0, 10000,

  // target, lastErr, sumErr
  0.0, 0.0, 0.0,

  // lastMeas, dErrFilt, lastOut
  0.0, 0.0, 0.0,

  // lastUNorm, lastPwm
  0.0, 0,

  // active, holding, printed
  false, false, false,

  // startMs
  0
};

// =====================
// Manual mode state
// =====================

bool manualMode = false;
int manualDir = 0;

// Used to detect escape sequences from arrow keys.
int escState = 0;

// =====================
// Timing state
// =====================

unsigned long lastCtrlUs = 0;
unsigned long lastPrintMs = 0;

// =====================
// Function declarations
// =====================

Quat normalizeQuat(Quat q);
Quat quatConjugate(Quat q);
Quat quatMultiply(Quat a, Quat b);
Quat quatFromBNO(imu::Quaternion q);
float quatAngleDeg(Quat q);

bool imuStart();
bool imuReady();
void imuRead();
void imuZero();

float axisVal();
float axisRaw();
float errDeg(float target, float current);
float angDiff(float target, float current);

long counts(Motor& m);

void drive(Motor& m, int dir, int pwm);
void off(Motor& m);
void hold(Motor& m);

float sat1(float x);
void pid(Motor& m, float dt, bool enabled);

bool motorEnabled(Motor& m);
void resetMotor(Motor& m);
void resetTargets();
void setTarget(float deg);

void serialCheck();
void handleChar(char c);
void handleArrow(char c);

void updateAll();
void printData();
void menu();

// =====================
// Quaternion math
// =====================

Quat normalizeQuat(Quat q) {
  float n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

  if (n < 0.000001) {
    return {1.0, 0.0, 0.0, 0.0};
  }

  q.w /= n;
  q.x /= n;
  q.y /= n;
  q.z /= n;

  return q;
}

Quat quatConjugate(Quat q) {
  q = normalizeQuat(q);

  return {
    q.w,
    -q.x,
    -q.y,
    -q.z
  };
}

Quat quatMultiply(Quat a, Quat b) {
  Quat q;

  q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

  return normalizeQuat(q);
}

Quat quatFromBNO(imu::Quaternion q) {
  Quat out = {
    (float)q.w(),
    (float)q.x(),
    (float)q.y(),
    (float)q.z()
  };

  return normalizeQuat(out);
}

float quatAngleDeg(Quat q) {
  q = normalizeQuat(q);

  float w = fabs(q.w);
  w = constrain(w, -1.0, 1.0);

  return 2.0 * acos(w) * 180.0 / PI;
}

// =====================
// IMU functions
// =====================

bool imuStart() {
  Wire.begin();
  Wire1.begin();

  delay(100);

  bool upperOk = bnoUpper.begin();
  bool forearmOk = bnoForearm.begin();

  if (!upperOk) {
    Serial.println("ERROR: Upper BNO055 not detected.");
  }

  if (!forearmOk) {
    Serial.println("ERROR: Forearm BNO055 not detected.");
  }

  if (!upperOk || !forearmOk) {
    imuOk = false;
    return false;
  }

  delay(1000);

  bnoUpper.setExtCrystalUse(true);
  bnoForearm.setExtCrystalUse(true);

  imuOk = true;

  imuRead();
  imuZero();

  Serial.println("IMUs started and zeroed.");

  return true;
}

bool imuReady() {
  return imuOk;
}

void imuRead() {
  if (!imuOk) {
    return;
  }

  qUpperRaw = quatFromBNO(bnoUpper.getQuat());
  qForearmRaw = quatFromBNO(bnoForearm.getQuat());

  // Zeroed orientation:
  // qZeroed = conjugate(qZero) * qCurrent
  qUpperZeroed = quatMultiply(quatConjugate(qUpperZero), qUpperRaw);
  qForearmZeroed = quatMultiply(quatConjugate(qForearmZero), qForearmRaw);

  // Relative joint quaternion:
  // qJoint = conjugate(qUpperZeroed) * qForearmZeroed
  qJointZeroed = quatMultiply(quatConjugate(qUpperZeroed), qForearmZeroed);

  // Joint angle magnitude in degrees.
  jointAngleDeg = quatAngleDeg(qJointZeroed);
}

void imuZero() {
  if (!imuOk) {
    Serial.println("Cannot zero IMUs. IMU not ready.");
    return;
  }

  qUpperRaw = quatFromBNO(bnoUpper.getQuat());
  qForearmRaw = quatFromBNO(bnoForearm.getQuat());

  qUpperZero = qUpperRaw;
  qForearmZero = qForearmRaw;

  qUpperZeroed = {1.0, 0.0, 0.0, 0.0};
  qForearmZeroed = {1.0, 0.0, 0.0, 0.0};
  qJointZeroed = {1.0, 0.0, 0.0, 0.0};

  jointAngleDeg = 0.0;

  resetTargets();

  Serial.println("IMU recalibrated. Current joint angle is now 0.");
}

// =====================
// Angle helpers
// =====================

float axisVal() {
  // Right now the control angle is the relative joint angle.
  return jointAngleDeg;
}

float axisRaw() {
  return jointAngleDeg;
}

float angDiff(float target, float current) {
  // For this elbow angle, we are using normal subtraction.
  // If later you use yaw from 0-360, then wrapping may be needed.
  return target - current;
}

float errDeg(float target, float current) {
  return angDiff(target, current);
}

// =====================
// Motor helpers
// =====================

long counts(Motor& m) {
  return m.encSign * m.enc->read();
}

void drive(Motor& m, int dir, int pwm) {
  pwm = constrain(pwm, 0, 255);

  if (pwm <= 0 || dir == 0) {
    off(m);
    return;
  }

  if (dir > 0) {
    analogWrite(m.in1, pwm);
    analogWrite(m.in2, 0);
  } else {
    analogWrite(m.in1, 0);
    analogWrite(m.in2, pwm);
  }
}

void off(Motor& m) {
  analogWrite(m.in1, 0);
  analogWrite(m.in2, 0);
}

void hold(Motor& m) {
  // Brake/hold by driving both inputs high.
  // This is separate from active PID drive.
  analogWrite(m.in1, 255);
  analogWrite(m.in2, 255);
}

float sat1(float x) {
  if (x < 0.0) {
    return 0.0;
  }

  if (x > 1.0) {
    return 1.0;
  }

  return x;
}

// =====================
// PID controller
// =====================

void pid(Motor& m, float dt, bool enabled) {
  if (!imuReady() || !enabled) {
    off(m);
    resetMotor(m);
    return;
  }

  if (!m.active) {
    off(m);
    m.lastUNorm = 0.0;
    m.lastPwm = 0;
    return;
  }

  float current = axisVal();
  float error = errDeg(m.target, current);
  float absErr = fabs(error);

  // Hysteresis:
  // Enter hold at m.tol.
  // Leave hold only when error grows larger than 1.5 * m.tol.
  float exitTol = m.tol * 1.5;
  bool atTarget = m.holding ? (absErr <= exitTol) : (absErr <= m.tol);

  if (atTarget) {
    hold(m);

    // For the visualizer:
    // This means active PID drive is 0 while we are holding.
    // The driver is braking, but PID is not commanding forward/reverse motion.
    m.lastUNorm = 0.0;
    m.lastPwm = 0;

    m.sumErr = 0.0;
    m.lastErr = error;
    m.lastMeas = current;
    m.dErrFilt = 0.0;
    m.lastOut = 0.0;

    m.holding = true;
    m.startMs = millis();

    if (!m.printed) {
      Serial.print(m.name);
      Serial.println(" reached target.");
      m.printed = true;
    }

    return;
  }

  // If we were holding and got pushed away, restart PID cleanly.
  if (m.holding) {
    m.holding = false;
    m.printed = false;

    m.sumErr = 0.0;
    m.lastErr = error;
    m.lastMeas = current;
    m.dErrFilt = 0.0;
    m.lastOut = 0.0;
    m.startMs = millis();

    Serial.print(m.name);
    Serial.println(" moved away from target. PID re-engaging.");
  }

  // Timeout protection
  if (millis() - m.startMs > m.timeout) {
    Serial.print(m.name);
    Serial.println(" timeout. Motor stopped.");

    off(m);
    resetMotor(m);
    return;
  }

  // Slow zone limits the max PWM near the target.
  int pwmLimit = m.maxPwm;

  if (absErr <= m.slowZone) {
    pwmLimit = m.slowPwm;
  }

  // Derivative on measurement:
  // This avoids a derivative kick when target changes.
  float dMeas = (current - m.lastMeas) / dt;
  float dErrRaw = -dMeas;

  // Low-pass filter on derivative.
  m.dErrFilt = 0.2 * dErrRaw + 0.8 * m.dErrFilt;
  float dErr = m.dErrFilt;

  // Conditional integration anti-windup.
  // If output is already saturated and error is pushing farther into saturation,
  // do not keep growing the integral.
  bool saturated = (fabs(m.lastOut) >= fabs(m.uFull)) &&
                   ((m.lastOut > 0.0) == (error > 0.0));

  if (!saturated) {
    m.sumErr += error * dt;
  }

  // Integral clamp based on uFull and ki.
  if (fabs(m.ki) > 0.000001) {
    float iLimit = fabs(m.uFull / m.ki);
    m.sumErr = constrain(m.sumErr, -iLimit, iLimit);
  } else {
    m.sumErr = 0.0;
  }

  // PID output before PWM conversion.
  float out = m.kp * error + m.kd * dErr + m.ki * m.sumErr;

  m.lastErr = error;
  m.lastMeas = current;
  m.lastOut = out;

  // ===============================
  // Normalized PWM mapping
  // ===============================
  //
  // uNorm = sat1(abs(uPID) / uFull)
  //
  // PWM = PWMmin + (PWMmax - PWMmin) * uNorm
  //
  // uNorm is what we graph in Python.
  // It is between 0.0 and 1.0.

  float uPID = out;

  float uFullNow = fabs(m.uFull);

  if (uFullNow < 0.000001) {
    uFullNow = 1.0;
  }

  float uNorm = sat1(fabs(uPID) / uFullNow);

  int pwmMaxNow = pwmLimit;
  int pwmMinNow = min(m.minPwm, pwmMaxNow);

  int pwm = pwmMinNow + (int)((pwmMaxNow - pwmMinNow) * uNorm);

  if (fabs(uPID) < 0.0001) {
    pwm = 0;
    uNorm = 0.0;
  }

  // Store values so printData() can send them to the visualizer.
  m.lastUNorm = uNorm;
  m.lastPwm = pwm;

  int dir = (uPID >= 0.0) ? FWD : REV;

  // Apply motor direction sign.
  dir *= m.motSign;

  drive(m, dir, pwm);
}

// =====================
// Target and reset functions
// =====================

bool motorEnabled(Motor& m) {
  // Motor 1 is disabled for now.
  if (&m == &m1) {
    return false;
  }

  // Motor 2 controls the elbow joint.
  if (&m == &m2) {
    return true;
  }

  return false;
}

void resetMotor(Motor& m) {
  m.active = false;
  m.holding = false;
  m.printed = false;

  m.sumErr = 0.0;
  m.lastErr = 0.0;
  m.lastMeas = axisVal();
  m.dErrFilt = 0.0;
  m.lastOut = 0.0;

  m.lastUNorm = 0.0;
  m.lastPwm = 0;
}

void resetTargets() {
  resetMotor(m1);
  resetMotor(m2);

  m1.target = axisVal();
  m2.target = axisVal();

  manualMode = false;
  manualDir = 0;

  off(m1);
  off(m2);
}

void setTarget(float deg) {
  manualMode = false;
  manualDir = 0;

  float current = axisVal();

  m2.target = deg;
  m2.active = true;
  m2.holding = false;
  m2.printed = false;

  m2.sumErr = 0.0;
  m2.lastErr = errDeg(m2.target, current);
  m2.lastMeas = current;
  m2.dErrFilt = 0.0;
  m2.lastOut = 0.0;
  m2.lastUNorm = 0.0;
  m2.lastPwm = 0;
  m2.startMs = millis();

  Serial.print("New Motor 2 target: ");
  Serial.print(deg, 2);
  Serial.println(" deg");
}

// =====================
// Serial command handling
// =====================

void serialCheck() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Handle arrow key escape sequence:
    // left  = ESC [ D
    // right = ESC [ C
    if (escState == 0) {
      if (c == 27) {
        escState = 1;
      } else {
        handleChar(c);
      }
    } else if (escState == 1) {
      if (c == '[') {
        escState = 2;
      } else {
        escState = 0;
      }
    } else if (escState == 2) {
      handleArrow(c);
      escState = 0;
    }
  }
}

void handleChar(char c) {
  if (c == '\n' || c == '\r') {
    return;
  }

  if (isdigit(c)) {
    int digit = c - '0';
    setTarget(KEY_TARGETS[digit]);
    return;
  }

  if (c == 's' || c == 'p') {
    Serial.println("Stop command received.");

    manualMode = false;
    manualDir = 0;

    resetTargets();

    return;
  }

  if (c == 'r') {
    Serial.println("Recalibrating IMUs...");
    imuZero();
    return;
  }

  if (c == 'm') {
    menu();
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(c);
}

void handleArrow(char c) {
  if (c == 'D') {
    // Left arrow = manual reverse
    manualMode = true;
    manualDir = REV;

    m2.active = false;
    m2.holding = false;
    m2.printed = false;

    Serial.println("Manual mode: Motor 2 reverse");
  } else if (c == 'C') {
    // Right arrow = manual forward
    manualMode = true;
    manualDir = FWD;

    m2.active = false;
    m2.holding = false;
    m2.printed = false;

    Serial.println("Manual mode: Motor 2 forward");
  }
}

// =====================
// Main update function
// =====================

void updateAll() {
  serialCheck();

  unsigned long nowUs = micros();

  if (nowUs - lastCtrlUs < CTRL_US) {
    return;
  }

  float dt = (nowUs - lastCtrlUs) / 1000000.0;

  lastCtrlUs = nowUs;

  if (dt <= 0.0) {
    dt = CTRL_US / 1000000.0;
  }

  // Prevent a huge dt from messing up derivative if the loop pauses.
  if (dt > 0.05) {
    dt = 0.05;
  }

  imuRead();

  static bool wasManual = false;

  if (manualMode) {
    off(m1);

    int manualPwm = Man_speed;

    drive(m2, manualDir * m2.motSign, manualPwm);

    // This is the important fix:
    // Store manual PWM so the visualizer does not stay at 0.
    m2.lastPwm = manualPwm;
    m2.lastUNorm = (float)manualPwm / (float)m2.maxPwm;

    if (m2.lastUNorm > 1.0) {
      m2.lastUNorm = 1.0;
    }

    printData();

    wasManual = true;
    return;
  }

  // Bumpless transfer from manual mode back to PID mode.
  if (wasManual) {
    float current = axisVal();

    m2.sumErr = 0.0;
    m2.lastErr = errDeg(m2.target, current);
    m2.lastMeas = current;
    m2.dErrFilt = 0.0;
    m2.lastOut = 0.0;
    m2.lastUNorm = 0.0;
    m2.lastPwm = 0;
    m2.holding = false;
    m2.printed = false;
    m2.startMs = millis();

    wasManual = false;
  }

  pid(m1, dt, motorEnabled(m1));
  pid(m2, dt, motorEnabled(m2));

  printData();
}

// =====================
// Printing for visualizer
// =====================

void printQuat(const char* label, Quat q) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print(q.w, 6);
  Serial.print(", ");
  Serial.print(q.x, 6);
  Serial.print(", ");
  Serial.print(q.y, 6);
  Serial.print(", ");
  Serial.println(q.z, 6);
}

void printData() {
  if (millis() - lastPrintMs < PRINT_MS) {
    return;
  }

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

  Serial.print(" | PWMNorm: ");
  Serial.print(m2.lastUNorm, 3);

  Serial.print(" | PWM: ");
  Serial.print(m2.lastPwm);

  Serial.print(" | Mode: ");

  if (manualMode) {
    Serial.print("Manual");
  } else if (m2.active) {
    Serial.print("PID");
  } else {
    Serial.print("Idle");
  }

  Serial.print(" | M1Counts: ");
  Serial.print(counts(m1));

  Serial.print(" | M2Counts: ");
  Serial.println(counts(m2));

  printQuat("qUpperZeroed", qUpperZeroed);
  printQuat("qForearmZeroed", qForearmZeroed);
  printQuat("qJointZeroed", qJointZeroed);
}

// =====================
// Menu
// =====================

void menu() {
  Serial.println();
  Serial.println("========== MENU ==========");
  Serial.println("0-9  : Move Motor 2 to digit * 10 degrees");
  Serial.println("Left : Manual Motor 2 reverse");
  Serial.println("Right: Manual Motor 2 forward");
  Serial.println("s/p  : Stop motor");
  Serial.println("r    : Recalibrate / zero IMUs");
  Serial.println("m    : Print menu");
  Serial.println("==========================");
  Serial.println();
}

// =====================
// Setup and loop
// =====================

void setup() {
  Serial.begin(115200);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  off(m1);
  off(m2);

  delay(1500);

  Serial.println("Starting dual IMU elbow controller...");

  if (!imuStart()) {
    Serial.println("IMU startup failed. Check wiring.");
  }

  resetTargets();

  lastCtrlUs = micros();
  lastPrintMs = millis();

  menu();
}

void loop() {
  updateAll();
}