#include <Arduino.h>
#include <math.h>
#include <ctype.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ======================================================
// Dual BNO055 IMU + Motor 2 PID Control
// IMU spike rejection + low-pass filtering
// Sinusoidal trajectory mode
// Correct UCmd = signed PWM command for graphing/system ID
// ======================================================

// ---------------------
// IMU setup
// ---------------------

Adafruit_BNO055 bnoUpper(0, 0x28, &Wire);
Adafruit_BNO055 bnoForearm(1, 0x28, &Wire1);

struct Quat {
  float w;
  float x;
  float y;
  float z;
};

Quat qUpRaw = {1.0, 0.0, 0.0, 0.0};
Quat qForeRaw = {1.0, 0.0, 0.0, 0.0};

Quat qUpZero = {1.0, 0.0, 0.0, 0.0};
Quat qForeZero = {1.0, 0.0, 0.0, 0.0};

Quat qUpZeroed = {1.0, 0.0, 0.0, 0.0};
Quat qForeZeroed = {1.0, 0.0, 0.0, 0.0};
Quat qJointZeroed = {1.0, 0.0, 0.0, 0.0};

float ang = 0.0;
float rawAng = 0.0;

bool imuOk = false;

// ---------------------
// IMU filter
// ---------------------

float filtAng = 0.0;
float avgAng = 0.0;
float varAng = 0.0;
float goodAng = 0.0;

bool filtOn = false;
unsigned long rejSpks = 0;

const float FILT_A = 0.25;
const float AVG_A = 0.10;
const float VAR_A = 0.10;

const float JMP_MAX = 25.0;
const float VAR_MAX = 100.0;

// ---------------------
// Motor pins
// ---------------------

const int M1_IN1 = 4;
const int M1_IN2 = 5;
const int M1_ENC_A = 30;
const int M1_ENC_B = 31;

const int M2_IN1 = 2;
const int M2_IN2 = 3;
const int M2_ENC_A = 28;
const int M2_ENC_B = 29;

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);

// ---------------------
// Constants
// ---------------------

const int FWD = 1;
const int REV = -1;

const int M1_ENC_SIGN = 1;
const int M2_ENC_SIGN = 1;

const int M1_MOT_SIGN = 1;
const int M2_MOT_SIGN = 1;

const int MAN_PWM = 125;

const unsigned long CTRL_US = 10000;
const unsigned long PRINT_MS = 200;

const float KEY_TGTS[10] = {
  5.0, 10.0, 20.0, 30.0, 40.0,
  50.0, 60.0, 70.0, 80.0, 90.0
};

// ---------------------
// Trajectory settings
// ---------------------

bool trajOn = false;

// Change this between 0.05 and 0.10 for your real testing.
// 0.05 Hz = 20 second period.
// 0.10 Hz = 10 second period.
const float TRAJ_FREQ = 0.05;

const float TRAJ_CENTER_DEG = 45.0;
const float TRAJ_AMP_DEG = 45.0;

unsigned long trajStartMs = 0;

// ---------------------
// Motor struct
// ---------------------

struct Motor {
  const char* name;

  int in1;
  int in2;
  Encoder* enc;

  int encSign;
  int motSign;

  float kp;
  float kd;
  float ki;
  float uFull;

  int minPwm;
  int maxPwm;
  int slowPwm;

  float tol;
  float slowZone;
  unsigned long timeout;

  float target;
  float lastErr;
  float sumErr;
  float lastMeas;
  float lastOut;

  float lastUNorm;
  int lastPwm;
  float lastU;

  bool active;
  bool holding;
  bool printed;

  unsigned long startMs;
};

Motor m1 = {
  "M1",
  M1_IN1, M1_IN2, &enc1,
  M1_ENC_SIGN, M1_MOT_SIGN,

  0.56, 0.0, 0.005,
  20.0,

  150, 255, 150,
  1.0, 10.0, 15000,

  0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0, 0.0,

  false, false, false,
  0
};

Motor m2 = {
  "M2",
  M2_IN1, M2_IN2, &enc2,
  M2_ENC_SIGN, M2_MOT_SIGN,

  0.56, 0.0, 0.005,
  20.0,

  150, 255, 150,
  1.0, 10.0, 15000,

  0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0, 0.0,

  false, false, false,
  0
};

// ---------------------
// Runtime state
// ---------------------

bool manMode = false;
int manDir = 0;
int escState = 0;

unsigned long lastCtrlUs = 0;
unsigned long lastPrintMs = 0;

// ======================================================
// Function declarations
// ======================================================

Quat normQ(Quat q);
Quat conjQ(Quat q);
Quat mulQ(Quat a, Quat b);
Quat fromBno(imu::Quaternion q);
float angleQ(Quat q);

float filtJoint(float raw);
void resetFilt(float start);

bool imuStart();
bool imuReady();
void imuRead();
void imuZero();

float axisVal();
float errDeg(float target, float current);

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

float calcTraj(float t);
void startTraj();
void stopTraj();
void updateTraj();

void serialCheck();
void handleChar(char c);
void handleArrow(char c);

void updateAll();
void printQuat(const char* label, Quat q);
void printData();
void menu();

// ======================================================
// Quaternion math
// ======================================================

Quat normQ(Quat q) {
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

Quat conjQ(Quat q) {
  q = normQ(q);
  return {q.w, -q.x, -q.y, -q.z};
}

Quat mulQ(Quat a, Quat b) {
  Quat q;

  q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

  return normQ(q);
}

Quat fromBno(imu::Quaternion q) {
  Quat out = {
    (float)q.w(),
    (float)q.x(),
    (float)q.y(),
    (float)q.z()
  };

  return normQ(out);
}

float angleQ(Quat q) {
  q = normQ(q);

  float w = fabs(q.w);
  w = constrain(w, -1.0, 1.0);

  return 2.0 * acos(w) * 180.0 / PI;
}

// ======================================================
// IMU filter
// ======================================================

float filtJoint(float raw) {
  if (!filtOn) {
    resetFilt(raw);
    return raw;
  }

  float jump = fabs(raw - filtAng);
  float dAvg = raw - avgAng;
  float instVar = dAvg * dAvg;

  float vLim = varAng * 3.0;

  if (vLim < VAR_MAX) {
    vLim = VAR_MAX;
  }

  bool spike = (jump > JMP_MAX) && (instVar > vLim);

  if (spike) {
    rejSpks++;
    filtAng = goodAng;
    return filtAng;
  }

  avgAng = AVG_A * raw + (1.0 - AVG_A) * avgAng;

  float dNew = raw - avgAng;
  float newVar = dNew * dNew;

  varAng = VAR_A * newVar + (1.0 - VAR_A) * varAng;

  filtAng = FILT_A * raw + (1.0 - FILT_A) * filtAng;
  goodAng = filtAng;

  return filtAng;
}

void resetFilt(float start) {
  rawAng = start;
  filtAng = start;
  avgAng = start;
  varAng = 0.0;
  goodAng = start;
  filtOn = true;
  rejSpks = 0;
}

// ======================================================
// IMU functions
// ======================================================

bool imuStart() {
  Wire.begin();
  Wire1.begin();

  delay(100);

  bool upOk = bnoUpper.begin();
  bool foreOk = bnoForearm.begin();

  if (!upOk) {
    Serial.println("ERROR: Upper BNO055 not detected.");
  }

  if (!foreOk) {
    Serial.println("ERROR: Forearm BNO055 not detected.");
  }

  if (!upOk || !foreOk) {
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

  qUpRaw = fromBno(bnoUpper.getQuat());
  qForeRaw = fromBno(bnoForearm.getQuat());

  qUpZeroed = mulQ(conjQ(qUpZero), qUpRaw);
  qForeZeroed = mulQ(conjQ(qForeZero), qForeRaw);

  qJointZeroed = mulQ(conjQ(qUpZeroed), qForeZeroed);

  rawAng = angleQ(qJointZeroed);
  ang = filtJoint(rawAng);
}

void imuZero() {
  if (!imuOk) {
    Serial.println("Cannot zero IMUs. IMU not ready.");
    return;
  }

  qUpRaw = fromBno(bnoUpper.getQuat());
  qForeRaw = fromBno(bnoForearm.getQuat());

  qUpZero = qUpRaw;
  qForeZero = qForeRaw;

  qUpZeroed = {1.0, 0.0, 0.0, 0.0};
  qForeZeroed = {1.0, 0.0, 0.0, 0.0};
  qJointZeroed = {1.0, 0.0, 0.0, 0.0};

  rawAng = 0.0;
  ang = 0.0;

  resetFilt(0.0);
  resetTargets();

  Serial.println("IMU recalibrated. Current joint angle is now 0.");
}

// ======================================================
// Angle helpers
// ======================================================

float axisVal() {
  return ang;
}

float errDeg(float target, float current) {
  return target - current;
}

// ======================================================
// Motor helpers
// ======================================================

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

// ======================================================
// PID controller
// ======================================================

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
    m.lastU = 0.0;
    return;
  }

  float current = axisVal();
  float error = errDeg(m.target, current);
  float absErr = fabs(error);

  bool trajMode = trajOn && (&m == &m2);

  bool errFlip =
      (error > 0.0 && m.lastErr < 0.0) ||
      (error < 0.0 && m.lastErr > 0.0);

  if (errFlip) {
    m.sumErr = 0.0;
  }

  bool atTarget = false;

  if (!trajMode) {
    float exitTol = m.tol * 1.5;
    atTarget = m.holding ? (absErr <= exitTol) : (absErr <= m.tol);
  }

  if (atTarget) {
    hold(m);

    m.lastUNorm = 0.0;
    m.lastPwm = 0;
    m.lastU = 0.0;

    m.sumErr = 0.0;
    m.lastErr = error;
    m.lastMeas = current;
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

  if (m.holding) {
    m.holding = false;
    m.printed = false;

    m.sumErr = 0.0;
    m.lastErr = error;
    m.lastMeas = current;
    m.lastOut = 0.0;
    m.lastU = 0.0;
    m.startMs = millis();

    Serial.print(m.name);
    Serial.println(" moved away from target. PID re-engaging.");
  }

  if (!trajMode && millis() - m.startMs > m.timeout) {
    Serial.print(m.name);
    Serial.println(" timeout. Motor stopped.");

    off(m);
    resetMotor(m);
    return;
  }

  int pwmLimit = m.maxPwm;

  if (absErr <= m.slowZone) {
    pwmLimit = m.slowPwm;
  }

  float dMeas = (current - m.lastMeas) / dt;
  float dErr = -dMeas;

  bool saturated =
      (fabs(m.lastOut) >= fabs(m.uFull)) &&
      ((m.lastOut > 0.0) == (error > 0.0));

  if (!saturated) {
    m.sumErr += error * dt;
  }

  if (fabs(m.ki) > 0.000001) {
    float iLimit = fabs(m.uFull / m.ki);
    m.sumErr = constrain(m.sumErr, -iLimit, iLimit);
  } else {
    m.sumErr = 0.0;
  }

  float out =
      m.kp * error +
      m.kd * dErr +
      m.ki * m.sumErr;

  m.lastErr = error;
  m.lastMeas = current;
  m.lastOut = out;

  float uFull = fabs(m.uFull);

  if (uFull < 0.000001) {
    uFull = 1.0;
  }

  float uNorm = sat1(fabs(out) / uFull);
  int pwmMin = min(m.minPwm, pwmLimit);
  int pwm = pwmMin + (int)((pwmLimit - pwmMin) * uNorm);

  if (fabs(out) < 0.0001) {
    pwm = 0;
    uNorm = 0.0;
  }

  int dir = (out >= 0.0) ? FWD : REV;

  // Correct plant input signal:
  // This is the signed PWM command actually sent to the motor driver.
  float uCmd = dir * m.motSign * pwm;

  m.lastUNorm = uNorm;
  m.lastPwm = pwm;
  m.lastU = uCmd;

  drive(m, dir * m.motSign, pwm);
}

// ======================================================
// Target, reset, and trajectory
// ======================================================

bool motorEnabled(Motor& m) {
  if (&m == &m1) {
    return false;
  }

  if (&m == &m2) {
    return true;
  }

  return false;
}

void resetMotor(Motor& m) {
  m.active = false;
  m.holding = false;
  m.printed = false;

  m.lastErr = 0.0;
  m.sumErr = 0.0;
  m.lastMeas = axisVal();
  m.lastOut = 0.0;

  m.lastUNorm = 0.0;
  m.lastPwm = 0;
  m.lastU = 0.0;
}

void resetTargets() {
  stopTraj();

  resetMotor(m1);
  resetMotor(m2);

  m1.target = axisVal();
  m2.target = axisVal();

  manMode = false;
  manDir = 0;

  off(m1);
  off(m2);
}

void setTarget(float deg) {
  stopTraj();

  manMode = false;
  manDir = 0;

  float current = axisVal();

  m2.target = deg;
  m2.active = true;
  m2.holding = false;
  m2.printed = false;

  m2.lastErr = errDeg(m2.target, current);
  m2.sumErr = 0.0;
  m2.lastMeas = current;
  m2.lastOut = 0.0;
  m2.lastUNorm = 0.0;
  m2.lastPwm = 0;
  m2.lastU = 0.0;
  m2.startMs = millis();

  Serial.print("New Motor 2 target: ");
  Serial.print(deg, 2);
  Serial.println(" deg");
}

float calcTraj(float t) {
  return TRAJ_CENTER_DEG
         - TRAJ_AMP_DEG * cos(2.0 * PI * TRAJ_FREQ * t - 2.0 * PI);
}

void startTraj() {
  manMode = false;
  manDir = 0;

  trajOn = true;
  trajStartMs = millis();

  float current = axisVal();

  m2.target = calcTraj(0.0);
  m2.active = true;
  m2.holding = false;
  m2.printed = false;

  m2.lastErr = errDeg(m2.target, current);
  m2.sumErr = 0.0;
  m2.lastMeas = current;
  m2.lastOut = 0.0;
  m2.lastUNorm = 0.0;
  m2.lastPwm = 0;
  m2.lastU = 0.0;
  m2.startMs = millis();

  Serial.print("Sinusoidal trajectory started. Frequency: ");
  Serial.print(TRAJ_FREQ, 3);
  Serial.println(" Hz");
}

void stopTraj() {
  trajOn = false;
  trajStartMs = 0;
}

void updateTraj() {
  if (!trajOn || manMode) {
    return;
  }

  float t = (millis() - trajStartMs) / 1000.0;

  m2.target = calcTraj(t);
  m2.active = true;

  m2.holding = false;
  m2.printed = false;
}

// ======================================================
// Serial commands
// ======================================================

void serialCheck() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (escState == 0) {
      if (c == 27) {
        escState = 1;
      } else {
        handleChar(c);
      }
    } else if (escState == 1) {
      escState = (c == '[') ? 2 : 0;
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
    setTarget(KEY_TGTS[digit]);
    return;
  }

  if (c == 'x' || c == 'X') {
    if (trajOn) {
      Serial.println("Sinusoidal trajectory stopped.");
      resetTargets();
    } else {
      startTraj();
    }

    return;
  }

  if (c == 's' || c == 'p') {
    Serial.println("Stop command received.");
    resetTargets();
    return;
  }

  if (c == 'r') {
    Serial.println("Recalibrating IMUs...");
    resetTargets();
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
  stopTraj();

  if (c == 'D') {
    manMode = true;
    manDir = REV;

    m2.active = false;
    m2.holding = false;
    m2.printed = false;

    Serial.println("Manual mode: Motor 2 reverse");
  } else if (c == 'C') {
    manMode = true;
    manDir = FWD;

    m2.active = false;
    m2.holding = false;
    m2.printed = false;

    Serial.println("Manual mode: Motor 2 forward");
  }
}

// ======================================================
// Main update
// ======================================================

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

  if (dt > 0.05) {
    dt = 0.05;
  }

  imuRead();

  static bool wasMan = false;

  if (manMode) {
    off(m1);

    drive(m2, manDir * m2.motSign, MAN_PWM);

    m2.lastPwm = MAN_PWM;
    m2.lastUNorm = (float)MAN_PWM / (float)m2.maxPwm;

    // Correct signed manual command.
    m2.lastU = manDir * m2.motSign * MAN_PWM;

    if (m2.lastUNorm > 1.0) {
      m2.lastUNorm = 1.0;
    }

    printData();

    wasMan = true;
    return;
  }

  if (wasMan) {
    float current = axisVal();

    m2.lastErr = errDeg(m2.target, current);
    m2.sumErr = 0.0;
    m2.lastMeas = current;
    m2.lastOut = 0.0;
    m2.lastUNorm = 0.0;
    m2.lastPwm = 0;
    m2.lastU = 0.0;
    m2.holding = false;
    m2.printed = false;
    m2.startMs = millis();

    wasMan = false;
  }

  updateTraj();

  pid(m1, dt, motorEnabled(m1));
  pid(m2, dt, motorEnabled(m2));

  printData();
}

// ======================================================
// Printing
// ======================================================

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

  float targetDeg = manMode ? axisVal() : m2.target;
  float currentDeg = axisVal();
  float errorDeg = manMode ? 0.0 : errDeg(targetDeg, currentDeg);

  Serial.print("\nTargetDeg: ");
  Serial.print(targetDeg, 2);

  Serial.print(" | CurrentDeg: ");
  Serial.print(currentDeg, 2);

  Serial.print(" | RawDeg: ");
  Serial.print(rawAng, 2);

  Serial.print(" | RejectedSpikes: ");
  Serial.print(rejSpks);

  Serial.print(" | ErrorDeg: ");
  Serial.print(errorDeg, 2);

  Serial.print(" | PWMNorm: ");
  Serial.print(m2.lastUNorm, 3);

  Serial.print(" | PWM: ");
  Serial.print(m2.lastPwm);

  Serial.print(" | UCmd: ");
  Serial.print(m2.lastU, 3);

  Serial.print(" | Mode: ");

  if (manMode) {
    Serial.print("Manual");
  } else if (trajOn) {
    Serial.print("Traj");
  } else if (m2.active) {
    Serial.print("PID");
  } else {
    Serial.print("Idle");
  }

  Serial.print(" | FreqHz: ");
  Serial.print(TRAJ_FREQ, 3);

  Serial.print(" | M1Counts: ");
  Serial.print(counts(m1));

  Serial.print(" | M2Counts: ");
  Serial.println(counts(m2));

  printQuat("qUpperZeroed", qUpZeroed);
  printQuat("qForearmZeroed", qForeZeroed);
  printQuat("qJointZeroed", qJointZeroed);
}

// ======================================================
// Menu
// ======================================================

void menu() {
  Serial.println();
  Serial.println("========== MENU ==========");
  Serial.println("0-9  : Move Motor 2 to selected target");
  Serial.println("0 = 5 deg");
  Serial.println("1 = 10 deg");
  Serial.println("2 = 20 deg");
  Serial.println("3 = 30 deg");
  Serial.println("4 = 40 deg");
  Serial.println("5 = 50 deg");
  Serial.println("6 = 60 deg");
  Serial.println("7 = 70 deg");
  Serial.println("8 = 80 deg");
  Serial.println("9 = 90 deg");
  Serial.println("x    : Start/stop sinusoidal trajectory");
  Serial.println("Left : Manual Motor 2 reverse");
  Serial.println("Right: Manual Motor 2 forward");
  Serial.println("s/p  : Stop motor");
  Serial.println("r    : Recalibrate / zero IMUs");
  Serial.println("m    : Print menu");
  Serial.println("==========================");
  Serial.println();
}

// ======================================================
// Setup and loop
// ======================================================

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