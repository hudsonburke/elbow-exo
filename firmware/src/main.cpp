#include <Arduino.h>
#include <math.h>
#include <ctype.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// =====================================================
// Fast elbow exoskeleton data output
// Prints:
// DATA,time_ms,theta_deg,m1_counts,m2_counts
// =====================================================


// =====================================================
// Pin setup
// =====================================================

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


// =====================================================
// Serial print timing
// =====================================================

const unsigned long DATA_INTERVAL_MS = 10;     // 10 ms = about 100 Hz
const unsigned long QUAT_INTERVAL_MS = 100;    // 100 ms = about 10 Hz

const bool PRINT_FAST_DATA = true;
const bool PRINT_QUATERNIONS_SLOW = true;


// =====================================================
// Motor direction constants
// =====================================================

const int FWD = 1;
const int REV = -1;
const int STOPPED = 0;


// =====================================================
// Encoder objects
// =====================================================

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);


// =====================================================
// IMU objects
// =====================================================

// Upper IMU is on Wire.
// Forearm IMU is on Wire1.
// If your I2C scanner shows the forearm IMU is 0x29, change 0x28 to 0x29 below.
Adafruit_BNO055 bnoUpper = Adafruit_BNO055(0, 0x28, &Wire);
Adafruit_BNO055 bnoForearm = Adafruit_BNO055(1, 0x28, &Wire1);


// =====================================================
// Quaternion struct
// =====================================================

struct Q {
  float w;
  float x;
  float y;
  float z;
};

Q qUpperZero = {1.0, 0.0, 0.0, 0.0};
Q qForearmZero = {1.0, 0.0, 0.0, 0.0};

Q qUpperZeroed = {1.0, 0.0, 0.0, 0.0};
Q qForearmZeroed = {1.0, 0.0, 0.0, 0.0};
Q qJointZeroed = {1.0, 0.0, 0.0, 0.0};


// =====================================================
// Angle filtering and IMU status
// =====================================================

float ang = 0.0;
float rawAng = 0.0;
float filtAng = 0.0;

bool filtInit = false;
bool imuOk = false;
bool lastImuReadOk = false;

const float FILTER_ALPHA = 0.30;
const float SPIKE_LIMIT_DEG = 35.0;


// =====================================================
// Motor struct
// =====================================================

struct Motor {
  int in1;
  int in2;

  Encoder* enc;

  int encSign;
  int motSign;

  float target;
  float tol;
  float slowZone;

  bool active;

  float integ;
  float lastErr;

  float lastUNorm;
  float lastU;
  int lastPwm;

  int minPwm;
  int maxPwm;
  int slowPwm;

  unsigned long startMs;

  bool holding;
};


Motor m1 = {
  M1_IN1,
  M1_IN2,
  &enc1,
  1,
  1,
  0.0,
  1.0,
  5.0,
  false,
  0.0,
  0.0,
  0.0,
  0.0,
  0,
  150,
  255,
  150,
  0,
  false
};


Motor m2 = {
  M2_IN1,
  M2_IN2,
  &enc2,
  1,
  1,
  0.0,
  1.0,
  5.0,
  false,
  0.0,
  0.0,
  0.0,
  0.0,
  0,
  150,
  255,
  150,
  0,
  false
};


// =====================================================
// PID constants
// =====================================================

float kp = 1.75;
float ki = 0.05;
float kd = 0.0;

const float U_FULL = 20.0;
const float INTEGRAL_LIMIT = 300.0;

const unsigned long TARGET_TIMEOUT_MS = 15000;


// =====================================================
// Manual motor constants
// =====================================================

const int MAN_PWM = 150;
const unsigned long MANUAL_TIMEOUT_MS = 250;

int manDir = 0;
unsigned long lastManualMs = 0;


// =====================================================
// Trajectory constants
// =====================================================

bool trajOn = false;
unsigned long trajStartMs = 0;

const float TRAJ_FREQ = 0.05;
const float TRAJ_CENTER_DEG = 45.0;
const float TRAJ_AMP_DEG = 45.0;


// =====================================================
// I2C scanner helper
// =====================================================

void scanBus(TwoWire& bus, const char* name) {
  Serial.print("Scanning ");
  Serial.println(name);

  int found = 0;

  for (byte address = 1; address < 127; address++) {
    bus.beginTransmission(address);
    byte error = bus.endTransmission();

    if (error == 0) {
      Serial.print("Found device on ");
      Serial.print(name);
      Serial.print(" at address 0x");

      if (address < 16) {
        Serial.print("0");
      }

      Serial.println(address, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.print("No devices found on ");
    Serial.println(name);
  }

  Serial.println();
}


// =====================================================
// Quaternion math
// =====================================================

Q normQ(Q q) {
  float n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

  if (n < 1e-9) {
    return {1.0, 0.0, 0.0, 0.0};
  }

  return {
    q.w / n,
    q.x / n,
    q.y / n,
    q.z / n
  };
}


Q conjQ(Q q) {
  q = normQ(q);

  return {
    q.w,
    -q.x,
    -q.y,
    -q.z
  };
}


Q mulQ(Q a, Q b) {
  Q r;

  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

  return normQ(r);
}


float angleQ(Q q) {
  q = normQ(q);

  float w = fabs(q.w);

  if (w > 1.0) {
    w = 1.0;
  }

  float angle = 2.0 * acos(w) * 180.0 / PI;

  return angle;
}


// =====================================================
// IMU angle filtering
// =====================================================

void resetFilt(float value) {
  filtAng = value;
  filtInit = true;
}


float filtJoint(float rawAngle) {
  if (!filtInit) {
    resetFilt(rawAngle);
    return rawAngle;
  }

  float diff = rawAngle - filtAng;

  if (fabs(diff) > SPIKE_LIMIT_DEG) {
    return filtAng;
  }

  filtAng = FILTER_ALPHA * rawAngle + (1.0 - FILTER_ALPHA) * filtAng;

  return filtAng;
}


// =====================================================
// IMU functions
// =====================================================

bool imuStart() {
  Wire.begin();
  Wire1.begin();

  // Slower I2C is safer for the BNO055.
  Wire.setClock(100000);
  Wire1.setClock(100000);

  delay(500);

  scanBus(Wire, "Wire");
  scanBus(Wire1, "Wire1");

  bool upperOk = false;
  bool forearmOk = false;

  for (int i = 0; i < 5; i++) {
    if (!upperOk) {
      upperOk = bnoUpper.begin();
    }

    if (!forearmOk) {
      forearmOk = bnoForearm.begin();
    }

    if (upperOk && forearmOk) {
      break;
    }

    Serial.println("Retrying IMU startup...");
    delay(500);
  }

  if (!upperOk) {
    Serial.println("ERROR: Upper IMU not detected.");
  }

  if (!forearmOk) {
    Serial.println("ERROR: Forearm IMU not detected.");
    Serial.println("Check that forearm IMU SDA is on pin 17 and SCL is on pin 16.");
    Serial.println("If scanner shows 0x29 on Wire1, change the forearm address to 0x29.");
  }

  if (!upperOk || !forearmOk) {
    imuOk = false;
    lastImuReadOk = false;
    return false;
  }

  delay(1000);

  bnoUpper.setExtCrystalUse(true);
  bnoForearm.setExtCrystalUse(true);

  delay(500);

  imuOk = true;
  lastImuReadOk = true;

  Serial.println("IMUs started.");

  return true;
}


bool imuReady() {
  return imuOk && lastImuReadOk;
}


bool imuReadRaw(Q& qUpperRaw, Q& qForearmRaw) {
  if (!imuOk) {
    lastImuReadOk = false;
    return false;
  }

  imu::Quaternion rawUpper = bnoUpper.getQuat();
  imu::Quaternion rawForearm = bnoForearm.getQuat();

  Q upperTemp = {
    (float)rawUpper.w(),
    (float)rawUpper.x(),
    (float)rawUpper.y(),
    (float)rawUpper.z()
  };

  Q forearmTemp = {
    (float)rawForearm.w(),
    (float)rawForearm.x(),
    (float)rawForearm.y(),
    (float)rawForearm.z()
  };

  float upperNorm = sqrt(
    upperTemp.w * upperTemp.w +
    upperTemp.x * upperTemp.x +
    upperTemp.y * upperTemp.y +
    upperTemp.z * upperTemp.z
  );

  float forearmNorm = sqrt(
    forearmTemp.w * forearmTemp.w +
    forearmTemp.x * forearmTemp.x +
    forearmTemp.y * forearmTemp.y +
    forearmTemp.z * forearmTemp.z
  );

  if (upperNorm < 0.5 || forearmNorm < 0.5) {
    lastImuReadOk = false;
    return false;
  }

  qUpperRaw = normQ(upperTemp);
  qForearmRaw = normQ(forearmTemp);

  lastImuReadOk = true;
  return true;
}


void imuZero() {
  if (!imuOk) {
    Serial.println("Cannot zero IMUs. IMU not ready.");
    return;
  }

  Q qUpperRaw;
  Q qForearmRaw;

  bool readOk = false;

  for (int i = 0; i < 10; i++) {
    if (imuReadRaw(qUpperRaw, qForearmRaw)) {
      readOk = true;
      break;
    }

    delay(20);
  }

  if (!readOk) {
    Serial.println("Cannot zero IMUs. Bad quaternion read.");
    lastImuReadOk = false;
    return;
  }

  qUpperZero = qUpperRaw;
  qForearmZero = qForearmRaw;

  qUpperZeroed = {1.0, 0.0, 0.0, 0.0};
  qForearmZeroed = {1.0, 0.0, 0.0, 0.0};
  qJointZeroed = {1.0, 0.0, 0.0, 0.0};

  rawAng = 0.0;
  ang = 0.0;
  resetFilt(0.0);

  enc1.write(0);
  enc2.write(0);

  lastImuReadOk = true;

  Serial.println("IMUs and encoders zeroed.");
}


bool imuRead() {
  if (!imuOk) {
    lastImuReadOk = false;
    return false;
  }

  Q qUpperRaw;
  Q qForearmRaw;

  bool readOk = imuReadRaw(qUpperRaw, qForearmRaw);

  if (!readOk) {
    return false;
  }

  qUpperZeroed = mulQ(conjQ(qUpperZero), qUpperRaw);
  qForearmZeroed = mulQ(conjQ(qForearmZero), qForearmRaw);

  qJointZeroed = mulQ(conjQ(qUpperZeroed), qForearmZeroed);

  rawAng = angleQ(qJointZeroed);
  ang = filtJoint(rawAng);

  lastImuReadOk = true;

  return true;
}


// =====================================================
// Motor helper functions
// =====================================================

bool motorEnabled(Motor& m) {
  if (&m == &m1) {
    return false;
  }

  if (&m == &m2) {
    return true;
  }

  return false;
}


float axisVal() {
  return ang;
}


float errDeg(float target, float current) {
  return target - current;
}


long counts(Motor& m) {
  return m.encSign * m.enc->read();
}


void drive(Motor& m, int dir, int pwm) {
  pwm = constrain(pwm, 0, 195);

  if (dir > 0) {
    analogWrite(m.in1, pwm);
    analogWrite(m.in2, 0);
  }
  else if (dir < 0) {
    analogWrite(m.in1, 0);
    analogWrite(m.in2, pwm);
  }
  else {
    analogWrite(m.in1, 0);
    analogWrite(m.in2, 0);
  }
}


void off(Motor& m) {
  analogWrite(m.in1, 0);
  analogWrite(m.in2, 0);

  m.lastPwm = 0;
  m.lastUNorm = 0.0;
  m.lastU = 0.0;
  m.holding = false;
}


void hold(Motor& m) {
  analogWrite(m.in1, 255);
  analogWrite(m.in2, 255);

  m.lastPwm = 0;
  m.lastUNorm = 0.0;
  m.lastU = 0.0;
  m.holding = true;
}


void resetMotorControl(Motor& m) {
  m.integ = 0.0;
  m.lastErr = 0.0;
  m.lastPwm = 0;
  m.lastUNorm = 0.0;
  m.lastU = 0.0;
  m.startMs = millis();
  m.holding = false;
}


// =====================================================
// Trajectory functions
// =====================================================

float calcTraj(float t) {
  return TRAJ_CENTER_DEG
         - TRAJ_AMP_DEG * cos(2.0 * PI * TRAJ_FREQ * t - 2.0 * PI);
}


void startTraj() {
  manDir = 0;
  off(m2);

  trajOn = true;
  trajStartMs = millis();

  m2.active = true;
  resetMotorControl(m2);

  Serial.println("Trajectory started.");
}


void stopTraj() {
  trajOn = false;
  m2.active = false;
  off(m2);

  Serial.println("Trajectory stopped.");
}


void updateTraj() {
  if (!trajOn) {
    return;
  }

  float t = (millis() - trajStartMs) / 1000.0;

  m2.target = calcTraj(t);
  m2.active = true;
}


// =====================================================
// Target control
// =====================================================

void setTarget(float targetDeg) {
  trajOn = false;
  manDir = 0;

  m2.target = targetDeg;
  m2.active = true;

  resetMotorControl(m2);

  Serial.print("New target: ");
  Serial.print(targetDeg);
  Serial.println(" deg");
}


void stopAll() {
  trajOn = false;
  manDir = 0;

  m1.active = false;
  m2.active = false;

  off(m1);
  off(m2);

  Serial.println("Motors stopped.");
}


// =====================================================
// Manual control
// =====================================================

void manualM2(int dir) {
  trajOn = false;

  manDir = dir;
  lastManualMs = millis();

  m2.active = false;
  m2.holding = false;

  int actualDir = dir * m2.motSign;

  m2.lastPwm = MAN_PWM;
  m2.lastUNorm = (float)MAN_PWM / 255.0;
  m2.lastU = actualDir * MAN_PWM;

  drive(m2, actualDir, MAN_PWM);
}


void updateManualTimeout() {
  if (manDir == 0) {
    return;
  }

  if (millis() - lastManualMs > MANUAL_TIMEOUT_MS) {
    manDir = 0;
    off(m2);
  }
}


// =====================================================
// PID control
// =====================================================

void pid(Motor& m, float dt) {
  if (!imuReady() || !motorEnabled(m)) {
    off(m);
    return;
  }

  if (&m == &m2 && manDir != 0) {
    return;
  }

  if (!m.active) {
    return;
  }

  bool trajMode = trajOn && (&m == &m2);

  float current = axisVal();
  float e = errDeg(m.target, current);

  if (!trajMode && fabs(e) <= m.tol) {
    m.active = false;
    hold(m);
    return;
  }

  if (!trajMode && millis() - m.startMs > TARGET_TIMEOUT_MS) {
    m.active = false;
    off(m);
    Serial.println("Target timeout.");
    return;
  }

  m.integ += e * dt;
  m.integ = constrain(m.integ, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float de = 0.0;

  if (dt > 1e-4) {
    de = (e - m.lastErr) / dt;
  }

  float out = kp * e + ki * m.integ + kd * de;

  m.lastErr = e;

  float absOut = fabs(out);

  if (absOut < 1e-5) {
    off(m);
    return;
  }

  float uNorm = absOut / U_FULL;
  uNorm = constrain(uNorm, 0.0, 1.0);

  int pwm = m.minPwm + (int)((m.maxPwm - m.minPwm) * uNorm);
  pwm = constrain(pwm, m.minPwm, m.maxPwm);

  if (!trajMode && fabs(e) < m.slowZone) {
    pwm = m.slowPwm;
  }

  int dir = (out >= 0.0) ? FWD : REV;

  int actualDir = dir * m.motSign;

  m.lastPwm = pwm;
  m.lastUNorm = (float)pwm / 255.0;
  m.lastU = actualDir * pwm;

  drive(m, actualDir, pwm);
}


// =====================================================
// Serial print functions
// =====================================================

void printQ(const char* label, Q q) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print(q.w, 6);
  Serial.print(",");
  Serial.print(q.x, 6);
  Serial.print(",");
  Serial.print(q.y, 6);
  Serial.print(",");
  Serial.println(q.z, 6);
}


void printData() {
  static unsigned long lastDataPrint = 0;
  static unsigned long lastQuatPrint = 0;

  unsigned long now = millis();

  if (PRINT_FAST_DATA && now - lastDataPrint >= DATA_INTERVAL_MS) {
    lastDataPrint = now;

    Serial.print("DATA,");
    Serial.print(now);
    Serial.print(",");
    Serial.print(axisVal(), 4);
    Serial.print(",");
    Serial.print(counts(m1));
    Serial.print(",");
    Serial.println(counts(m2));
  }

  if (PRINT_QUATERNIONS_SLOW && now - lastQuatPrint >= QUAT_INTERVAL_MS) {
    lastQuatPrint = now;

    printQ("qUpperZeroed", qUpperZeroed);
    printQ("qForearmZeroed", qForearmZeroed);
    printQ("qJointZeroed", qJointZeroed);
  }
}


// =====================================================
// Menu and serial commands
// =====================================================

void printMenu() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  0-9  : set target angle");
  Serial.println("         0 = 5 deg");
  Serial.println("         1 = 10 deg");
  Serial.println("         2 = 20 deg");
  Serial.println("         ...");
  Serial.println("         9 = 90 deg");
  Serial.println("  x    : start/stop sinusoidal trajectory");
  Serial.println("  left : manual reverse Motor 2");
  Serial.println("  right: manual forward Motor 2");
  Serial.println("  s/p  : stop motor");
  Serial.println("  r    : reset IMU zero and encoder counts");
  Serial.println("  m    : print menu");
  Serial.println();
  Serial.println("Fast data format:");
  Serial.println("  DATA,time_ms,theta_deg,m1_counts,m2_counts");
  Serial.println();
}


void handleArrow(char arrowChar) {
  if (arrowChar == 'D') {
    manualM2(REV);
  }
  else if (arrowChar == 'C') {
    manualM2(FWD);
  }
}


void handleChar(char c) {
  if (c == '\n' || c == '\r') {
    return;
  }

  if (isdigit(c)) {
    int digit = c - '0';

    float targetDeg;

    if (digit == 0) {
      targetDeg = 5.0;
    }
    else {
      targetDeg = digit * 10.0;
    }

    setTarget(targetDeg);
    return;
  }

  if (c == 'x' || c == 'X') {
    if (trajOn) {
      stopTraj();
    }
    else {
      startTraj();
    }

    return;
  }

  if (c == 's' || c == 'S' || c == 'p' || c == 'P') {
    stopAll();
    return;
  }

  if (c == 'r' || c == 'R') {
    imuZero();
    return;
  }

  if (c == 'm' || c == 'M') {
    printMenu();
    return;
  }
}


void serialCheck() {
  static int escState = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (escState == 0) {
      if (c == 27) {
        escState = 1;
      }
      else {
        handleChar(c);
      }
    }
    else if (escState == 1) {
      if (c == '[') {
        escState = 2;
      }
      else {
        escState = 0;
      }
    }
    else if (escState == 2) {
      handleArrow(c);
      escState = 0;
    }
  }
}


// =====================================================
// Setup and loop
// =====================================================

void setup() {
  Serial.begin(115200);

  // Longer delay gives the BNO055 more time to power up.
  delay(2500);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  analogWriteResolution(8);

  off(m1);
  off(m2);

  Serial.println("Starting elbow controller...");

  bool started = imuStart();

  if (!started) {
    Serial.println("IMU start failed. Check wiring.");
    while (1) {
      delay(1000);
    }
  }

  imuZero();

  printMenu();

  Serial.println("Controller ready.");
}


void loop() {
  static unsigned long lastLoopUs = micros();

  unsigned long nowUs = micros();
  float dt = (nowUs - lastLoopUs) / 1000000.0;
  lastLoopUs = nowUs;

  if (dt <= 0.0 || dt > 0.2) {
    dt = 0.01;
  }

  serialCheck();

  imuRead();

  updateTraj();

  updateManualTimeout();

  pid(m1, dt);
  pid(m2, dt);

  printData();
}