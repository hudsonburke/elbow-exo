#include <Arduino.h>
#include <math.h>
#include <ctype.h>

#include <Wire.h>
#include <Encoder.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// =========================
// IMU
// =========================
//
// Two BNO055s on separate Teensy I2C buses.
// Upper-arm/base IMU:   Wire
// Forearm/moving IMU:  Wire1
//
// The controller uses one scalar measurement:
//   jointAngleDeg = forearm angle relative to upper arm
//
// That angle is computed from relative quaternions, not Euler angles.

Adafruit_BNO055 bnoUpper(0, 0x28, &Wire);
Adafruit_BNO055 bnoForearm(1, 0x28, &Wire1);

bool imuUpperOk = false;
bool imuForearmOk = false;

imu::Quaternion qUpper;
imu::Quaternion qForearm;
imu::Quaternion qRel;
imu::Quaternion qZeroRel(1.0, 0.0, 0.0, 0.0);

float jointRawDeg = 0.0;
float jointAngleDeg = 0.0;

// This magnitude-only version does not require hinge-axis selection.
// It measures the size of the zeroed relative rotation between the two IMUs.
// Output is always positive: 0 deg to 180 deg.


// =========================
// CONTROL MODE
// =========================
//
// This version is fixed to one quaternion-based joint angle magnitude.
// No X/Y/Z hinge axis is selected.
// Serial choices: r = zero, 0-9 = target, s = stop, m = menu.

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

// Roll mode:
// Motor 1 turns the IMU.
// Motor 2 runs opposite.
const int M1_ROLL_SIGN = 1;
const int M2_ROLL_SIGN = -1;

const int MIN_PWM = 125;
const int MAX_PWM = 255;
const int CMD_PWM = 220;

const unsigned long CTRL_US = 10000;   // 10 ms
const unsigned long PRINT_MS = 250;

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
  1.0, 5.0, 12000,
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
  1.0, 5.0, 12000,
  0.0, 0.0, 0.0,
  false, false, false,
  0
};

unsigned long lastCtrlUs = 0;
unsigned long lastPrintMs = 0;


// =========================
// FUNCTION DECLARATIONS
// =========================

void imuStart();
void imuRead();
void imuZero();

bool imuReady();

imu::Quaternion normalizeQ(imu::Quaternion q);
imu::Quaternion relativeQ(const imu::Quaternion& upper, const imu::Quaternion& forearm);
float relativeAngleMagnitudeDeg(const imu::Quaternion& q);

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

long counts(Motor& m);

void drive(Motor& m, int dir, int pwm);
void off(Motor& m);
void hold(Motor& m);

void serialCheck();
void handleCmd(char c);
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

  Serial.println();
  Serial.println("Place the mechanism at the zero position.");
  Serial.print("Selected measurement: ");
  Serial.println(axisName());
  Serial.println("Type r and press Enter to recalibrate the current joint position to 0.");
  Serial.println();

  waitCal();

  imuRead();

  m1.target = axisVal();
  m2.target = axisVal();

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
  Serial.println("Starting two BNO055 IMUs...");

  Wire.begin();
  Wire1.begin();

  if (bnoUpper.begin()) {
    imuUpperOk = true;
    Serial.println("Upper-arm BNO055 detected on Wire.");
    delay(500);
    bnoUpper.setExtCrystalUse(true);  
    // Use IMU-only fusion to avoid magnetometer-based heading
    // inconsistencies which can make the relative orientation change
    // when the whole system is rotated. IMU mode uses accel+gyro.
    bnoUpper.setMode(OPERATION_MODE_IMUPLUS);
    delay(200);
  } else {
    imuUpperOk = false;
    Serial.println("Upper-arm BNO055 NOT detected on Wire.");
  }

  if (bnoForearm.begin()) {
    imuForearmOk = true;
    Serial.println("Forearm BNO055 detected on Wire1.");
    delay(500);
    bnoForearm.setExtCrystalUse(true);
    bnoForearm.setMode(OPERATION_MODE_IMUPLUS);
    delay(200);
  } else {
    imuForearmOk = false;
    Serial.println("Forearm BNO055 NOT detected on Wire1.");
  }
}

bool imuReady() {
  return imuUpperOk && imuForearmOk;
}

imu::Quaternion normalizeQ(imu::Quaternion q) {
  float mag = sqrt(
    q.w()*q.w() +
    q.x()*q.x() +
    q.y()*q.y() +
    q.z()*q.z()
  );

  if (isnan(mag) || mag < 0.000001) {
    return imu::Quaternion(1.0, 0.0, 0.0, 0.0);
  }

  q.normalize();
  return q;
}

imu::Quaternion relativeQ(const imu::Quaternion& upper, const imu::Quaternion& forearm) {
  // Rotation of forearm IMU relative to upper-arm IMU.
  // If both IMUs rotate together in space, this relative rotation should stay constant.
  imu::Quaternion upperInv = upper.conjugate();
  imu::Quaternion rel = upperInv * forearm;
  return normalizeQ(rel);
}

// Magnitude-only relative joint angle.
// This ignores the quaternion axis and uses only the size of the zeroed relative rotation.
// For a true 1-DOF joint, this should change when the joint bends and stay nearly constant
// when the entire arm/person rotates without changing the joint angle.
// Output is always positive: +45 and -45 both report 45.
float relativeAngleMagnitudeDeg(const imu::Quaternion& q) {
  float w = q.w();
  float x = q.x();
  float y = q.y();
  float z = q.z();

  if (isnan(w) || isnan(x) || isnan(y) || isnan(z)) {
    return jointAngleDeg; // keep last good value
  }

  // Normalize again defensively in case numerical error accumulated.
  float mag = sqrt(w*w + x*x + y*y + z*z);
  if (isnan(mag) || mag < 0.000001) {
    return jointAngleDeg;
  }

  w = w / mag;

  // Keep acos input valid even if numerical noise makes w slightly outside [-1, 1].
  w = constrain(w, -1.0, 1.0);

  // q and -q represent the same orientation. fabs(w) returns the shortest rotation magnitude.
  float angleRad = 2.0 * acos(fabs(w));

  if (isnan(angleRad)) {
    return jointAngleDeg;
  }

  float angleDeg = angleRad * 180.0 / PI;

  if (angleDeg < 0.0001) {
    angleDeg = 0.0;
  }

  return angleDeg;
}
void imuRead() {
  if (!imuReady()) {
    return;
  }

  qUpper = normalizeQ(bnoUpper.getQuat());
  qForearm = normalizeQ(bnoForearm.getQuat());

  qRel = relativeQ(qUpper, qForearm);

  // Remove the zero/reference relative orientation.
  // After imuZero(), the current pose reports as 0 degrees.
  imu::Quaternion zeroInv = qZeroRel.conjugate();
  imu::Quaternion qJoint = normalizeQ(zeroInv * qRel);

  jointRawDeg = relativeAngleMagnitudeDeg(qJoint);
  jointAngleDeg = jointRawDeg;
}

void imuZero() {
  if (!imuReady()) {
    Serial.println("Cannot recalibrate: one or both IMUs are not ready.");
    return;
  }

  qUpper = normalizeQ(bnoUpper.getQuat());
  qForearm = normalizeQ(bnoForearm.getQuat());

  qZeroRel = relativeQ(qUpper, qForearm);

  jointRawDeg = 0.0;
  jointAngleDeg = 0.0;

  resetTargets();

  Serial.println("IMUs recalibrated. Current upper-arm/forearm joint position is now 0.");
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
  return "relative quaternion angle magnitude";
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
        Serial.println("Calibration complete.");
        Serial.println();
        return;
      }

      if (c == 'm' || c == 'M') {
        menu();
        continue;
      }

      Serial.println("Please type r and press Enter to recalibrate first.");
    }
  }
}


void menu() {
  Serial.println("Serial control mode is ON.");
  Serial.print("Active measurement: ");
  Serial.println(axisName());
  Serial.println("Type a number and press Enter to move Motor 2 to that joint angle:");
  Serial.println("Motor 2 controls the zeroed relative quaternion angle magnitude.");
  Serial.println("Motor 1 is kept stopped in this version.");
  Serial.println("0 = 0 deg");
  Serial.println("1 = 10 deg");
  Serial.println("2 = 20 deg");
  Serial.println("3 = 30 deg");
  Serial.println("4 = 40 deg");
  Serial.println("5 = 50 deg");
  Serial.println("6 = 60 deg");
  Serial.println("7 = 70 deg");
  Serial.println("8 = 80 deg");
  Serial.println("9 = 90 deg");
  Serial.println("s = stop both motors");
  Serial.println("r = recalibrate joint angle to 0");
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

  pid(m1, dt, motorEnabled(m1));
  pid(m2, dt, motorEnabled(m2));

  if (m1.active || m2.active) {
    printData();
  }
}

bool motorEnabled(Motor& m) {
  // This version is a 1-DOF joint controller.
  // Motor 2 is controlled by the quaternion joint angle.
  // Motor 1 is kept off for now.
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
  if (millis() - m.startMs > m.timeout) {
    off(m);
    resetMotor(m);

    Serial.println();
    Serial.print(m.name);
    Serial.println(" move timed out. Motor is fully OFF.");

    Serial.print("Measurement: ");
    Serial.print(axisName());

    Serial.print(" | TargetDeg: ");
    Serial.print(m.target, 2);

    Serial.print(" | CurrentDeg: ");
    Serial.print(axisVal(), 2);

    Serial.print(" | ErrorDeg: ");
    Serial.println(errDeg(m.target, axisVal()), 2);

    Serial.println("Choose another target or recalibrate with r.");
    Serial.println();

    return;
  }

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
  imuRead();

  m1.target = axisVal();
  m2.target = axisVal();

  resetMotor(m1);
  resetMotor(m2);

  off(m1);
  off(m2);
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

    if (c == '\n' || c == '\r') {
      continue;
    }

    handleCmd(c);
  }
}

void handleCmd(char c) {
  if (c >= '0' && c <= '9') {
    int key = c - '0';
    float target = KEY_TARGETS[key];

    // 1-DOF mode: Motor 2 controls the quaternion-derived joint angle.
    // Motor 1 stays stopped.
    imuRead();

    off(m1);
    resetMotor(m1);
    m1.target = axisVal();

    setTarget(m2, target, CMD_PWM);

    Serial.println("Joint-angle mode active: Motor 2 is controlling the quaternion-derived joint angle. Motor 1 is stopped.");
    Serial.println();
    return;
  }


  if (c == 'r' || c == 'R') {
    imuZero();

    Serial.println("Target reset to current joint angle after IMU zero.");
    Serial.println("Choose a number target when ready.");
    Serial.println();
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

    float targetDeg = m2.target;
    float currentDeg = axisVal();
    float errorDeg = errDeg(targetDeg, currentDeg);

    // Teleplot output
    Serial.print(">targetDeg:");
    Serial.println(targetDeg, 2);

    Serial.print(">currentDeg:");
    Serial.println(currentDeg, 2);

    Serial.print(">errorDeg:");
    Serial.println(errorDeg, 2);

    // Readable output
    Serial.print("Measurement: ");
    Serial.print(axisName());

    Serial.print(" | TargetDeg: ");
    Serial.print(targetDeg, 2);

    Serial.print(" | CurrentDeg: ");
    Serial.print(currentDeg, 2);

    Serial.print(" | ErrorDeg: ");
    Serial.print(errorDeg, 2);

    Serial.print(" | M1Counts: ");
    Serial.print(counts(m1));

    Serial.print(" | M2Counts: ");
    Serial.println(counts(m2));
  }
}