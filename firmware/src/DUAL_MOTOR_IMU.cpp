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

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
bool imuOk = false;

enum AxisId {
  AX_PITCH = 0,
  AX_ROLL  = 1,
  AX_YAW   = 2
};

struct Axis {
  const char* name;
  char key;
  int sign;
  float zero;
  float raw;
  float val;
};

// If an axis reads backwards, flip the sign here.
Axis ax[3] = {
  {"pitch", 'p', -1, 0.0, 0.0, 0.0},
  {"roll",  'o',  1, 0.0, 0.0, 0.0},
  {"yaw",   'y',  1, 0.0, 0.0, 0.0}
};

AxisId axis = AX_PITCH;


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

void axisMenu();
void pickAxis();
bool setAxis(char c);

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

  pickAxis();

  Serial.println();
  Serial.println("Place the mechanism at the zero position.");
  Serial.print("Selected IMU axis: ");
  Serial.println(axisName());
  Serial.println("Type r and press Enter to recalibrate the selected IMU axis to 0.");
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
  Serial.println("Starting BNO055 IMU...");

  Wire.begin();

  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring or I2C address.");
    imuOk = false;
    return;
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  imuOk = true;
  Serial.println("BNO055 detected!");

  delay(500);
}

void imuRead() {
  if (!imuOk) {
    return;
  }

  sensors_event_t e;
  bno.getEvent(&e);

  // BNO055 orientation mapping:
  // x = yaw
  // y = pitch
  // z = roll
  ax[AX_YAW].raw = angDiff(e.orientation.x, ax[AX_YAW].zero);
  ax[AX_PITCH].raw = angDiff(e.orientation.y, ax[AX_PITCH].zero);
  ax[AX_ROLL].raw = angDiff(e.orientation.z, ax[AX_ROLL].zero);

  for (int i = 0; i < 3; i++) {
    ax[i].val = ax[i].sign * ax[i].raw;
  }
}

void imuZero() {
  if (!imuOk) {
    return;
  }

  sensors_event_t e;
  bno.getEvent(&e);

  ax[AX_YAW].zero = e.orientation.x;
  ax[AX_PITCH].zero = e.orientation.y;
  ax[AX_ROLL].zero = e.orientation.z;

  for (int i = 0; i < 3; i++) {
    ax[i].raw = 0.0;
    ax[i].val = 0.0;
  }

  resetTargets();

  Serial.print("IMU recalibrated. Selected axis is ");
  Serial.print(axisName());
  Serial.println(" and current position is now 0.");
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
  return ax[axis].val;
}

float axisRaw() {
  return ax[axis].raw;
}

const char* axisName() {
  return ax[axis].name;
}


// =========================
// AXIS / MENU FUNCTIONS
// =========================

void axisMenu() {
  Serial.println("Choose which IMU axis the motors should control:");
  Serial.println("p = pitch");
  Serial.println("y = yaw");
  Serial.println("o = roll");
  Serial.println("Note: roll uses o because r is used for recalibration.");
  Serial.println();
}

void pickAxis() {
  axisMenu();

  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        continue;
      }

      if (setAxis(c)) {
        Serial.println();
        Serial.print("IMU axis selected: ");
        Serial.println(axisName());
        Serial.println();
        return;
      }

      Serial.println("Please choose p, y, or o first.");
    }
  }
}

bool setAxis(char c) {
  c = tolower(c);

  for (int i = 0; i < 3; i++) {
    if (c == ax[i].key) {
      axis = (AxisId)i;
      return true;
    }
  }

  return false;
}

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

      Serial.println("Please type r and press Enter to recalibrate first.");
    }
  }
}

void menu() {
  Serial.println("Serial control mode is ON.");
  Serial.print("Active IMU axis: ");
  Serial.println(axisName());
  Serial.println("Type a number and press Enter to move to that selected-axis angle:");
  Serial.println("Pitch mode: Motor 2 controls pitch. Motor 1 stays stopped.");
  Serial.println("Roll mode: Motor 1 and Motor 2 control roll in opposite directions.");
  Serial.println("Yaw mode: both motors use shared behavior until tuned separately.");
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
  Serial.println("p = switch to pitch");
  Serial.println("y = switch to yaw");
  Serial.println("o = switch to roll");
  Serial.println("s = stop both motors");
  Serial.println("r = recalibrate IMU to 0");
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
  // Pitch mode uses Motor 2 only.
  if (axis == AX_PITCH && &m == &m1) {
    return false;
  }

  return true;
}

void pid(Motor& m, float dt, bool enabled) {
  if (!imuOk || !enabled) {
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

      Serial.print("Axis: ");
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

    Serial.print("Axis: ");
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

  if (axis == AX_ROLL) {
    dir *= m.rollSign;
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
  Serial.print(" degrees using IMU ");
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

    if (axis == AX_PITCH) {
      // Pitch: Motor 2 only.
      imuRead();

      off(m1);
      resetMotor(m1);
      m1.target = axisVal();

      setTarget(m2, target, CMD_PWM);

      Serial.println("Pitch mode active: Motor 2 is controlling pitch. Motor 1 is stopped.");
      Serial.println();
      return;
    }

    if (axis == AX_ROLL) {
      // Roll: both motors, opposite directions.
      setTarget(m1, target, CMD_PWM);
      setTarget(m2, target, CMD_PWM);

      Serial.println("Roll mode active: Motor 1 turns the IMU and Motor 2 runs opposite to reverse/counter that motion.");
      Serial.println();
      return;
    }

    // Yaw: both motors for now.
    setTarget(m1, target, CMD_PWM);
    setTarget(m2, target, CMD_PWM);

    Serial.println("Yaw mode active: using shared behavior for now.");
    Serial.println();
    return;
  }

  if (c == 'p' || c == 'P' ||
      c == 'y' || c == 'Y' ||
      c == 'o' || c == 'O') {
    if (setAxis(c)) {
      resetTargets();

      Serial.println();
      Serial.print("Control axis switched to ");
      Serial.print(axisName());
      Serial.println(".");
      Serial.println("Targets were reset to the current selected-axis angle.");
      Serial.println("Choose a number target when ready.");
      Serial.println();
    }

    return;
  }

  if (c == 'r' || c == 'R') {
    imuZero();

    Serial.println("Target reset to current angle after IMU zero.");
    Serial.println("Choose a number target when ready.");
    Serial.println();
    return;
  }

  if (c == 's' || c == 'S') {
    resetTargets();

    Serial.println();
    Serial.print("Motor 1 and Motor 2 stopped. Current ");
    Serial.print(axisName());
    Serial.print(" angle: ");
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
    Serial.print("Axis: ");
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