#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*
  Open-loop system identification sketch for the elbow exoskeleton.

  This version is for PlatformIO / VS Code as src/main.cpp.

  What this code is for:
    1. Do NOT run PID.
    2. Send known PWM commands to Motor 2.
    3. Log the actual command, IMU elbow angle, encoder counts, and cable estimate.
    4. Use the DATA lines in MATLAB System Identification.

  Serial commands:
    m  print menu
    z  zero IMUs and encoders
    s  stop motor immediately
    v  cycle fixed PWM value
    +  increase fixed PWM by 5
    -  decrease fixed PWM by 5
    n  run one fixed-PWM pulse trial
    q  run a PRBS-style pulse sequence
    a  manual reverse
    d  manual forward

  DATA format:
    DATA,time_s,theta_deg,raw_theta_deg,m1_counts,m2_counts,delta_l_m,u_cmd,u_eff,pwm,state,trial_id

  RESULT format:
    RESULT,trial_id,mode,T_s,u_cmd,u_eff,c0,ct,delta_c,delta_l_m,theta0_deg,thetat_deg,early_stop
*/

struct Quat {
  float w;
  float x;
  float y;
  float z;
};

enum RunMode {
  MODE_IDLE,
  MODE_WAIT_SINGLE,
  MODE_SINGLE,
  MODE_WAIT_PRBS,
  MODE_PRBS,
  MODE_MANUAL
};

// Forward declarations needed in .cpp files.
Quat normQ(Quat q);
Quat conjQ(Quat q);
Quat mulQ(Quat a, Quat b);
Quat fromBno(imu::Quaternion q);
float angleQDeg(Quat q);

bool startImu();
void readImu();
void zeroImu();

long countsM1();
long countsM2();
float cableDeltaFromCounts(long deltaC);
float computeUEff(int uCmd);
void driveM2Signed(int uCmd);
void stopMotor();

const char* modeName();
bool unsafeAngle();
void printResult(bool earlyStop);
void stopTrial(const char* reason, bool earlyStop);
void startSingleTrial();
void startPrbsTrial();
void updateTrial();

void printData();
void printMenu();
void zeroAll();
void handleSerial();

// ======================================================
// IMU setup
// ======================================================

// If your wiring is the opposite, swap Wire and Wire1 here.
Adafruit_BNO055 bnoUpper(0, 0x28, &Wire);
Adafruit_BNO055 bnoForearm(1, 0x28, &Wire1);

Quat qUpRaw = {1.0, 0.0, 0.0, 0.0};
Quat qForeRaw = {1.0, 0.0, 0.0, 0.0};
Quat qUpZero = {1.0, 0.0, 0.0, 0.0};
Quat qForeZero = {1.0, 0.0, 0.0, 0.0};
Quat qUpZeroed = {1.0, 0.0, 0.0, 0.0};
Quat qForeZeroed = {1.0, 0.0, 0.0, 0.0};
Quat qJointZeroed = {1.0, 0.0, 0.0, 0.0};

bool imuOk = false;
float rawThetaDeg = 0.0;
float thetaDeg = 0.0;

bool filtReady = false;
float filtThetaDeg = 0.0;
const float THETA_ALPHA = 0.25;

// ======================================================
// Motor and encoder pins
// ======================================================

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

const int FWD = 1;
const int REV = -1;

const int M1_ENC_SIGN = 1;
const int M2_ENC_SIGN = 1;
const int M2_MOT_SIGN = 1;

// Change this to REV if the identification pulse moves the wrong way.
const int TEST_SIGN = FWD;

// ======================================================
// Identification constants
// ======================================================

const float COUNTS_PER_OUTPUT_REV = 17280.0;
const float R_SPOOL_M = 0.01185;
const int U_DEAD = 150;

const int PWM_LIST[] = {150, 160, 170, 180, 190, 200};
const int PWM_LIST_COUNT = sizeof(PWM_LIST) / sizeof(PWM_LIST[0]);
int pwmIndex = 0;
int selectedPwm = PWM_LIST[0];

const unsigned long START_DELAY_MS = 1000;
const unsigned long PULSE_MS = 600;

const unsigned long PRBS_STEP_MS = 450;
const int PRBS_STEPS[] = {
  0, 160, 0, 180, 0, 170, 0, 200, 0, 160,
  0, 190, 0, 170, 0, 200, 0, 180, 0
};
const int PRBS_STEP_COUNT = sizeof(PRBS_STEPS) / sizeof(PRBS_STEPS[0]);

const float THETA_MIN_DEG = -10.0;
const float THETA_MAX_DEG = 100.0;
const unsigned long MAX_TRIAL_MS = 15000;

const unsigned long LOOP_US = 10000;
const unsigned long DATA_MS = 10;

// ======================================================
// State
// ======================================================

RunMode mode = MODE_IDLE;

unsigned int trialId = 0;
unsigned long lastLoopUs = 0;
unsigned long lastDataMs = 0;
unsigned long trialStartMs = 0;
unsigned long motionStartMs = 0;
unsigned long lastStepMs = 0;
int prbsIndex = 0;

long c0 = 0;
long ct = 0;
float theta0 = 0.0;
float thetaT = 0.0;

int currentDir = 0;
int currentPwm = 0;
int currentUCmd = 0;
float currentUEff = 0.0;

// ======================================================
// Quaternion helpers
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
  Quat out = {(float)q.w(), (float)q.x(), (float)q.y(), (float)q.z()};
  return normQ(out);
}

float angleQDeg(Quat q) {
  q = normQ(q);
  float w = fabs(q.w);
  w = constrain(w, -1.0, 1.0);
  return 2.0 * acos(w) * 180.0 / PI;
}

// ======================================================
// IMU
// ======================================================

bool startImu() {
  Wire.begin();
  Wire1.begin();
  delay(100);

  bool upOk = bnoUpper.begin();
  bool foreOk = bnoForearm.begin();

  if (!upOk) {
    Serial.println("ERROR,upper_imu_not_detected");
  }
  if (!foreOk) {
    Serial.println("ERROR,forearm_imu_not_detected");
  }
  if (!upOk || !foreOk) {
    imuOk = false;
    return false;
  }

  delay(1000);
  bnoUpper.setExtCrystalUse(true);
  bnoForearm.setExtCrystalUse(true);
  imuOk = true;
  zeroImu();
  return true;
}

void readImu() {
  if (!imuOk) {
    return;
  }

  qUpRaw = fromBno(bnoUpper.getQuat());
  qForeRaw = fromBno(bnoForearm.getQuat());

  qUpZeroed = mulQ(conjQ(qUpZero), qUpRaw);
  qForeZeroed = mulQ(conjQ(qForeZero), qForeRaw);
  qJointZeroed = mulQ(conjQ(qUpZeroed), qForeZeroed);

  rawThetaDeg = angleQDeg(qJointZeroed);

  if (!filtReady) {
    filtThetaDeg = rawThetaDeg;
    filtReady = true;
  } else {
    filtThetaDeg = THETA_ALPHA * rawThetaDeg + (1.0 - THETA_ALPHA) * filtThetaDeg;
  }

  thetaDeg = filtThetaDeg;
}

void zeroImu() {
  if (!imuOk) {
    return;
  }

  qUpZero = fromBno(bnoUpper.getQuat());
  qForeZero = fromBno(bnoForearm.getQuat());
  qUpZeroed = {1.0, 0.0, 0.0, 0.0};
  qForeZeroed = {1.0, 0.0, 0.0, 0.0};
  qJointZeroed = {1.0, 0.0, 0.0, 0.0};

  rawThetaDeg = 0.0;
  thetaDeg = 0.0;
  filtThetaDeg = 0.0;
  filtReady = true;

  Serial.println("EVENT,imu_zeroed");
}

// ======================================================
// Motor / encoder helpers
// ======================================================

long countsM1() {
  return M1_ENC_SIGN * enc1.read();
}

long countsM2() {
  return M2_ENC_SIGN * enc2.read();
}

float cableDeltaFromCounts(long deltaC) {
  float deltaPhi = (2.0 * PI / COUNTS_PER_OUTPUT_REV) * (float)deltaC;
  return R_SPOOL_M * deltaPhi;
}

float computeUEff(int uCmd) {
  if (abs(uCmd) <= U_DEAD) {
    return 0.0;
  }
  if (uCmd > 0) {
    return (float)(uCmd - U_DEAD);
  }
  return (float)(uCmd + U_DEAD);
}

void driveM2Signed(int uCmd) {
  int dir = 0;
  int pwm = abs(uCmd);

  if (uCmd > 0) {
    dir = FWD;
  } else if (uCmd < 0) {
    dir = REV;
  }

  pwm = constrain(pwm, 0, 255);
  int actualDir = dir * M2_MOT_SIGN;

  currentDir = actualDir;
  currentPwm = pwm;
  currentUCmd = actualDir * pwm;
  currentUEff = computeUEff(currentUCmd);

  if (pwm <= 0 || actualDir == 0) {
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, 0);
    currentDir = 0;
    currentPwm = 0;
    currentUCmd = 0;
    currentUEff = 0.0;
    return;
  }

  if (actualDir > 0) {
    analogWrite(M2_IN1, pwm);
    analogWrite(M2_IN2, 0);
  } else {
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, pwm);
  }
}

void stopMotor() {
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);
  currentDir = 0;
  currentPwm = 0;
  currentUCmd = 0;
  currentUEff = 0.0;
}

// ======================================================
// Trials
// ======================================================

const char* modeName() {
  switch (mode) {
    case MODE_IDLE: return "idle";
    case MODE_WAIT_SINGLE: return "wait_single";
    case MODE_SINGLE: return "single";
    case MODE_WAIT_PRBS: return "wait_prbs";
    case MODE_PRBS: return "prbs";
    case MODE_MANUAL: return "manual";
  }
  return "unknown";
}

bool unsafeAngle() {
  return thetaDeg <= THETA_MIN_DEG || thetaDeg >= THETA_MAX_DEG;
}

void printResult(bool earlyStop) {
  unsigned long nowMs = millis();
  float T = (nowMs - motionStartMs) / 1000.0;
  if (T <= 0.0) {
    T = 0.001;
  }
ct = countsM2();
  thetaT = thetaDeg;
  long deltaC = ct - c0;
  float deltaL = cableDeltaFromCounts(deltaC);

  Serial.print("RESULT,");
  Serial.print(trialId);
  Serial.print(",");
  Serial.print(modeName());
  Serial.print(",");
  Serial.print(T, 4);
  Serial.print(",");
  Serial.print(currentUCmd);
  Serial.print(",");
  Serial.print(currentUEff, 3);
  Serial.print(",");
  Serial.print(c0);
  Serial.print(",");
  Serial.print(ct);
  Serial.print(",");
  Serial.print(deltaC);
  Serial.print(",");
  Serial.print(deltaL, 8);
  Serial.print(",");
  Serial.print(theta0, 4);
  Serial.print(",");
  Serial.print(thetaT, 4);
  Serial.print(",");
  Serial.println(earlyStop ? 1 : 0);
}

void stopTrial(const char* reason, bool earlyStop) {
  printResult(earlyStop);
  stopMotor();
  mode = MODE_IDLE;

  Serial.print("EVENT,stop,");
  Serial.print(trialId);
  Serial.print(",");
  Serial.println(reason);
}

void startSingleTrial() {
  stopMotor();
  trialId++;
  mode = MODE_WAIT_SINGLE;

  enc1.write(0);
  enc2.write(0);
  readImu();

  trialStartMs = millis();
  motionStartMs = trialStartMs + START_DELAY_MS;
  c0 = countsM2();
  theta0 = thetaDeg;

  Serial.print("EVENT,start_single,");
  Serial.print(trialId);
  Serial.print(",pwm,");
  Serial.print(selectedPwm);
  Serial.print(",delay_ms,");
  Serial.println(START_DELAY_MS);
}

void startPrbsTrial() {
  stopMotor();
  trialId++;
  mode = MODE_WAIT_PRBS;

  enc1.write(0);
  enc2.write(0);
  readImu();

  prbsIndex = 0;
  trialStartMs = millis();
  motionStartMs = trialStartMs + START_DELAY_MS;
  lastStepMs = motionStartMs;
  c0 = countsM2();
  theta0 = thetaDeg;

  Serial.print("EVENT,start_prbs,");
  Serial.print(trialId);
  Serial.print(",steps,");
  Serial.print(PRBS_STEP_COUNT);
  Serial.print(",step_ms,");
  Serial.println(PRBS_STEP_MS);
}

void updateTrial() {
  unsigned long now = millis();

  if (mode == MODE_IDLE || mode == MODE_MANUAL) {
    return;
  }

  if (!imuOk) {
    stopTrial("imu_not_ready", true);
    return;
  }

  if (unsafeAngle()) {
    stopTrial("angle_limit", true);
    return;
  }

  if ((now - trialStartMs) > MAX_TRIAL_MS) {
    stopTrial("max_trial_time", true);
    return;
  }

  if (mode == MODE_WAIT_SINGLE) {
    stopMotor();
    if (now >= motionStartMs) {
      c0 = countsM2();
      theta0 = thetaDeg;
      mode = MODE_SINGLE;
      motionStartMs = now;
      Serial.print("EVENT,motion_start,");
      Serial.println(trialId);
    }
    return;
  }

  if (mode == MODE_SINGLE) {
    driveM2Signed(TEST_SIGN * selectedPwm);
    if ((now - motionStartMs) >= PULSE_MS) {
      stopTrial("single_done", false);
    }
    return;
  }

  if (mode == MODE_WAIT_PRBS) {
    stopMotor();
    if (now >= motionStartMs) {
      c0 = countsM2();
      theta0 = thetaDeg;
      mode = MODE_PRBS;
      lastStepMs = now;
      prbsIndex = 0;
      Serial.print("EVENT,motion_start,");
      Serial.println(trialId);
    }
    return;
  }

  if (mode == MODE_PRBS) {
    if (prbsIndex >= PRBS_STEP_COUNT) {
      stopTrial("prbs_done", false);
      return;
    }

    driveM2Signed(TEST_SIGN * PRBS_STEPS[prbsIndex]);

    if ((now - lastStepMs) >= PRBS_STEP_MS) {
      prbsIndex++;
      lastStepMs = now;
    }
  }
}

// ======================================================
// Serial / logging
// ======================================================

void printData() {
  unsigned long now = millis();
  if ((now - lastDataMs) < DATA_MS) {
    return;
  }
  lastDataMs = now;

  float timeS = 0.0;
  if (trialStartMs > 0) {
    timeS = (now - trialStartMs) / 1000.0;
  } else {
    timeS = now / 1000.0;
  }

  long deltaC = countsM2() - c0;
  float deltaL = cableDeltaFromCounts(deltaC);

  Serial.print("DATA,");
  Serial.print(timeS, 4);
  Serial.print(",");
  Serial.print(thetaDeg, 4);
  Serial.print(",");
  Serial.print(rawThetaDeg, 4);
  Serial.print(",");
  Serial.print(countsM1());
  Serial.print(",");
  Serial.print(countsM2());
  Serial.print(",");
  Serial.print(deltaL, 8);
  Serial.print(",");
  Serial.print(currentUCmd);
  Serial.print(",");
  Serial.print(currentUEff, 3);
  Serial.print(",");
  Serial.print(currentPwm);
  Serial.print(",");
  Serial.print(modeName());
  Serial.print(",");
  Serial.println(trialId);
}

void printMenu() {
  Serial.println();
  Serial.println("========== OPEN-LOOP SYSTEM ID ==========");
  Serial.println("m  : menu");
  Serial.println("z  : zero IMUs and encoders");
  Serial.println("s  : stop immediately");
  Serial.println("v  : cycle selected PWM");
  Serial.println("+  : selected PWM + 5");
  Serial.println("-  : selected PWM - 5");
  Serial.println("n  : run one fixed-PWM pulse trial");
  Serial.println("q  : run PRBS-style pulse sequence");
  Serial.println("a  : manual reverse");
  Serial.println("d  : manual forward");
  Serial.println();
  Serial.println("DATA,time_s,theta_deg,raw_theta_deg,m1_counts,m2_counts,delta_l_m,u_cmd,u_eff,pwm,state,trial_id");
  Serial.println("RESULT,trial_id,mode,T_s,u_cmd,u_eff,c0,ct,delta_c,delta_l_m,theta0_deg,thetat_deg,early_stop");
  Serial.println();
  Serial.print("selectedPwm = ");
  Serial.println(selectedPwm);
  Serial.print("PULSE_MS = ");
  Serial.println(PULSE_MS);
  Serial.print("R_SPOOL_M = ");
  Serial.println(R_SPOOL_M, 5);
  Serial.print("U_DEAD = ");
  Serial.println(U_DEAD);
  Serial.println("=========================================");
  Serial.println();
}

void zeroAll() {
  stopMotor();
  enc1.write(0);
  enc2.write(0);
  zeroImu();
  c0 = countsM2();
  theta0 = thetaDeg;
  mode = MODE_IDLE;
  Serial.println("EVENT,zero_all");
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      continue;
    }

    if (c == 'm' || c == 'M') {
      printMenu();
    } else if (c == 'z' || c == 'Z' || c == 'r' || c == 'R') {
      zeroAll();
    } else if (c == 's' || c == 'S' || c == ' ') {
      stopTrial("user_stop", true);
    } else if (c == 'v' || c == 'V') {
      pwmIndex = (pwmIndex + 1) % PWM_LIST_COUNT;
      selectedPwm = PWM_LIST[pwmIndex];
      Serial.print("EVENT,pwm_selected,");
      Serial.println(selectedPwm);
    } else if (c == '+') {
      selectedPwm = constrain(selectedPwm + 5, 0, 255);
      Serial.print("EVENT,pwm_selected,");
      Serial.println(selectedPwm);
    } else if (c == '-') {
      selectedPwm = constrain(selectedPwm - 5, 0, 255);
      Serial.print("EVENT,pwm_selected,");
      Serial.println(selectedPwm);
    } else if (c == 'n' || c == 'N') {
      startSingleTrial();
    } else if (c == 'q' || c == 'Q') {
      startPrbsTrial();
    } else if (c == 'a' || c == 'A') {
      mode = MODE_MANUAL;
      driveM2Signed(-selectedPwm);
      Serial.println("EVENT,manual_reverse");
    } else if (c == 'd' || c == 'D') {
      mode = MODE_MANUAL;
      driveM2Signed(selectedPwm);
      Serial.println("EVENT,manual_forward");
    } else {
      Serial.print("EVENT,unknown_command,");
      Serial.println(c);
    }
  }
}

// ======================================================
// Arduino
// ======================================================

void setup() {
  Serial.begin(115200);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  stopMotor();

  delay(1500);
  Serial.println("EVENT,boot,open_loop_system_id");

  if (!startImu()) {
    Serial.println("ERROR,imu_start_failed");
  }

  enc1.write(0);
  enc2.write(0);
  c0 = countsM2();

  lastLoopUs = micros();
  lastDataMs = millis();

  printMenu();
}

void loop() {
  handleSerial();

  unsigned long nowUs = micros();
  if ((nowUs - lastLoopUs) >= LOOP_US) {
    lastLoopUs = nowUs;
    readImu();

    if (mode == MODE_MANUAL && unsafeAngle()) {
      stopTrial("manual_angle_limit", true);
    } else {
      updateTrial();
    }
  }

  printData();
}