#include <Arduino.h>
#include <math.h>

#include <Wire.h>
#include <Encoder.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool imuReady = false;


// IMU ANGLE VARIABLES

float pitchZero = 0.0;
float rawPitch = 0.0;
float currentPitch = 0.0;

float rollZero = 0.0;
float rawRoll = 0.0;
float currentRoll = 0.0;

float yawZero = 0.0;
float rawYaw = 0.0;
float currentYaw = 0.0;

// These are the values used by the PID.
// They change based on whether you selected pitch, roll, or yaw.
float rawControlAngle = 0.0;
float currentControlAngle = 0.0;

// If an axis reads backwards, flip that sign.
const int PITCH_SIGN = -1;
const int ROLL_SIGN = 1;
const int YAW_SIGN = 1;

enum ImuAxis {
  AXIS_PITCH,
  AXIS_ROLL,
  AXIS_YAW
};

ImuAxis selectedAxis = AXIS_PITCH;
const char* selectedAxisName = "pitch";


// MOTOR PINS

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


// DIRECTION SETTINGS

const int FORWARD = 1;
const int REVERSE = -1;

// Change these to -1 if encoder counts go the wrong way.
const int M1_ENC_SIGN = 1;
const int M2_ENC_SIGN = 1;

// Change these to -1 if motor moves the wrong way.
const int M1_MOTOR_SIGN = 1;
const int M2_MOTOR_SIGN = 1;

// Roll behavior:
// Motor 1 turns the IMU.
// Motor 2 runs opposite to reverse/counter that motion.
const int ROLL_M1_DIRECTION_SIGN = 1;
const int ROLL_M2_DIRECTION_SIGN = -1;


// ENCODER / GEARBOX SETTINGS

const int COUNTS_PER_MOTOR_REV = 64;
const int GEAR_RATIO = 270;
const int COUNTS_PER_OUTPUT_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;


// PID VALUES

// Motor 1 selected-axis PID.
float m1Kp = 1.75;
float m1Kd = 0.0;
float m1Ki = 0.125;

// Motor 2 selected-axis PID.
float m2Kp = 1.75;
float m2Kd = 0.0;
float m2Ki = 0.125;


// MOTOR LIMITS

const int MIN_PWM = 125;
const int MAX_PWM = 255;

const int M1_MIN_PWM = 150;
const int M2_MIN_PWM = 150;

const float M1_ANGLE_TOLERANCE = 1.0;
const float M1_SLOW_ZONE = 5.0;
const int M1_SLOW_PWM = 150;

const float M2_ANGLE_TOLERANCE = 1.0;
const float M2_SLOW_ZONE = 5.0;
const int M2_SLOW_PWM = 150;

// Timeout only applies while trying to reach or correct the target.
// When the motor is holding at the target, the timeout refreshes.
const unsigned long M1_MOVE_TIMEOUT_MS = 12000;
const unsigned long M2_MOVE_TIMEOUT_MS = 12000;


// TIME SETTINGS

const unsigned long CONTROL_TIME_US = 10000; // 10 ms
const unsigned long PRINT_TIME_MS = 250;

unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;


// SERIAL CONTROL SETTINGS

// Number key preset targets.
// 0 -> 0 deg, 1 -> 10 deg, ..., 9 -> 90 deg
const float KEY_TARGETS[10] = {
  0.0, 10.0, 20.0, 30.0, 40.0,
  50.0, 60.0, 70.0, 80.0, 90.0
};


// MOTOR 1 CONTROL VARIABLES

float m1TargetAngle = 0.0;
float m1LastAngleError = 0.0;
float m1AngleErrorSum = 0.0;
int m1MaxPwm = 225;

bool m1Moving = false;
bool m1HoldingTarget = false;
bool m1ReachedMessagePrinted = false;

unsigned long m1MoveStartTime = 0;


// MOTOR 2 CONTROL VARIABLES

float targetAngle = 0.0;
float lastAngleError = 0.0;
float angleErrorSum = 0.0;
int m2MaxPwm = 225;

bool m2Moving = false;
bool m2HoldingTarget = false;
bool m2ReachedMessagePrinted = false;

unsigned long m2MoveStartTime = 0;


// FUNCTION DECLARATIONS

void startImu();
void chooseImuAxis();
void printAxisMenu();
void setImuAxis(char command);
void resetTargetsToCurrentAngle();
void zeroImu();
void readImuAngles();
void readPitch();

float angleDiff(float currentAngle, float zeroAngle);
float controlError(float targetValue, float currentValue);

void waitForRecalibration();
void printMenu();

void moveM1Degrees(float degrees, int direction, int maxPwm);
void moveM1ToAngle(float newTargetAngle, int maxPwm);
void moveM2ToAngle(float newTargetAngle, int maxPwm);
void holdTargets(unsigned long holdTimeMs);

void setM1Target(float newTargetAngle, int maxPwm);
void setM2Target(float newTargetAngle, int maxPwm);

void updateMotors();
void updateM1Pid(float dt);
void updateM2Pid(float dt);

bool m1AtTarget();
bool m2AtTarget();

long readM1Counts();
long readM2Counts();

long degreesToCounts(float degrees);
float countsToDegrees(long counts);

void runM1(int direction, int pwmValue);
void runM2(int direction, int pwmValue);

void stopM1();
void stopM2();
void brakeM1();
void brakeM2();

void checkSerial();
void handleKeyCommand(char command);
void printData();


// SETUP

void setup() {
  Serial.begin(9600);
  delay(2000);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  stopM1();
  stopM2();

  enc1.write(0);
  enc2.write(0);

  startImu();

  m1Moving = false;
  m2Moving = false;

  lastControlTime = micros();

  Serial.println();
  Serial.println("System ready.");

  chooseImuAxis();

  Serial.println();
  Serial.println("Place the mechanism at the zero position.");
  Serial.print("Selected IMU axis: ");
  Serial.println(selectedAxisName);
  Serial.println("Type r and press Enter to recalibrate the selected IMU axis to 0.");
  Serial.println();

  waitForRecalibration();

  readPitch();

  m1TargetAngle = currentControlAngle;
  targetAngle = currentControlAngle;

  printMenu();

  lastControlTime = micros();
}


// LOOP

void loop() {
  updateMotors();
}


// IMU FUNCTIONS

void startImu() {
  Serial.println("Starting BNO055 IMU...");

  Wire.begin();

  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring or I2C address.");
    imuReady = false;
    return;
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  imuReady = true;
  Serial.println("BNO055 detected!");

  delay(500);
}

void chooseImuAxis() {
  bool axisChosen = false;

  printAxisMenu();

  while (!axisChosen) {
    if (Serial.available() > 0) {
      char command = Serial.read();

      if (command == '\n' || command == '\r') {
        continue;
      }

      if (command == 'p' || command == 'P' ||
          command == 'y' || command == 'Y' ||
          command == 'o' || command == 'O') {
        setImuAxis(command);
        axisChosen = true;

        Serial.println();
        Serial.print("IMU axis selected: ");
        Serial.println(selectedAxisName);
        Serial.println();
      }
      else {
        Serial.println("Please choose p, y, or o first.");
      }
    }
  }
}

void printAxisMenu() {
  Serial.println("Choose which IMU axis the motors should control:");
  Serial.println("p = pitch");
  Serial.println("y = yaw");
  Serial.println("o = roll");
  Serial.println("Note: roll uses o because r is used for recalibration.");
  Serial.println();
}

void setImuAxis(char command) {
  if (command == 'p' || command == 'P') {
    selectedAxis = AXIS_PITCH;
    selectedAxisName = "pitch";
  }
  else if (command == 'y' || command == 'Y') {
    selectedAxis = AXIS_YAW;
    selectedAxisName = "yaw";
  }
  else if (command == 'o' || command == 'O') {
    selectedAxis = AXIS_ROLL;
    selectedAxisName = "roll";
  }
}

void resetTargetsToCurrentAngle() {
  readPitch();

  m1TargetAngle = currentControlAngle;
  targetAngle = currentControlAngle;

  m1AngleErrorSum = 0.0;
  m1LastAngleError = 0.0;
  angleErrorSum = 0.0;
  lastAngleError = 0.0;

  m1Moving = false;
  m1HoldingTarget = false;
  m1ReachedMessagePrinted = false;
  stopM1();

  m2Moving = false;
  m2HoldingTarget = false;
  m2ReachedMessagePrinted = false;
  stopM2();
}

void zeroImu() {
  if (!imuReady) {
    return;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  // BNO055 orientation mapping:
  // x = yaw / heading
  // y = pitch
  // z = roll
  yawZero = event.orientation.x;
  pitchZero = event.orientation.y;
  rollZero = event.orientation.z;

  rawYaw = 0.0;
  currentYaw = 0.0;

  rawPitch = 0.0;
  currentPitch = 0.0;

  rawRoll = 0.0;
  currentRoll = 0.0;

  rawControlAngle = 0.0;
  currentControlAngle = 0.0;

  resetTargetsToCurrentAngle();

  Serial.print("IMU recalibrated. Selected axis is ");
  Serial.print(selectedAxisName);
  Serial.println(" and current position is now 0.");
}

void readImuAngles() {
  if (!imuReady) {
    return;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  rawYaw = angleDiff(event.orientation.x, yawZero);
  rawPitch = angleDiff(event.orientation.y, pitchZero);
  rawRoll = angleDiff(event.orientation.z, rollZero);

  currentYaw = YAW_SIGN * rawYaw;
  currentPitch = PITCH_SIGN * rawPitch;
  currentRoll = ROLL_SIGN * rawRoll;

  if (selectedAxis == AXIS_PITCH) {
    rawControlAngle = rawPitch;
    currentControlAngle = currentPitch;
  }
  else if (selectedAxis == AXIS_YAW) {
    rawControlAngle = rawYaw;
    currentControlAngle = currentYaw;
  }
  else {
    rawControlAngle = rawRoll;
    currentControlAngle = currentRoll;
  }
}

// Kept so older parts of the code can still call readPitch().
void readPitch() {
  readImuAngles();
}

float angleDiff(float currentAngle, float zeroAngle) {
  float difference = currentAngle - zeroAngle;

  while (difference > 180.0) {
    difference -= 360.0;
  }

  while (difference < -180.0) {
    difference += 360.0;
  }

  return difference;
}

float controlError(float targetValue, float currentValue) {
  return angleDiff(targetValue, currentValue);
}


// STARTUP MENU

void waitForRecalibration() {
  bool recalibrated = false;

  while (!recalibrated) {
    if (Serial.available() > 0) {
      char command = Serial.read();

      if (command == '\n' || command == '\r') {
        continue;
      }

      if (command == 'r' || command == 'R') {
        zeroImu();
        Serial.println("Calibration complete.");
        Serial.println();
        recalibrated = true;
      }
      else {
        Serial.println("Please type r and press Enter to recalibrate first.");
      }
    }
  }
}

void printMenu() {
  Serial.println("Serial control mode is ON.");
  Serial.print("Active IMU axis: ");
  Serial.println(selectedAxisName);
  Serial.println("Type a number and press Enter to move to that selected-axis angle:");
  Serial.println("Pitch mode: Motor 2 controls pitch. Motor 1 stays stopped.");
  Serial.println("Roll mode: Motor 1 and Motor 2 control roll in opposite directions.");
  Serial.println("Yaw mode: current shared behavior is used until you tune yaw separately.");
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
  Serial.println("p = switch control axis to pitch");
  Serial.println("y = switch control axis to yaw");
  Serial.println("o = switch control axis to roll");
  Serial.println("s = stop Motor 1 and Motor 2");
  Serial.println("r = recalibrate IMU to 0");
  Serial.println("m = print this menu again");
  Serial.println();
}


// MOTION COMMANDS

void moveM1Degrees(float degrees, int direction, int maxPwm) {
  readPitch();

  float newTargetAngle = currentControlAngle + (direction * degrees);
  setM1Target(newTargetAngle, maxPwm);

  while (!m1AtTarget()) {
    updateMotors();
    delay(1);
  }

  brakeM1();
  m1Moving = false;
  m1HoldingTarget = false;
  m1ReachedMessagePrinted = false;

  Serial.print("Motor 1 reached target. CurrentDeg: ");
  Serial.print(currentControlAngle, 2);

  Serial.print(" | RawAxis: ");
  Serial.print(rawControlAngle, 2);

  Serial.print(" | M1Counts: ");
  Serial.println(readM1Counts());
}

void moveM1ToAngle(float newTargetAngle, int maxPwm) {
  setM1Target(newTargetAngle, maxPwm);

  while (!m1AtTarget()) {
    updateMotors();
    delay(1);
  }

  brakeM1();
  m1Moving = false;
  m1HoldingTarget = false;
  m1ReachedMessagePrinted = false;

  Serial.print("Motor 1 reached target. CurrentDeg: ");
  Serial.print(currentControlAngle, 2);

  Serial.print(" | RawAxis: ");
  Serial.print(rawControlAngle, 2);

  Serial.print(" | M1Counts: ");
  Serial.println(readM1Counts());
}

void moveM2ToAngle(float newTargetAngle, int maxPwm) {
  setM2Target(newTargetAngle, maxPwm);

  while (!m2AtTarget()) {
    updateMotors();
    delay(1);
  }

  brakeM2();
  m2Moving = false;
  m2HoldingTarget = false;
  m2ReachedMessagePrinted = false;

  Serial.print("Motor 2 reached target. CurrentDeg: ");
  Serial.print(currentControlAngle, 2);

  Serial.print(" | RawAxis: ");
  Serial.print(rawControlAngle, 2);

  Serial.print(" | M2Counts: ");
  Serial.println(readM2Counts());
}

void holdTargets(unsigned long holdTimeMs) {
  unsigned long startTime = millis();

  while (millis() - startTime < holdTimeMs) {
    updateMotors();
    delay(1);
  }
}

void setM1Target(float newTargetAngle, int maxPwm) {
  readPitch();

  m1TargetAngle = newTargetAngle;
  m1MaxPwm = constrain(maxPwm, MIN_PWM, MAX_PWM);

  m1AngleErrorSum = 0.0;
  m1LastAngleError = controlError(m1TargetAngle, currentControlAngle);

  m1Moving = true;
  m1HoldingTarget = false;
  m1ReachedMessagePrinted = false;
  m1MoveStartTime = millis();

  Serial.println();
  Serial.print("Moving Motor 1 to ");
  Serial.print(m1TargetAngle, 2);
  Serial.print(" degrees using IMU ");
  Serial.print(selectedAxisName);
  Serial.println(".");
  Serial.println();
}

void setM2Target(float newTargetAngle, int maxPwm) {
  readPitch();

  targetAngle = newTargetAngle;
  m2MaxPwm = constrain(maxPwm, MIN_PWM, MAX_PWM);

  angleErrorSum = 0.0;
  lastAngleError = controlError(targetAngle, currentControlAngle);

  m2Moving = true;
  m2HoldingTarget = false;
  m2ReachedMessagePrinted = false;
  m2MoveStartTime = millis();

  Serial.println();
  Serial.print("Moving Motor 2 to ");
  Serial.print(targetAngle, 2);
  Serial.print(" degrees using IMU ");
  Serial.print(selectedAxisName);
  Serial.println(".");
  Serial.println("Teleplot is plotting targetDeg and currentDeg.");
  Serial.println("Readable data will also print below:");
  Serial.println();
}


// PID CONTROL

void updateMotors() {
  checkSerial();

  unsigned long now = micros();

  if (now - lastControlTime < CONTROL_TIME_US) {
    return;
  }

  float dt = (now - lastControlTime) / 1000000.0;
  lastControlTime = now;

  if (dt <= 0) {
    dt = 0.001;
  }

  readPitch();

  updateM1Pid(dt);
  updateM2Pid(dt);

  if (m1Moving || m2Moving) {
    printData();
  }
}

void updateM1Pid(float dt) {
  if (!imuReady) {
    stopM1();
    m1Moving = false;
    m1HoldingTarget = false;
    m1ReachedMessagePrinted = false;
    return;
  }

  // Pitch mode is intentionally Motor 2 only.
  // Motor 2 pitch behavior is the one that already works.
  if (selectedAxis == AXIS_PITCH) {
    stopM1();
    m1Moving = false;
    m1HoldingTarget = false;
    m1ReachedMessagePrinted = false;
    return;
  }

  // No command yet: motor should be fully off.
  if (!m1Moving) {
    stopM1();
    return;
  }

  float error = controlError(m1TargetAngle, currentControlAngle);
  float absError = fabs(error);

  // At target: hold/brake, but keep PID monitoring active.
  if (absError <= M1_ANGLE_TOLERANCE) {
    brakeM1();

    m1AngleErrorSum = 0.0;
    m1LastAngleError = error;

    m1HoldingTarget = true;

    // Refresh timeout while holding.
    m1MoveStartTime = millis();

    if (!m1ReachedMessagePrinted) {
      Serial.println();
      Serial.println("Motor 1 reached target. PID hold is active.");
      Serial.print("Axis: ");
      Serial.print(selectedAxisName);
      Serial.print(" | TargetDeg: ");
      Serial.print(m1TargetAngle, 2);
      Serial.print(" | CurrentDeg: ");
      Serial.print(currentControlAngle, 2);
      Serial.print(" | ErrorDeg: ");
      Serial.print(error, 2);
      Serial.print(" | RawAxis: ");
      Serial.print(rawControlAngle, 2);
      Serial.print(" | M1Counts: ");
      Serial.print(readM1Counts());
      Serial.print(" | M2Counts: ");
      Serial.println(readM2Counts());
      Serial.println("If the mechanism is pushed away, Motor 1 PID will correct it.");
      Serial.println();

      m1ReachedMessagePrinted = true;
    }

    return;
  }

  // If it was holding and got pushed away, restart correction.
  if (m1HoldingTarget) {
    m1HoldingTarget = false;
    m1ReachedMessagePrinted = false;

    m1AngleErrorSum = 0.0;
    m1LastAngleError = error;

    m1MoveStartTime = millis();

    Serial.println();
    Serial.println("Motor 1 moved away from target. PID is correcting.");
    Serial.println();
  }

  // Timeout: fully off, not hold/brake.
  if (millis() - m1MoveStartTime > M1_MOVE_TIMEOUT_MS) {
    stopM1();

    m1Moving = false;
    m1HoldingTarget = false;
    m1ReachedMessagePrinted = false;

    Serial.println();
    Serial.println("Motor 1 move timed out. Motor is fully OFF.");
    Serial.print("Axis: ");
    Serial.print(selectedAxisName);
    Serial.print(" | TargetDeg: ");
    Serial.print(m1TargetAngle, 2);
    Serial.print(" | CurrentDeg: ");
    Serial.print(currentControlAngle, 2);
    Serial.print(" | ErrorDeg: ");
    Serial.println(controlError(m1TargetAngle, currentControlAngle), 2);
    Serial.println("Choose another target or recalibrate with r.");
    Serial.println();

    return;
  }

  float errorChange = (error - m1LastAngleError) / dt;

  m1AngleErrorSum += error * dt;

  if (m1AngleErrorSum > 100) {
    m1AngleErrorSum = 100;
  }

  if (m1AngleErrorSum < -100) {
    m1AngleErrorSum = -100;
  }

  float output = m1Kp * error
               + m1Kd * errorChange
               + m1Ki * m1AngleErrorSum;

  int pwmValue = abs((int)output);

  int pwmLimit = m1MaxPwm;

  if (absError <= M1_SLOW_ZONE) {
    pwmLimit = M1_SLOW_PWM;
  }

  if (pwmValue > pwmLimit) {
    pwmValue = pwmLimit;
  }

  if (pwmValue > 0 && pwmValue < M1_MIN_PWM) {
    pwmValue = M1_MIN_PWM;
  }

  int direction = FORWARD;

  if (output < 0) {
    direction = REVERSE;
  }

  if (selectedAxis == AXIS_ROLL) {
    direction *= ROLL_M1_DIRECTION_SIGN;
  }

  direction *= M1_MOTOR_SIGN;

  runM1(direction, pwmValue);

  m1LastAngleError = error;
}

void updateM2Pid(float dt) {
  if (!imuReady) {
    stopM2();

    m2Moving = false;
    m2HoldingTarget = false;
    m2ReachedMessagePrinted = false;
    return;
  }

  // No command yet: motor should be fully off.
  if (!m2Moving) {
    stopM2();
    return;
  }

  float error = controlError(targetAngle, currentControlAngle);
  float absError = fabs(error);

  // At target: hold/brake, but keep PID monitoring active.
  if (absError <= M2_ANGLE_TOLERANCE) {
    brakeM2();

    angleErrorSum = 0.0;
    lastAngleError = error;

    m2HoldingTarget = true;

    // Refresh timeout while holding.
    m2MoveStartTime = millis();

    if (!m2ReachedMessagePrinted) {
      Serial.println();
      Serial.println("Motor 2 reached target. PID hold is active.");
      Serial.print("Axis: ");
      Serial.print(selectedAxisName);
      Serial.print(" | TargetDeg: ");
      Serial.print(targetAngle, 2);
      Serial.print(" | CurrentDeg: ");
      Serial.print(currentControlAngle, 2);
      Serial.print(" | ErrorDeg: ");
      Serial.print(error, 2);
      Serial.print(" | RawAxis: ");
      Serial.print(rawControlAngle, 2);
      Serial.print(" | M1Counts: ");
      Serial.print(readM1Counts());
      Serial.print(" | M2Counts: ");
      Serial.println(readM2Counts());
      Serial.println("If the mechanism is pushed away, PID will correct it.");
      Serial.println();

      m2ReachedMessagePrinted = true;
    }

    return;
  }

  // If it was holding and got pushed away, restart correction.
  if (m2HoldingTarget) {
    m2HoldingTarget = false;
    m2ReachedMessagePrinted = false;

    angleErrorSum = 0.0;
    lastAngleError = error;

    m2MoveStartTime = millis();

    Serial.println();
    Serial.println("Motor 2 moved away from target. PID is correcting.");
    Serial.println();
  }

  // Timeout: fully off, not hold/brake.
  if (millis() - m2MoveStartTime > M2_MOVE_TIMEOUT_MS) {
    stopM2();

    m2Moving = false;
    m2HoldingTarget = false;
    m2ReachedMessagePrinted = false;

    Serial.println();
    Serial.println("Motor 2 move timed out. Motor is fully OFF.");
    Serial.print("Axis: ");
    Serial.print(selectedAxisName);
    Serial.print(" | TargetDeg: ");
    Serial.print(targetAngle, 2);
    Serial.print(" | CurrentDeg: ");
    Serial.print(currentControlAngle, 2);
    Serial.print(" | ErrorDeg: ");
    Serial.println(controlError(targetAngle, currentControlAngle), 2);
    Serial.println("Choose another target or recalibrate with r.");
    Serial.println();

    return;
  }

  float errorChange = (error - lastAngleError) / dt;

  angleErrorSum += error * dt;

  if (angleErrorSum > 100) {
    angleErrorSum = 100;
  }

  if (angleErrorSum < -100) {
    angleErrorSum = -100;
  }

  float output = m2Kp * error
               + m2Kd * errorChange
               + m2Ki * angleErrorSum;

  int pwmValue = abs((int)output);

  int pwmLimit = m2MaxPwm;

  if (absError <= M2_SLOW_ZONE) {
    pwmLimit = M2_SLOW_PWM;
  }

  if (pwmValue > pwmLimit) {
    pwmValue = pwmLimit;
  }

  if (pwmValue > 0 && pwmValue < M2_MIN_PWM) {
    pwmValue = M2_MIN_PWM;
  }

  int direction = FORWARD;

  if (output < 0) {
    direction = REVERSE;
  }

  // In roll mode, Motor 2 runs opposite Motor 1.
  if (selectedAxis == AXIS_ROLL) {
    direction *= ROLL_M2_DIRECTION_SIGN;
  }

  direction *= M2_MOTOR_SIGN;

  runM2(direction, pwmValue);

  lastAngleError = error;
}


// TARGET CHECKS

bool m1AtTarget() {
  readPitch();

  float error = controlError(m1TargetAngle, currentControlAngle);

  return fabs(error) <= M1_ANGLE_TOLERANCE;
}

bool m2AtTarget() {
  readPitch();

  float error = controlError(targetAngle, currentControlAngle);

  return fabs(error) <= M2_ANGLE_TOLERANCE;
}


// ENCODER FUNCTIONS

long readM1Counts() {
  return enc1.read() * M1_ENC_SIGN;
}

long readM2Counts() {
  return enc2.read() * M2_ENC_SIGN;
}

long degreesToCounts(float degrees) {
  return (long)((degrees / 360.0) * COUNTS_PER_OUTPUT_REV);
}

float countsToDegrees(long counts) {
  return ((float)counts / COUNTS_PER_OUTPUT_REV) * 360.0;
}


// MOTOR DRIVER FUNCTIONS

void runM1(int direction, int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);

  if (direction == FORWARD) {
    analogWrite(M1_IN1, pwmValue);
    analogWrite(M1_IN2, 0);
  }
  else if (direction == REVERSE) {
    analogWrite(M1_IN1, 0);
    analogWrite(M1_IN2, pwmValue);
  }
  else {
    stopM1();
  }
}

void runM2(int direction, int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);

  if (direction == FORWARD) {
    analogWrite(M2_IN1, pwmValue);
    analogWrite(M2_IN2, 0);
  }
  else if (direction == REVERSE) {
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, pwmValue);
  }
  else {
    stopM2();
  }
}


// MOTOR OFF VS HOLD FUNCTIONS

// stopM1()/stopM2()
// Fully OFF / coast.
// Both IN pins are LOW.
// Used when idle, stopped by the user, recalibrated, switched axis, or timed out.

void stopM1() {
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);

  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
}

void stopM2() {
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);

  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
}

// brakeM1()/brakeM2()
// Active hold / brake.
// Both IN pins are HIGH.
// Used only after reaching the IMU target, so the motor holds position.

void brakeM1() {
  analogWrite(M1_IN1, 255);
  analogWrite(M1_IN2, 255);
}

void brakeM2() {
  analogWrite(M2_IN1, 255);
  analogWrite(M2_IN2, 255);
}


// SERIAL INPUT / OUTPUT

void checkSerial() {
  while (Serial.available() > 0) {
    char command = Serial.read();

    if (command == '\n' || command == '\r') {
      continue;
    }

    handleKeyCommand(command);
  }
}

void handleKeyCommand(char command) {
  if (command >= '0' && command <= '9') {
    int keyNumber = command - '0';
    float requestedTarget = KEY_TARGETS[keyNumber];

    if (selectedAxis == AXIS_PITCH) {
      // Pitch behavior:
      // Motor 2 controls pitch.
      // Motor 1 stays fully off.
      readPitch();

      m1TargetAngle = currentControlAngle;
      m1AngleErrorSum = 0.0;
      m1LastAngleError = 0.0;

      m1Moving = false;
      m1HoldingTarget = false;
      m1ReachedMessagePrinted = false;

      stopM1();

      setM2Target(requestedTarget, 220);

      Serial.println("Pitch mode active: Motor 2 is controlling pitch. Motor 1 is stopped.");
      Serial.println();
      return;
    }

    if (selectedAxis == AXIS_ROLL) {
      // Roll behavior:
      // Both motors use the same roll target.
      // Motor 2 direction is reversed inside updateM2Pid().
      setM1Target(requestedTarget, 220);
      setM2Target(requestedTarget, 220);

      Serial.println("Roll mode active: Motor 1 turns the IMU and Motor 2 runs opposite to reverse/counter that motion.");
      Serial.println();
      return;
    }

    // Yaw behavior:
    // Current shared behavior until you tune yaw separately.
    setM1Target(requestedTarget, 220);
    setM2Target(requestedTarget, 220);

    Serial.println("Yaw mode active: using current shared behavior for now.");
    Serial.println();
    return;
  }

  if (command == 'p' || command == 'P' ||
      command == 'y' || command == 'Y' ||
      command == 'o' || command == 'O') {
    setImuAxis(command);
    resetTargetsToCurrentAngle();

    Serial.println();
    Serial.print("Control axis switched to ");
    Serial.print(selectedAxisName);
    Serial.println(".");
    Serial.println("Targets were reset to the current selected-axis angle.");
    Serial.println("Choose a number target when ready.");
    Serial.println();

    return;
  }

  if (command == 'r' || command == 'R') {
    zeroImu();

    Serial.println("Target reset to current angle after IMU zero.");
    Serial.println("Choose a number target when ready.");
    Serial.println();

    return;
  }

  if (command == 's' || command == 'S') {
    readPitch();

    m1TargetAngle = currentControlAngle;
    m1AngleErrorSum = 0.0;
    m1LastAngleError = 0.0;

    m1Moving = false;
    m1HoldingTarget = false;
    m1ReachedMessagePrinted = false;

    stopM1();

    targetAngle = currentControlAngle;
    angleErrorSum = 0.0;
    lastAngleError = 0.0;

    m2Moving = false;
    m2HoldingTarget = false;
    m2ReachedMessagePrinted = false;

    stopM2();

    Serial.println();
    Serial.print("Motor 1 and Motor 2 stopped. Current ");
    Serial.print(selectedAxisName);
    Serial.print(" angle: ");
    Serial.println(targetAngle, 2);
    Serial.println("Choose another target.");
    Serial.println();

    return;
  }

  if (command == 'm' || command == 'M') {
    printMenu();
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(command);
}

void printData() {
  if (millis() - lastPrintTime >= PRINT_TIME_MS) {
    lastPrintTime = millis();

    float error = controlError(targetAngle, currentControlAngle);

    // Teleplot output
    Serial.print(">targetDeg:");
    Serial.println(targetAngle, 2);

    Serial.print(">currentDeg:");
    Serial.println(currentControlAngle, 2);

    Serial.print(">errorDeg:");
    Serial.println(error, 2);

    // Readable terminal output
    Serial.print("Axis: ");
    Serial.print(selectedAxisName);

    Serial.print(" | TargetDeg: ");
    Serial.print(targetAngle, 2);

    Serial.print(" | CurrentDeg: ");
    Serial.print(currentControlAngle, 2);

    Serial.print(" | ErrorDeg: ");
    Serial.print(error, 2);

    Serial.print(" | M1Counts: ");
    Serial.print(readM1Counts());

    Serial.print(" | M2Counts: ");
    Serial.println(readM2Counts());
  }
}