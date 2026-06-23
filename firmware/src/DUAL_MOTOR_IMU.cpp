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

// IMU angle variables.
// The original code only used pitch. This version keeps pitch, roll, and yaw
// so you can choose which IMU axis the motors should control.
float pitchZero = 0.0;
float rawPitch = 0.0;
float currentPitch = 0.0;

float rollZero = 0.0;
float rawRoll = 0.0;
float currentRoll = 0.0;

float yawZero = 0.0;
float rawYaw = 0.0;
float currentYaw = 0.0;

// These two variables are the selected axis used by the PID loops.
// If you choose pitch, these follow pitch.
// If you choose roll, these follow roll.
// If you choose yaw, these follow yaw.
float rawControlAngle = 0.0;
float currentControlAngle = 0.0;

// If your IMU reads +90 as -90 on an axis, change that axis sign.
// Your old pitch value used -1, so pitch remains -1.
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

// Encoder objects
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

// ROLL MODE DIRECTION SETTINGS
// In roll mode, Motor 1 creates the roll motion and Motor 2 runs opposite
// to reverse/counter that motion. If roll correction goes the wrong way,
// flip one of these values from 1 to -1 or -1 to 1.
const int ROLL_M1_DIRECTION_SIGN = 1;
const int ROLL_M2_DIRECTION_SIGN = -1;


// ENCODER / GEARBOX SETTINGS


const int COUNTS_PER_MOTOR_REV = 64;
const int GEAR_RATIO = 270;
const int COUNTS_PER_OUTPUT_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;


// PID VALUES
// Motor 1 uses the same selected IMU-axis PID style as Motor 2.
float m1Kp = 1.75;
float m1Kd = 0.0;
float m1Ki = 0.125;

// Motor 2 uses selected IMU-axis PID.
float m2Kp = 1.75;
float m2Kd = 0.0;
float m2Ki = 0.125;

// MOTOR LIMITS


const int MIN_PWM = 125;
const int MAX_PWM = 255;

// Motor 1 now uses the same minimum PWM style as Motor 2.
const int M1_MIN_PWM = 150;

// Motor 2 needs stronger PWM because the mechanism has load.
const int M2_MIN_PWM = 150;

const float M1_ANGLE_TOLERANCE = 1.0;
const float M1_SLOW_ZONE = 5.0;
const int M1_SLOW_PWM = 150;

const float M2_ANGLE_TOLERANCE = 1.0;
const float M2_SLOW_ZONE = 5.0;
const int M2_SLOW_PWM = 150;

// Safety timeout so Motor 1 does not run forever if something is wrong.
const unsigned long M1_MOVE_TIMEOUT_MS = 12000;

// Safety timeout so Motor 2 does not run forever if something is wrong.
const unsigned long M2_MOVE_TIMEOUT_MS = 12000;


// TIME SETTINGS


const unsigned long CONTROL_TIME_US = 10000; // 10 ms
const unsigned long PRINT_TIME_MS = 250;

unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;


// SERIAL CONTROL SETTINGS


// Number key preset targets for both motors.
// 0 -> 0 deg, 1 -> 10 deg, ..., 9 -> 90 deg
const float KEY_TARGETS[10] = {
  0.0, 10.0, 20.0, 30.0, 40.0,
  50.0, 60.0, 70.0, 80.0, 90.0
};


// MOTOR 1 CONTROL VARIABLES


float m1TargetAngle = 0.0;
float m1LastAngleError = 0.0;
float m1AngleErrorSum = 0.0;
int m1MaxPwm = 180;

// Motor 1 only runs PID when this is true.
bool m1Moving = false;

// Motor 1 hold mode.
// This lets PID stay active after reaching the target.
bool m1HoldingTarget = false;
bool m1ReachedMessagePrinted = false;

unsigned long m1MoveStartTime = 0;


// MOTOR 2 CONTROL VARIABLES
 

float targetAngle = 0.0;
float lastAngleError = 0.0;
float angleErrorSum = 0.0;
int m2MaxPwm = 180;

// Motor 2 only runs PID when this is true.
bool m2Moving = false;

// Motor 2 hold mode.
// This lets PID stay active after reaching the target.
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
  brakeM1();

  m2Moving = false;
  m2HoldingTarget = false;
  m2ReachedMessagePrinted = false;
  brakeM2();
}

void zeroImu() {
  if (!imuReady) {
    return;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  // BNO055 orientation mapping used here:
  // x = yaw / heading
  // y = pitch, kept the same as your old code
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

  // BNO055 orientation mapping used here:
  // x = yaw / heading
  // y = pitch, kept the same as your old code
  // z = roll
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

// Kept so older code can still call readPitch().
// It now reads all IMU angles and updates the selected control angle.
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


// Kept so older automatic sequences can still call moveM1Degrees().
// Motor 1 now moves by the selected IMU angle instead of encoder counts.
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

// Blocking Motor 1 selected-axis command.
// This mirrors the old blocking Motor 2 command, but for Motor 1.
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

// Old blocking Motor 2 command.
// Kept for future automatic sequences.
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

// New non-blocking Motor 1 command.
// This starts motion, but does not block the loop.
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

  Serial.println("Motor 1 is using selected IMU-axis PID.");
  Serial.println();
}

// New non-blocking Motor 2 command.
// This starts motion, but does not block the loop.
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

  // Motor 1 uses selected IMU-axis PID like Motor 2.
  // After reaching the target, m1Moving stays true so PID can correct if pushed away.
  updateM1Pid(dt);

  // Motor 2 PID runs when m2Moving is true.
  // After reaching the target, m2Moving stays true so PID can correct if pushed away.
  updateM2Pid(dt);

  // Print Teleplot/readable values while either IMU PID is active.
  if (m1Moving || m2Moving) {
    printData();
  }
}

void updateM1Pid(float dt) {
  if (!imuReady) {
    brakeM1();
    m1Moving = false;
    m1HoldingTarget = false;
    m1ReachedMessagePrinted = false;
    return;
  }

  // Pitch mode is intentionally Motor 2 only because Motor 2 pitch behavior
  // is already working well. Motor 1 is kept stopped during pitch control.
  if (selectedAxis == AXIS_PITCH) {
    brakeM1();
    m1Moving = false;
    m1HoldingTarget = false;
    m1ReachedMessagePrinted = false;
    return;
  }

  // Do nothing until a number key command starts PID.
  if (!m1Moving) {
    brakeM1();
    return;
  }

  float error = controlError(m1TargetAngle, currentControlAngle);
  float absError = fabs(error);

  // If Motor 1 is inside the tolerance, hold the target.
  // Important: m1Moving stays true, so PID stays active.
  if (absError <= M1_ANGLE_TOLERANCE) {
    brakeM1();

    m1AngleErrorSum = 0.0;
    m1LastAngleError = error;

    m1HoldingTarget = true;

    // Refresh timeout while holding so it does not stop after 12 seconds.
    m1MoveStartTime = millis();

    if (!m1ReachedMessagePrinted) {
      Serial.println();
      Serial.println("Motor 1 reached target. PID hold is still active.");
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

  // If it was holding and now got pushed away, restart PID correction.
  if (m1HoldingTarget) {
    m1HoldingTarget = false;
    m1ReachedMessagePrinted = false;

    m1AngleErrorSum = 0.0;
    m1LastAngleError = error;

    // Give the correction a fresh timeout window.
    m1MoveStartTime = millis();

    Serial.println();
    Serial.println("Motor 1 moved away from target. PID is correcting.");
    Serial.println();
  }

  // Safety timeout only applies while trying to reach or correct the target.
  if (millis() - m1MoveStartTime > M1_MOVE_TIMEOUT_MS) {
    brakeM1();
    m1Moving = false;
    m1HoldingTarget = false;
    m1ReachedMessagePrinted = false;

    Serial.println();
    Serial.println("Motor 1 move timed out. Motor stopped.");
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
    brakeM2();
    m2Moving = false;
    m2HoldingTarget = false;
    m2ReachedMessagePrinted = false;
    return;
  }

  // Do nothing until a number key command starts PID.
  if (!m2Moving) {
    brakeM2();
    return;
  }

  float error = controlError(targetAngle, currentControlAngle);
  float absError = fabs(error);

  // If Motor 2 is inside the tolerance, hold the target.
  // Important: m2Moving stays true, so PID stays active.
  if (absError <= M2_ANGLE_TOLERANCE) {
    brakeM2();

    angleErrorSum = 0.0;
    lastAngleError = error;

    m2HoldingTarget = true;

    // Refresh timeout while holding so it does not stop after 12 seconds.
    m2MoveStartTime = millis();

    if (!m2ReachedMessagePrinted) {
      Serial.println();
      Serial.println("Motor 2 reached target. PID hold is still active.");
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

  // If it was holding and now got pushed away, restart PID correction.
  if (m2HoldingTarget) {
    m2HoldingTarget = false;
    m2ReachedMessagePrinted = false;

    angleErrorSum = 0.0;
    lastAngleError = error;

    // Give the correction a fresh timeout window.
    m2MoveStartTime = millis();

    Serial.println();
    Serial.println("Motor 2 moved away from target. PID is correcting.");
    Serial.println();
  }

  // Safety timeout only applies while trying to reach or correct the target.
  if (millis() - m2MoveStartTime > M2_MOVE_TIMEOUT_MS) {
    brakeM2();
    m2Moving = false;
    m2HoldingTarget = false;
    m2ReachedMessagePrinted = false;

    Serial.println();
    Serial.println("Motor 2 move timed out. Motor stopped.");
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

  // In roll mode, Motor 2 intentionally runs opposite Motor 1.
  // This lets Motor 1 create the roll turn while Motor 2 reverses/counters it.
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

void stopM1() {
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);
}

void stopM2() {
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);
}

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

    // Ignore line endings from Serial Monitor.
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
      // Pitch behavior: keep the working Motor 2 pitch PID behavior.
      // Motor 1 is not used for pitch mode.
      readPitch();
      m1TargetAngle = currentControlAngle;
      m1AngleErrorSum = 0.0;
      m1LastAngleError = 0.0;
      m1Moving = false;
      m1HoldingTarget = false;
      m1ReachedMessagePrinted = false;
      brakeM1();

      setM2Target(requestedTarget, 180);

      Serial.println("Pitch mode active: Motor 2 is controlling pitch. Motor 1 is stopped.");
      Serial.println();
      return;
    }

    if (selectedAxis == AXIS_ROLL) {
      // Roll behavior: both motors use the same roll target, but Motor 2 direction
      // is reversed inside updateM2Pid() using ROLL_M2_DIRECTION_SIGN.
      setM1Target(requestedTarget, 180);
      setM2Target(requestedTarget, 180);

      Serial.println("Roll mode active: Motor 1 turns the IMU and Motor 2 runs opposite to reverse/counter that motion.");
      Serial.println();
      return;
    }

    // Yaw behavior is left as the current shared behavior so it can be tuned later.
    setM1Target(requestedTarget, 180);
    setM2Target(requestedTarget, 180);

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
    brakeM1();

    targetAngle = currentControlAngle;
    angleErrorSum = 0.0;
    lastAngleError = 0.0;

    m2Moving = false;
    m2HoldingTarget = false;
    m2ReachedMessagePrinted = false;
    brakeM2();

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

    float m1Error = controlError(m1TargetAngle, currentControlAngle);
    float m2Error = controlError(targetAngle, currentControlAngle);

  
    // Teleplot output
    // These two values are plotted.
    // In roll mode, Motor 1 and Motor 2 have the same target, but Motor 2 direction is reversed.
    
    Serial.print(">targetDeg:");
    Serial.println(targetAngle, 2);

    Serial.print(">currentDeg:");
    Serial.println(currentControlAngle, 2);

    // Normal readable terminal output

    Serial.print("Axis: ");
    Serial.print(selectedAxisName);

    Serial.print(" | M1TargetDeg: ");
    Serial.print(m1TargetAngle, 2);

    Serial.print(" | M2TargetDeg: ");
    Serial.print(targetAngle, 2);

    Serial.print(" | CurrentDeg: ");
    Serial.print(currentControlAngle, 2);

    Serial.print(" | M1ErrorDeg: ");
    Serial.print(m1Error, 2);

    Serial.print(" | M2ErrorDeg: ");
    Serial.print(m2Error, 2);

    Serial.print(" | RawAxis: ");
    Serial.print(rawControlAngle, 2);

    Serial.print(" | Pitch: ");
    Serial.print(currentPitch, 2);

    Serial.print(" | Roll: ");
    Serial.print(currentRoll, 2);

    Serial.print(" | Yaw: ");
    Serial.print(currentYaw, 2);

    Serial.print(" | M1Counts: ");
    Serial.print(readM1Counts());

    Serial.print(" | M2Counts: ");
    Serial.println(readM2Counts());
  }
}
