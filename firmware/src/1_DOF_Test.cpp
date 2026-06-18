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

float pitchZero = 0.0;
float rawPitch = 0.0;
float currentPitch = 0.0;

// If your IMU reads +90 as -90, keep this as -1.
// If your IMU reads +90 as +90, change this to 1.
const int PITCH_SIGN = -1;


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


// ENCODER / GEARBOX SETTINGS


const int COUNTS_PER_MOTOR_REV = 64;
const int GEAR_RATIO = 270;
const int COUNTS_PER_OUTPUT_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;


// PID VALUES


// Motor 1 uses encoder PID.
float m1Kp = 0.12;
float m1Kd = 0.012;
float m1Ki = 0.0;

// Motor 2 uses IMU pitch PID.
float m2Kp = 1.75;
float m2Kd = 0.0;
float m2Ki = 0.125;

// MOTOR LIMITS


const int MIN_PWM = 125;
const int MAX_PWM = 255;

// Motor 2 needs stronger PWM because the mechanism has load.
const int M2_MIN_PWM = 150;

const int M1_COUNT_TOLERANCE = 10;

const float M2_PITCH_TOLERANCE = 1.0;
const float M2_SLOW_ZONE = 5.0;
const int M2_SLOW_PWM = 150;

// Safety timeout so Motor 2 does not run forever if something is wrong.
const unsigned long M2_MOVE_TIMEOUT_MS = 12000;


// TIME SETTINGS


const unsigned long CONTROL_TIME_US = 10000; // 10 ms
const unsigned long PRINT_TIME_MS = 250;

unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;


// SERIAL CONTROL SETTINGS


// Number key preset targets for Motor 2.
// 0 -> 0 deg, 1 -> 10 deg, ..., 9 -> 90 deg
const float KEY_TARGETS[10] = {
  0.0, 10.0, 20.0, 30.0, 40.0,
  50.0, 60.0, 70.0, 80.0, 90.0
};


// MOTOR 1 CONTROL VARIABLES


long m1TargetCounts = 0;
float m1LastError = 0.0;
float m1ErrorSum = 0.0;
int m1MaxPwm = 220;


// MOTOR 2 CONTROL VARIABLES
 

float targetPitch = 0.0;
float lastPitchError = 0.0;
float pitchErrorSum = 0.0;
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
void zeroImu();
void readPitch();
float angleDiff(float currentAngle, float zeroAngle);

void waitForRecalibration();
void printMenu();

void moveM1Degrees(float degrees, int direction, int maxPwm);
void moveM2ToPitch(float newTargetPitch, int maxPwm);
void holdTargets(unsigned long holdTimeMs);

void setM1Target(float degrees, int direction, int maxPwm);
void setM2Target(float newTargetPitch, int maxPwm);

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

  m1TargetCounts = readM1Counts();

  m2Moving = false;

  lastControlTime = micros();

  Serial.println();
  Serial.println("System ready.");
  Serial.println("Place the mechanism at the zero position.");
  Serial.println("Type r and press Enter to recalibrate the IMU.");
  Serial.println();

  waitForRecalibration();

  readPitch();
  targetPitch = currentPitch;

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

void zeroImu() {
  if (!imuReady) {
    return;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  pitchZero = event.orientation.y;

  rawPitch = 0.0;
  currentPitch = 0.0;

  targetPitch = currentPitch;

  lastPitchError = 0.0;
  pitchErrorSum = 0.0;

  m2Moving = false;
  m2HoldingTarget = false;
  m2ReachedMessagePrinted = false;
  brakeM2();

  Serial.println("IMU recalibrated. Current position is now 0.");
}

void readPitch() {
  if (!imuReady) {
    return;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  rawPitch = angleDiff(event.orientation.y, pitchZero);

  // Example:
  // rawPitch = -90
  // currentPitch = +90
  currentPitch = PITCH_SIGN * rawPitch;
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
  Serial.println("Type a number and press Enter to move Motor 2:");
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
  Serial.println("s = stop Motor 2");
  Serial.println("r = recalibrate IMU to 0");
  Serial.println("m = print this menu again");
  Serial.println();
}


// MOTION COMMANDS


void moveM1Degrees(float degrees, int direction, int maxPwm) {
  setM1Target(degrees, direction, maxPwm);

  while (!m1AtTarget()) {
    updateMotors();
    delay(1);
  }

  brakeM1();
}

// Old blocking Motor 2 command.
// Kept for future automatic sequences.
void moveM2ToPitch(float newTargetPitch, int maxPwm) {
  setM2Target(newTargetPitch, maxPwm);

  while (!m2AtTarget()) {
    updateMotors();
    delay(1);
  }

  brakeM2();
  m2Moving = false;
  m2HoldingTarget = false;
  m2ReachedMessagePrinted = false;

  Serial.print("Motor 2 reached target. CurrentDeg: ");
  Serial.print(currentPitch, 2);

  Serial.print(" | RawPitch: ");
  Serial.print(rawPitch, 2);

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

void setM1Target(float degrees, int direction, int maxPwm) {
  long moveCounts = degreesToCounts(degrees);

  m1TargetCounts += direction * moveCounts;
  m1MaxPwm = constrain(maxPwm, MIN_PWM, MAX_PWM);

  m1ErrorSum = 0.0;
  m1LastError = m1TargetCounts - readM1Counts();
}

// New non-blocking Motor 2 command.
// This starts motion, but does not block the loop.
void setM2Target(float newTargetPitch, int maxPwm) {
  readPitch();

  targetPitch = newTargetPitch;
  m2MaxPwm = constrain(maxPwm, MIN_PWM, MAX_PWM);

  pitchErrorSum = 0.0;
  lastPitchError = targetPitch - currentPitch;

  m2Moving = true;
  m2HoldingTarget = false;
  m2ReachedMessagePrinted = false;
  m2MoveStartTime = millis();

  Serial.println();
  Serial.print("Moving Motor 2 to ");
  Serial.print(targetPitch, 2);
  Serial.println(" degrees.");

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

  // Motor 1 is kept for future use.
  updateM1Pid(dt);

  // Motor 2 PID runs when m2Moving is true.
  // After reaching the target, m2Moving stays true so PID can correct if pushed away.
  updateM2Pid(dt);

  // Print Teleplot/readable values while Motor 2 PID is active.
  if (m2Moving) {
    printData();
  }
}

void updateM1Pid(float dt) {
  long currentCounts = readM1Counts();

  float error = m1TargetCounts - currentCounts;
  float absError = fabs(error);

  if (absError <= M1_COUNT_TOLERANCE) {
    brakeM1();
    m1ErrorSum = 0.0;
    m1LastError = error;
    return;
  }

  float errorChange = (error - m1LastError) / dt;

  m1ErrorSum += error * dt;

  if (m1ErrorSum > 300) {
    m1ErrorSum = 300;
  }

  if (m1ErrorSum < -300) {
    m1ErrorSum = -300;
  }

  float output = m1Kp * error
               + m1Kd * errorChange
               + m1Ki * m1ErrorSum;

  int pwmValue = abs((int)output);

  if (pwmValue > m1MaxPwm) {
    pwmValue = m1MaxPwm;
  }

  if (pwmValue > 0 && pwmValue < MIN_PWM) {
    pwmValue = MIN_PWM;
  }

  int direction = FORWARD;

  if (output < 0) {
    direction = REVERSE;
  }

  direction *= M1_MOTOR_SIGN;

  runM1(direction, pwmValue);

  m1LastError = error;
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

  float error = targetPitch - currentPitch;
  float absError = fabs(error);

  // If Motor 2 is inside the tolerance, hold the target.
  // Important: m2Moving stays true, so PID stays active.
  if (absError <= M2_PITCH_TOLERANCE) {
    brakeM2();

    pitchErrorSum = 0.0;
    lastPitchError = error;

    m2HoldingTarget = true;

    // Refresh timeout while holding so it does not stop after 12 seconds.
    m2MoveStartTime = millis();

    if (!m2ReachedMessagePrinted) {
      Serial.println();
      Serial.println("Motor 2 reached target. PID hold is still active.");
      Serial.print("TargetDeg: ");
      Serial.print(targetPitch, 2);
      Serial.print(" | CurrentDeg: ");
      Serial.print(currentPitch, 2);
      Serial.print(" | ErrorDeg: ");
      Serial.print(error, 2);
      Serial.print(" | RawPitch: ");
      Serial.print(rawPitch, 2);
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

    pitchErrorSum = 0.0;
    lastPitchError = error;

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
    Serial.print("TargetDeg: ");
    Serial.print(targetPitch, 2);
    Serial.print(" | CurrentDeg: ");
    Serial.print(currentPitch, 2);
    Serial.print(" | ErrorDeg: ");
    Serial.println(targetPitch - currentPitch, 2);
    Serial.println("Choose another target or recalibrate with r.");
    Serial.println();

    return;
  }

  float errorChange = (error - lastPitchError) / dt;

  pitchErrorSum += error * dt;

  if (pitchErrorSum > 100) {
    pitchErrorSum = 100;
  }

  if (pitchErrorSum < -100) {
    pitchErrorSum = -100;
  }

  float output = m2Kp * error
               + m2Kd * errorChange
               + m2Ki * pitchErrorSum;

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

  direction *= M2_MOTOR_SIGN;

  runM2(direction, pwmValue);

  lastPitchError = error;
}


// TARGET CHECKS


bool m1AtTarget() {
  long error = m1TargetCounts - readM1Counts();

  return labs(error) <= M1_COUNT_TOLERANCE;
}

bool m2AtTarget() {
  readPitch();

  float error = targetPitch - currentPitch;

  return fabs(error) <= M2_PITCH_TOLERANCE;
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
    setM2Target(KEY_TARGETS[keyNumber], 180);
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

    targetPitch = currentPitch;
    pitchErrorSum = 0.0;
    lastPitchError = 0.0;

    m2Moving = false;
    m2HoldingTarget = false;
    m2ReachedMessagePrinted = false;
    brakeM2();

    Serial.println();
    Serial.print("Motor 2 stopped. Current angle: ");
    Serial.println(targetPitch, 2);
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

    float error = targetPitch - currentPitch;

  
    // Teleplot output
    // Only these two values are plotted.
    
    Serial.print(">targetDeg:");
    Serial.println(targetPitch, 2);

    Serial.print(">currentDeg:");
    Serial.println(currentPitch, 2);

    // Normal readable terminal output

    Serial.print("TargetDeg: ");
    Serial.print(targetPitch, 2);

    Serial.print(" | CurrentDeg: ");
    Serial.print(currentPitch, 2);

    Serial.print(" | ErrorDeg: ");
    Serial.print(error, 2);

    Serial.print(" | RawPitch: ");
    Serial.print(rawPitch, 2);

   

    Serial.print(" | M2Counts: ");
    Serial.println(readM2Counts());
  }
}