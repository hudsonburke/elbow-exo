#include <Arduino.h>
#include <math.h>

#include <Wire.h>
#include <Encoder.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// IMU


// Do not name this "imu" because the Adafruit library already has a namespace called imu.
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
#define M1_IN1 5
#define M1_IN2 6
#define M1_ENC_A 2
#define M1_ENC_B 4

// Motor 2
#define M2_IN1 9
#define M2_IN2 10
#define M2_ENC_A 3
#define M2_ENC_B 7

// Encoder objects
Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);

// DIRECTION SETTINGS


const int FORWARD = 1;
const int REVERSE = -1;

// Change these to -1 if encoder counts go the wrong way
const int M1_ENC_SIGN = 1;
const int M2_ENC_SIGN = 1;

// Change these to -1 if motor moves the wrong way
const int M1_MOTOR_SIGN = 1;
const int M2_MOTOR_SIGN = 1;


// ENCODER / GEARBOX SETTINGS


const int COUNTS_PER_MOTOR_REV = 64;
const int GEAR_RATIO = 270;
const int COUNTS_PER_OUTPUT_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;


// PID VALUES


// Motor 1 uses encoder PID
float m1Kp = 0.12;
float m1Kd = 0.012;
float m1Ki = 0.0;

// Motor 2 uses IMU pitch PID
float m2Kp = 4.0;
float m2Kd = 0.15;
float m2Ki = 0.0;


// MOTOR LIMITS

const int MIN_PWM = 125;
const int MAX_PWM = 255;

const int M1_COUNT_TOLERANCE = 10;

const float M2_PITCH_TOLERANCE = 1.0;
const float M2_SLOW_ZONE = 15.0;
const int M2_SLOW_PWM = 130;


// TIME SETTINGS


const unsigned long CONTROL_TIME_US = 10000; // 10 ms
const unsigned long PRINT_TIME_MS = 250;

unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;


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


// FUNCTION DECLARATIONS


void startImu();
void zeroImu();
void readPitch();
float angleDiff(float currentAngle, float zeroAngle);

void moveM1Degrees(float degrees, int direction, int maxPwm);
void moveM2ToPitch(float newTargetPitch, int maxPwm);
void holdTargets(unsigned long holdTimeMs);

void setM1Target(float degrees, int direction, int maxPwm);

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
void printData();


// SETUP


void setup() {
  Serial.begin(9600);
  delay(2000);

  startImu();

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  stopM1();
  stopM2();

  enc1.write(0);
  enc2.write(0);

  m1TargetCounts = readM1Counts();

  readPitch();
  targetPitch = currentPitch;

  lastControlTime = micros();

  Serial.println("TargetDeg | CurrentDeg | ErrorDeg | RawPitch | M1Counts | M2Counts");
  Serial.println("Type r in the Serial Monitor to recalibrate IMU to 0.");

  Serial.println("Waiting 10 seconds before starting motor...");
  delay(10000);

  // Reset time after the 10 second wait
  lastControlTime = micros();

  Serial.println("Starting motion sequence...");

  // Current sequence only uses Motor 2.
  moveM2ToPitch(90.0, 180);
  holdTargets(3000);

  moveM2ToPitch(0.0, 180);
  holdTargets(3000);

  Serial.println();
  Serial.println("Sequence complete. Holding final target.");
  Serial.println();
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

  zeroImu();
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

  lastPitchError = 0.0;
  pitchErrorSum = 0.0;

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


// MOTION COMMANDS


void moveM1Degrees(float degrees, int direction, int maxPwm) {
  setM1Target(degrees, direction, maxPwm);

  while (!m1AtTarget()) {
    updateMotors();
    delay(1);
  }

  brakeM1();
}

void moveM2ToPitch(float newTargetPitch, int maxPwm) {
  readPitch();

  targetPitch = newTargetPitch;
  m2MaxPwm = constrain(maxPwm, MIN_PWM, MAX_PWM);

  pitchErrorSum = 0.0;
  lastPitchError = targetPitch - currentPitch;

  Serial.print("Moving Motor 2 to target: ");
  Serial.println(targetPitch, 2);

  while (!m2AtTarget()) {
    updateMotors();
    delay(1);
  }

  brakeM2();

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

  printData();
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
    return;
  }

  float error = targetPitch - currentPitch;
  float absError = fabs(error);

  if (absError <= M2_PITCH_TOLERANCE) {
    brakeM2();
    pitchErrorSum = 0.0;
    lastPitchError = error;
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

  if (pwmValue > 0 && pwmValue < MIN_PWM) {
    pwmValue = MIN_PWM;
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
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'r' || command == 'R') {
      zeroImu();
    }
  }
}

void printData() {
  if (millis() - lastPrintTime >= PRINT_TIME_MS) {
    lastPrintTime = millis();

    float error = targetPitch - currentPitch;

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
  }
}