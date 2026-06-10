#include <Arduino.h>
#include <math.h>

#include <Wire.h>
#include <Encoder.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// IMU setup
Adafruit_BNO055 imuSensor = Adafruit_BNO055(55, 0x28, &Wire);

bool isImuConnected = false;

// IMU zero/reference values
float yawZeroOffsetDeg = 0.0;
float pitchZeroOffsetDeg = 0.0;
float rollZeroOffsetDeg = 0.0;

// IMU angle values
float yawDeg = 0.0;
float rawPitchDeg = 0.0;
float correctedPitchDeg = 0.0;
float rollDeg = 0.0;

// depending on IMU orientation, you may need to invert the pitch angle
const int PITCH_DIRECTION_SIGN = -1;

// Motor 1 pins

#define MOTOR1_IN1 5
#define MOTOR1_IN2 6
#define MOTOR1_ENCODER_A 2
#define MOTOR1_ENCODER_B 4


// Motor 2 pins

#define MOTOR2_IN1 9
#define MOTOR2_IN2 10
#define MOTOR2_ENCODER_A 3
#define MOTOR2_ENCODER_B 7


// Encoder library objects

Encoder motor1Encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B);
Encoder motor2Encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B);


// Direction constants

const int MOTOR_FORWARD = 1;
const int MOTOR_REVERSE = -1;

// Encoder / gearbox settings

const int MOTOR_COUNTS_PER_REV_FULL = 64;
const int GEAR_RATIO = 270;
const int OUTPUT_COUNTS_PER_REV = MOTOR_COUNTS_PER_REV_FULL * GEAR_RATIO;

const int MOTOR1_ENCODER_DIRECTION_SIGN = 1;
const int MOTOR2_ENCODER_DIRECTION_SIGN = 1;

// Motor 1 still uses encoder PID
const int MOTOR1_OUTPUT_DIRECTION_SIGN = 1;

// If motor 2 moves the wrong direction using IMU PID, change this to -1.
const int MOTOR2_IMU_OUTPUT_DIRECTION_SIGN = 1;


// Motor 1 encoder PID constants

float motor1Kp = 0.12;
float motor1Kd = 0.012;
float motor1Ki = 0.0;

// Motor 2 IMU PID constants

// Error is in degrees, not encoder counts.
float motor2PitchKp = 4.0;
float motor2PitchKd = 0.15;
float motor2PitchKi = 0.0;


// Motor limits

const int MIN_PWM = 125;
const int MAX_PWM = 255;


// Motor 1 encoder tolerance

const int MOTOR1_ENCODER_TOLERANCE_COUNTS = 10;


// Motor 2 IMU angle tolerance

const float MOTOR2_PITCH_TOLERANCE_DEG = 1.0;

// Slow down when close to target angle
const float MOTOR2_PITCH_SLOW_ZONE_DEG = 15.0;
const int MOTOR2_SLOW_PWM = 130;

// Timing

const unsigned long CONTROL_UPDATE_PERIOD_US = 10000; // 10 ms
const unsigned long TELEMETRY_PRINT_INTERVAL_MS = 250;

unsigned long lastControlUpdateMicros = 0;
unsigned long lastTelemetryPrintMillis = 0;


// Motor 1 encoder controller variables

long motor1TargetEncoderCounts = 0;
float motor1PreviousError = 0.0;
float motor1IntegralError = 0.0;
int motor1ActiveMaxPwm = 220;


// Motor 2 IMU pitch controller variables

float motor2TargetPitchDeg = 0.0;
float motor2PreviousPitchError = 0.0;
float motor2IntegralPitchError = 0.0;
int motor2ActiveMaxPwm = 160;

// Function declarations

void moveMotor1ByEncoderDegrees(float degrees, int direction, int maxPwm);
void moveMotor2ToPitchAngle(float targetPitchDeg, int maxPwm);

void holdCurrentTargetsFor(unsigned long holdTimeMs);

void setMotor1RelativeEncoderTarget(float degrees, int direction, int maxPwm);
void setMotor2PitchTarget(float targetPitchDeg, int maxPwm);

void updateMotorControllers();
void updateMotor1EncoderPositionPid(float dt);
void updateMotor2ImuPitchPid(float dt);

bool isMotor1EncoderTargetReached();
bool isMotor2PitchTargetReached();

long getMotor1EncoderCounts();
long getMotor2EncoderCounts();

long motorDegreesToEncoderCounts(float degrees);
float encoderCountsToMotorDegrees(long counts);

void driveMotor1(int direction, int pwmValue);
void driveMotor2(int direction, int pwmValue);

void stopMotor1();
void stopMotor2();
void brakeMotor1();
void brakeMotor2();

void printTelemetry();

float getAngleDifferenceDeg(float currentAngleDeg, float zeroAngleDeg);
void initializeImu();
void zeroImuAtCurrentPosition();
void updateImuAngles();
void handleSerialCommands();

void setup() {
  Serial.begin(9600);
  delay(2000);

  // IMU setup
  
  initializeImu();

  // Motor setup

  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  stopMotor1();
  stopMotor2();


  // Reset encoder counts using Encoder library

  motor1Encoder.write(0);
  motor2Encoder.write(0);

  motor1TargetEncoderCounts = getMotor1EncoderCounts();

  updateImuAngles();
  motor2TargetPitchDeg = correctedPitchDeg;

  lastControlUpdateMicros = micros();

  Serial.println("targetPitchDeg | correctedPitchDeg | pitchErrorDeg | rawPitchDeg | motor2EncoderCounts");
  Serial.println("Type r in the Serial Monitor to recalibrate IMU to 0.");


  // Wait before motor starts

  stopMotor1();
  stopMotor2();

  Serial.println("Waiting 10 seconds before starting motor...");
  delay(10000);

  Serial.println("Starting motion sequence...");

  // Motion sequence using IMU PID control


  moveMotor2ToPitchAngle(90.0, 180);
  holdCurrentTargetsFor(3000);

  moveMotor2ToPitchAngle(0.0, 180);
  holdCurrentTargetsFor(3000);

  Serial.println();
  Serial.println("Sequence complete. Holding final IMU target.");
  Serial.println();
}

void loop() {
  updateMotorControllers();
}


// IMU functions


float getAngleDifferenceDeg(float currentAngleDeg, float zeroAngleDeg) {
  float differenceDeg = currentAngleDeg - zeroAngleDeg;

  while (differenceDeg > 180.0) {
    differenceDeg -= 360.0;
  }

  while (differenceDeg < -180.0) {
    differenceDeg += 360.0;
  }

  return differenceDeg;
}

void initializeImu() {
  Serial.println("Starting BNO055 IMU...");

  Wire.begin();

  if (!imuSensor.begin()) {
    Serial.println("BNO055 not detected. Check wiring or I2C address.");
    isImuConnected = false;
    return;
  }

  delay(1000);
  imuSensor.setExtCrystalUse(true);

  isImuConnected = true;
  Serial.println("BNO055 detected!");

  delay(500);

  zeroImuAtCurrentPosition();
}

void zeroImuAtCurrentPosition() {
  if (!isImuConnected) {
    return;
  }

  sensors_event_t imuEvent;
  imuSensor.getEvent(&imuEvent);

  yawZeroOffsetDeg = imuEvent.orientation.x;
  pitchZeroOffsetDeg = imuEvent.orientation.y;
  rollZeroOffsetDeg = imuEvent.orientation.z;

  yawDeg = 0.0;
  rawPitchDeg = 0.0;
  correctedPitchDeg = 0.0;
  rollDeg = 0.0;
         
  // Do not reset motor2TargetPitchDeg here.
  // motor2TargetPitchDeg should only change inside moveMotor2ToPitchAngle().

  motor2PreviousPitchError = 0.0;
  motor2IntegralPitchError = 0.0;

  Serial.println("IMU recalibrated. Current position is now 0.");
}

void updateImuAngles() {
  if (!isImuConnected) {
    return;
  }

  sensors_event_t imuEvent;
  imuSensor.getEvent(&imuEvent);

  yawDeg = getAngleDifferenceDeg(imuEvent.orientation.x, yawZeroOffsetDeg);

  // Raw pitch after zeroing
  rawPitchDeg = getAngleDifferenceDeg(imuEvent.orientation.y, pitchZeroOffsetDeg);

  // Corrected pitch used for PID control
  // Example: rawPitchDeg = -90, correctedPitchDeg = +90
  correctedPitchDeg = PITCH_DIRECTION_SIGN * rawPitchDeg;

  rollDeg = getAngleDifferenceDeg(imuEvent.orientation.z, rollZeroOffsetDeg);
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'r' || command == 'R') {
      zeroImuAtCurrentPosition();
    }
  }
}


// Motion command functions

void moveMotor1ByEncoderDegrees(float degrees, int direction, int maxPwm) {
  setMotor1RelativeEncoderTarget(degrees, direction, maxPwm);

  while (!isMotor1EncoderTargetReached()) {
    updateMotorControllers();
    delay(1);
  }

  brakeMotor1();
}

void moveMotor2ToPitchAngle(float targetPitchDeg, int maxPwm) {
  updateImuAngles();

  motor2TargetPitchDeg = targetPitchDeg;
  motor2ActiveMaxPwm = constrain(maxPwm, MIN_PWM, MAX_PWM);

  motor2IntegralPitchError = 0.0;
  motor2PreviousPitchError = motor2TargetPitchDeg - correctedPitchDeg;

  Serial.print("Moving motor 2 using IMU PID to pitch target: ");
  Serial.println(motor2TargetPitchDeg, 2);

  while (!isMotor2PitchTargetReached()) {
    updateMotorControllers();
    delay(1);
  }

  brakeMotor2();

  Serial.print("Motor 2 reached IMU pitch target. Final correctedPitchDeg: ");
  Serial.print(correctedPitchDeg, 2);

  Serial.print(" | rawPitchDeg: ");
  Serial.print(rawPitchDeg, 2);

  Serial.print(" | motor2EncoderCounts: ");
  Serial.println(getMotor2EncoderCounts());
}

void holdCurrentTargetsFor(unsigned long holdTimeMs) {
  unsigned long holdStartTime = millis();

  while (millis() - holdStartTime < holdTimeMs) {
    updateMotorControllers();
    delay(1);
  }
}

void setMotor1RelativeEncoderTarget(float degrees, int direction, int maxPwm) {
  long moveCounts = motorDegreesToEncoderCounts(degrees);

  motor1TargetEncoderCounts = motor1TargetEncoderCounts + direction * moveCounts;
  motor1ActiveMaxPwm = constrain(maxPwm, MIN_PWM, MAX_PWM);

  motor1IntegralError = 0.0;
  motor1PreviousError = motor1TargetEncoderCounts - getMotor1EncoderCounts();
}

void setMotor2PitchTarget(float targetPitchDeg, int maxPwm) {
  updateImuAngles();

  motor2TargetPitchDeg = targetPitchDeg;
  motor2ActiveMaxPwm = constrain(maxPwm, MIN_PWM, MAX_PWM);

  motor2IntegralPitchError = 0.0;
  motor2PreviousPitchError = motor2TargetPitchDeg - correctedPitchDeg;
}


// PID update functions


void updateMotorControllers() {
  handleSerialCommands();

  unsigned long nowMicros = micros();

  if (nowMicros - lastControlUpdateMicros < CONTROL_UPDATE_PERIOD_US) {
    return;
  }

  float dt = (nowMicros - lastControlUpdateMicros) / 1000000.0;
  lastControlUpdateMicros = nowMicros;

  if (dt <= 0) {
    dt = 0.001;
  }

  // Motor 2 PID depends on IMU, so update IMU first
  updateImuAngles();

  updateMotor1EncoderPositionPid(dt);
  updateMotor2ImuPitchPid(dt);

  printTelemetry();
}

void updateMotor1EncoderPositionPid(float dt) {
  long motor1CurrentCounts = getMotor1EncoderCounts();

  float motor1ErrorCounts = motor1TargetEncoderCounts - motor1CurrentCounts;
  float motor1AbsErrorCounts = fabs(motor1ErrorCounts);

  if (motor1AbsErrorCounts <= MOTOR1_ENCODER_TOLERANCE_COUNTS) {
    brakeMotor1();
    motor1IntegralError = 0.0;
    motor1PreviousError = motor1ErrorCounts;
    return;
  }

  float motor1ErrorDerivative = (motor1ErrorCounts - motor1PreviousError) / dt;

  motor1IntegralError = motor1IntegralError + motor1ErrorCounts * dt;

  if (motor1IntegralError > 300) {
    motor1IntegralError = 300;
  }

  if (motor1IntegralError < -300) {
    motor1IntegralError = -300;
  }

  float motor1ControlOutput = motor1Kp * motor1ErrorCounts
                            + motor1Kd * motor1ErrorDerivative
                            + motor1Ki * motor1IntegralError;

  int pwmValue = abs((int)motor1ControlOutput);

  if (pwmValue > motor1ActiveMaxPwm) {
    pwmValue = motor1ActiveMaxPwm;
  }

  if (pwmValue > 0 && pwmValue < MIN_PWM) {
    pwmValue = MIN_PWM;
  }

  int motorDirection = MOTOR_FORWARD;

  if (motor1ControlOutput < 0) {
    motorDirection = MOTOR_REVERSE;
  }

  motorDirection = motorDirection * MOTOR1_OUTPUT_DIRECTION_SIGN;

  driveMotor1(motorDirection, pwmValue);

  motor1PreviousError = motor1ErrorCounts;
}

void updateMotor2ImuPitchPid(float dt) {
  if (!isImuConnected) {
    brakeMotor2();
    return;
  }

  float pitchErrorDeg = motor2TargetPitchDeg - correctedPitchDeg;
  float absPitchErrorDeg = fabs(pitchErrorDeg);

  // Motor 2 stops based on IMU angle, not encoder counts.
  if (absPitchErrorDeg <= MOTOR2_PITCH_TOLERANCE_DEG) {
    brakeMotor2();
    motor2IntegralPitchError = 0.0;
    motor2PreviousPitchError = pitchErrorDeg;
    return;
  }

  float pitchErrorDerivative = (pitchErrorDeg - motor2PreviousPitchError) / dt;

  motor2IntegralPitchError = motor2IntegralPitchError + pitchErrorDeg * dt;

  if (motor2IntegralPitchError > 100) {
    motor2IntegralPitchError = 100;
  }

  if (motor2IntegralPitchError < -100) {
    motor2IntegralPitchError = -100;
  }

  float motor2ControlOutput = motor2PitchKp * pitchErrorDeg
                            + motor2PitchKd * pitchErrorDerivative
                            + motor2PitchKi * motor2IntegralPitchError;

  int pwmValue = abs((int)motor2ControlOutput);

  int pwmLimit = motor2ActiveMaxPwm;

  // Slow down close to target
  if (absPitchErrorDeg <= MOTOR2_PITCH_SLOW_ZONE_DEG) {
    pwmLimit = MOTOR2_SLOW_PWM;
  }

  if (pwmValue > pwmLimit) {
    pwmValue = pwmLimit;
  }

  if (pwmValue > 0 && pwmValue < MIN_PWM) {
    pwmValue = MIN_PWM;
  }

  int motorDirection = MOTOR_FORWARD;

  if (motor2ControlOutput < 0) {
    motorDirection = MOTOR_REVERSE;
  }

  motorDirection = motorDirection * MOTOR2_IMU_OUTPUT_DIRECTION_SIGN;

  driveMotor2(motorDirection, pwmValue);

  motor2PreviousPitchError = pitchErrorDeg;
}


// Target checks

bool isMotor1EncoderTargetReached() {
  long motor1ErrorCounts = motor1TargetEncoderCounts - getMotor1EncoderCounts();

  return labs(motor1ErrorCounts) <= MOTOR1_ENCODER_TOLERANCE_COUNTS;
}

bool isMotor2PitchTargetReached() {
  updateImuAngles();

  float pitchErrorDeg = motor2TargetPitchDeg - correctedPitchDeg;

  return fabs(pitchErrorDeg) <= MOTOR2_PITCH_TOLERANCE_DEG;
}

// Conversion functions


long motorDegreesToEncoderCounts(float degrees) {
  return (long)((degrees / 360.0) * OUTPUT_COUNTS_PER_REV);
}

float encoderCountsToMotorDegrees(long counts) {
  return ((float)counts / OUTPUT_COUNTS_PER_REV) * 360.0;
}


// Encoder functions using Paul Stoffregen Encoder library

long getMotor1EncoderCounts() {
  return motor1Encoder.read() * MOTOR1_ENCODER_DIRECTION_SIGN;
}

long getMotor2EncoderCounts() {
  return motor2Encoder.read() * MOTOR2_ENCODER_DIRECTION_SIGN;
}


// Motor control functions


void driveMotor1(int direction, int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);

  if (direction == MOTOR_FORWARD) {
    analogWrite(MOTOR1_IN1, pwmValue);
    analogWrite(MOTOR1_IN2, 0);
  } 
  else if (direction == MOTOR_REVERSE) {
    analogWrite(MOTOR1_IN1, 0);
    analogWrite(MOTOR1_IN2, pwmValue);
  } 
  else {
    stopMotor1();
  }
}

void driveMotor2(int direction, int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);

  if (direction == MOTOR_FORWARD) {
    analogWrite(MOTOR2_IN1, pwmValue);
    analogWrite(MOTOR2_IN2, 0);
  } 
  else if (direction == MOTOR_REVERSE) {
    analogWrite(MOTOR2_IN1, 0);
    analogWrite(MOTOR2_IN2, pwmValue);
  } 
  else {
    stopMotor2();
  }
}

void stopMotor1() {
  analogWrite(MOTOR1_IN1, 0);
  analogWrite(MOTOR1_IN2, 0);
}

void stopMotor2() {
  analogWrite(MOTOR2_IN1, 0);
  analogWrite(MOTOR2_IN2, 0);
}

void brakeMotor1() {
  analogWrite(MOTOR1_IN1, 255);
  analogWrite(MOTOR1_IN2, 255);
}

void brakeMotor2() {
  analogWrite(MOTOR2_IN1, 255);
  analogWrite(MOTOR2_IN2, 255);
}
// Serial output
void printTelemetry() {
  if (millis() - lastTelemetryPrintMillis >= TELEMETRY_PRINT_INTERVAL_MS) {
    lastTelemetryPrintMillis = millis();

    long motor2CurrentCounts = getMotor2EncoderCounts();
    float pitchErrorDeg = motor2TargetPitchDeg - correctedPitchDeg;

    Serial.print("TargetDeg:");
    Serial.print(motor2TargetPitchDeg, 2);

    Serial.print(" |Current Deg: ");
    Serial.print(correctedPitchDeg, 2);

    Serial.print(" |DegreeError: ");
    Serial.print(pitchErrorDeg, 2);
    
    Serial.print(" |M2EncoderCounts: ");
    Serial.println(motor2CurrentCounts);
  }
}