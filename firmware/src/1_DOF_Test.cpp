#include <Arduino.h>
#include <math.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ----------------------
// IMU setup
// ----------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool imuOK = false;

// IMU zero/reference values
float yawZero = 0.0;
float pitchZero = 0.0;
float rollZero = 0.0;

// Raw IMU angle differences
float yawAngle = 0.0;
float rawPitchAngle = 0.0;
float rollAngle = 0.0;

// Corrected pitch angle used for control
float pitchAngle = 0.0;

// If physical +90 reads as raw -90, keep this as -1.
// If physical +90 reads as raw +90, change this to 1.
const int PITCH_SIGN = -1;

// ----------------------
// Motor 1 pins
// ----------------------
#define M1_IN1 5
#define M1_IN2 6
#define M1_ENCA 2
#define M1_ENCB 4

// ----------------------
// Motor 2 pins
// ----------------------
#define M2_IN1 9
#define M2_IN2 10
#define M2_ENCA 3
#define M2_ENCB 7

// ----------------------
// Encoder positions
// ----------------------
volatile long pos1 = 0;
volatile long pos2 = 0;

// ----------------------
// Direction constants
// ----------------------
const int FORWARD = 1;
const int REVERSE = -1;

// ----------------------
// Encoder / gearbox settings
// ----------------------
const int MOTOR_COUNTS_PER_REV_FULL = 64;
const int GEAR_RATIO = 270;
const int OUTPUT_COUNTS_PER_REV = MOTOR_COUNTS_PER_REV_FULL * GEAR_RATIO;

const int ENCODER_SIGN_1 = 1;
const int ENCODER_SIGN_2 = 1;

// Motor 1 still uses encoder PID
const int MOTOR_SIGN_1 = 1;

// If motor 2 moves the wrong way using IMU PID, change this to -1
const int MOTOR2_IMU_SIGN = 1;

// ----------------------
// Motor 1 encoder PID constants
// ----------------------
float kp1 = 0.12;
float kd1 = 0.012;
float ki1 = 0.0;

// ----------------------
// Motor 2 IMU PID constants
// ----------------------
// Error is now in degrees, not encoder counts.
float kpPitch2 = 4.0;
float kdPitch2 = 0.15;
float kiPitch2 = 0.0;

// ----------------------
// Motor limits
// ----------------------
const int MIN_PWM = 125;
const int MAX_PWM = 255;

// ----------------------
// Motor 1 encoder tolerance
// ----------------------
const int TOLERANCE_COUNTS = 10;

// ----------------------
// Motor 2 IMU angle tolerance
// ----------------------
const float IMU_PITCH_TOLERANCE_DEG = 1.0;

// Slow down when close to target angle
const float IMU_SLOW_ZONE_DEG = 15.0;
const int IMU_SLOW_PWM = 130;

// ----------------------
// Timing
// ----------------------
const unsigned long CONTROL_PERIOD_US = 10000; // 10 ms
const unsigned long PLOT_INTERVAL_MS = 250;

unsigned long lastControlTime = 0;
unsigned long lastPlotTime = 0;

// ----------------------
// Motor 1 controller variables
// ----------------------
long target1 = 0;
float eprev1 = 0.0;
float eintegral1 = 0.0;
int activeMaxPwm1 = 220;

// ----------------------
// Motor 2 IMU controller variables
// ----------------------
float targetPitch2 = 0.0;
float eprevPitch2 = 0.0;
float eintegralPitch2 = 0.0;
int activeMaxPwm2 = 160;

// ----------------------
// Function declarations
// ----------------------
void readEncoder1();
void readEncoder2();

void moveMotor1ByDegrees(float degrees, int direction, int maxPwm);
void moveMotor2ToPitch(float targetPitch, int maxPwm);

void holdTargetsFor(unsigned long holdTimeMs);

void setMotor1RelativeTarget(float degrees, int direction, int maxPwm);
void setMotor2PitchTarget(float targetPitch, int maxPwm);

void updateBothPID();
void updateMotor1EncoderPID(float dt);
void updateMotor2IMUPID(float dt);

bool motor1TargetReached();
bool motor2PitchTargetReached();

long getMotor1Position();
long getMotor2Position();

long degreesToCounts(float degrees);
float countsToDegrees(long counts);

void setMotor1(int dir, int pwmVal);
void setMotor2(int dir, int pwmVal);

void stopMotor1();
void stopMotor2();
void brakeMotor1();
void brakeMotor2();

void printData();

float angleDifference(float currentAngle, float zeroAngle);
void setupIMU();
void recalibrateIMU();
void updateIMU();
void checkSerialCommands();

void setup() {
  Serial.begin(9600);
  delay(2000);

  // ----------------------
  // IMU setup
  // ----------------------
  setupIMU();

  // ----------------------
  // Encoder setup
  // ----------------------
  pinMode(M1_ENCA, INPUT_PULLUP);
  pinMode(M1_ENCB, INPUT_PULLUP);

  pinMode(M2_ENCA, INPUT_PULLUP);
  pinMode(M2_ENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_ENCA), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENCA), readEncoder2, RISING);

  // ----------------------
  // Motor setup
  // ----------------------
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  stopMotor1();
  stopMotor2();

  // Reset encoder counts
  noInterrupts();
  pos1 = 0;
  pos2 = 0;
  interrupts();

  target1 = getMotor1Position();

  updateIMU();
  targetPitch2 = pitchAngle;

  lastControlTime = micros();

  Serial.println("targetPitch2 pitchAngle pitchError rawPitch pos2 yawAngle rollAngle");
  Serial.println("Type r in the Serial Monitor to recalibrate IMU to 0.");

  // ----------------------
  // Wait before motor starts
  // ----------------------
  stopMotor1();
  stopMotor2();

  Serial.println("Waiting 10 seconds before starting motor...");
  delay(10000);

  Serial.println("Starting motion sequence...");

  // ----------------------
  // Motion sequence using IMU PID control
  // ----------------------

  // Move motor 2 until IMU pitch reaches 90 degrees
  moveMotor2ToPitch(90.0, 160);
  holdTargetsFor(3000);

  // Move motor 2 back until IMU pitch reaches 0 degrees
  moveMotor2ToPitch(0.0, 160);
  holdTargetsFor(3000);

  Serial.println("Sequence complete. Holding final IMU target.");
}

void loop() {
  updateBothPID();
}

// ----------------------
// IMU functions
// ----------------------

float angleDifference(float currentAngle, float zeroAngle) {
  float diff = currentAngle - zeroAngle;

  while (diff > 180.0) {
    diff -= 360.0;
  }

  while (diff < -180.0) {
    diff += 360.0;
  }

  return diff;
}

void setupIMU() {
  Serial.println("Starting BNO055 IMU...");

  Wire.begin();

  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring or I2C address.");
    imuOK = false;
    return;
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  imuOK = true;
  Serial.println("BNO055 detected!");

  delay(500);

  recalibrateIMU();
}

void recalibrateIMU() {
  if (!imuOK) {
    return;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  yawZero = event.orientation.x;
  pitchZero = event.orientation.y;
  rollZero = event.orientation.z;

  yawAngle = 0.0;
  rawPitchAngle = 0.0;
  pitchAngle = 0.0;
  rollAngle = 0.0;

  // IMPORTANT:
  // Do NOT reset targetPitch2 here.
  // targetPitch2 should only change inside moveMotor2ToPitch().

  eprevPitch2 = 0.0;
  eintegralPitch2 = 0.0;

  Serial.println("IMU recalibrated. Current position is now 0.");
}

void updateIMU() {
  if (!imuOK) {
    return;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  yawAngle = angleDifference(event.orientation.x, yawZero);

  // Raw pitch after zeroing
  rawPitchAngle = angleDifference(event.orientation.y, pitchZero);

  // Corrected pitch used for PID control
  // Example: rawPitch = -90, pitchAngle = +90
  pitchAngle = PITCH_SIGN * rawPitchAngle;

  rollAngle = angleDifference(event.orientation.z, rollZero);
}

void checkSerialCommands() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'r' || command == 'R') {
      recalibrateIMU();
    }
  }
}

// ----------------------
// Encoder interrupt functions
// ----------------------

void readEncoder1() {
  int b = digitalRead(M1_ENCB);

  if (b == HIGH) {
    pos1++;
  } 
  else {
    pos1--;
  }
}

void readEncoder2() {
  int b = digitalRead(M2_ENCB);

  if (b == HIGH) {
    pos2++;
  } 
  else {
    pos2--;
  }
}

// ----------------------
// Motion command functions
// ----------------------

void moveMotor1ByDegrees(float degrees, int direction, int maxPwm) {
  setMotor1RelativeTarget(degrees, direction, maxPwm);

  while (!motor1TargetReached()) {
    updateBothPID();
    delay(1);
  }

  brakeMotor1();
}

void moveMotor2ToPitch(float targetPitch, int maxPwm) {
  updateIMU();

  targetPitch2 = targetPitch;
  activeMaxPwm2 = constrain(maxPwm, MIN_PWM, MAX_PWM);

  eintegralPitch2 = 0.0;
  eprevPitch2 = targetPitch2 - pitchAngle;

  Serial.print("Moving motor 2 using IMU PID to pitch target: ");
  Serial.println(targetPitch2);

  while (!motor2PitchTargetReached()) {
    updateBothPID();
    delay(1);
  }

  brakeMotor2();

  Serial.print("Motor 2 reached IMU pitch target. Final pitchAngle: ");
  Serial.print(pitchAngle);
  Serial.print(" rawPitch: ");
  Serial.print(rawPitchAngle);
  Serial.print(" pos2 counts: ");
  Serial.println(getMotor2Position());
}

void holdTargetsFor(unsigned long holdTimeMs) {
  unsigned long startTime = millis();

  while (millis() - startTime < holdTimeMs) {
    updateBothPID();
    delay(1);
  }
}

void setMotor1RelativeTarget(float degrees, int direction, int maxPwm) {
  long moveCounts = degreesToCounts(degrees);

  target1 = target1 + direction * moveCounts;
  activeMaxPwm1 = constrain(maxPwm, MIN_PWM, MAX_PWM);

  eintegral1 = 0.0;
  eprev1 = target1 - getMotor1Position();
}

void setMotor2PitchTarget(float targetPitch, int maxPwm) {
  updateIMU();

  targetPitch2 = targetPitch;
  activeMaxPwm2 = constrain(maxPwm, MIN_PWM, MAX_PWM);

  eintegralPitch2 = 0.0;
  eprevPitch2 = targetPitch2 - pitchAngle;
}

// ----------------------
// PID update functions
// ----------------------

void updateBothPID() {
  checkSerialCommands();

  unsigned long now = micros();

  if (now - lastControlTime < CONTROL_PERIOD_US) {
    return;
  }

  float dt = (now - lastControlTime) / 1000000.0;
  lastControlTime = now;

  if (dt <= 0) {
    dt = 0.001;
  }

  // Motor 2 PID depends on IMU, so update IMU first
  updateIMU();

  updateMotor1EncoderPID(dt);
  updateMotor2IMUPID(dt);

  printData();
}

void updateMotor1EncoderPID(float dt) {
  long pos = getMotor1Position();

  float error = target1 - pos;
  float absError = fabs(error);

  if (absError <= TOLERANCE_COUNTS) {
    brakeMotor1();
    eintegral1 = 0.0;
    eprev1 = error;
    return;
  }

  float dedt = (error - eprev1) / dt;
  eintegral1 = eintegral1 + error * dt;

  if (eintegral1 > 300) {
    eintegral1 = 300;
  }

  if (eintegral1 < -300) {
    eintegral1 = -300;
  }

  float u = kp1 * error + kd1 * dedt + ki1 * eintegral1;

  int pwm = abs((int)u);

  if (pwm > activeMaxPwm1) {
    pwm = activeMaxPwm1;
  }

  if (pwm > 0 && pwm < MIN_PWM) {
    pwm = MIN_PWM;
  }

  int dir = FORWARD;

  if (u < 0) {
    dir = REVERSE;
  }

  dir = dir * MOTOR_SIGN_1;

  setMotor1(dir, pwm);

  eprev1 = error;
}

void updateMotor2IMUPID(float dt) {
  if (!imuOK) {
    brakeMotor2();
    return;
  }

  float error = targetPitch2 - pitchAngle;
  float absError = fabs(error);

  // Motor 2 stops based on IMU angle, NOT encoder counts
  if (absError <= IMU_PITCH_TOLERANCE_DEG) {
    brakeMotor2();
    eintegralPitch2 = 0.0;
    eprevPitch2 = error;
    return;
  }

  float dedt = (error - eprevPitch2) / dt;

  eintegralPitch2 = eintegralPitch2 + error * dt;

  if (eintegralPitch2 > 100) {
    eintegralPitch2 = 100;
  }

  if (eintegralPitch2 < -100) {
    eintegralPitch2 = -100;
  }

  float u = kpPitch2 * error + kdPitch2 * dedt + kiPitch2 * eintegralPitch2;

  int pwm = abs((int)u);

  int pwmLimit = activeMaxPwm2;

  // Slow down close to target
  if (absError <= IMU_SLOW_ZONE_DEG) {
    pwmLimit = IMU_SLOW_PWM;
  }

  if (pwm > pwmLimit) {
    pwm = pwmLimit;
  }

  if (pwm > 0 && pwm < MIN_PWM) {
    pwm = MIN_PWM;
  }

  int dir = FORWARD;

  if (u < 0) {
    dir = REVERSE;
  }

  dir = dir * MOTOR2_IMU_SIGN;

  setMotor2(dir, pwm);

  eprevPitch2 = error;
}

// ----------------------
// Target checks
// ----------------------

bool motor1TargetReached() {
  long error = target1 - getMotor1Position();
  return labs(error) <= TOLERANCE_COUNTS;
}

bool motor2PitchTargetReached() {
  updateIMU();

  float error = targetPitch2 - pitchAngle;

  return fabs(error) <= IMU_PITCH_TOLERANCE_DEG;
}

// ----------------------
// Conversion functions
// ----------------------

long degreesToCounts(float degrees) {
  return (long)((degrees / 360.0) * OUTPUT_COUNTS_PER_REV);
}

float countsToDegrees(long counts) {
  return ((float)counts / OUTPUT_COUNTS_PER_REV) * 360.0;
}

// ----------------------
// Encoder functions
// ----------------------

long getMotor1Position() {
  long currentPos;

  noInterrupts();
  currentPos = pos1;
  interrupts();

  return currentPos * ENCODER_SIGN_1;
}

long getMotor2Position() {
  long currentPos;

  noInterrupts();
  currentPos = pos2;
  interrupts();

  return currentPos * ENCODER_SIGN_2;
}

// ----------------------
// Motor control functions
// ----------------------

void setMotor1(int dir, int pwmVal) {
  pwmVal = constrain(pwmVal, 0, 255);

  if (dir == FORWARD) {
    analogWrite(M1_IN1, pwmVal);
    analogWrite(M1_IN2, 0);
  } 
  else if (dir == REVERSE) {
    analogWrite(M1_IN1, 0);
    analogWrite(M1_IN2, pwmVal);
  } 
  else {
    stopMotor1();
  }
}

void setMotor2(int dir, int pwmVal) {
  pwmVal = constrain(pwmVal, 0, 255);

  if (dir == FORWARD) {
    analogWrite(M2_IN1, pwmVal);
    analogWrite(M2_IN2, 0);
  } 
  else if (dir == REVERSE) {
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, pwmVal);
  } 
  else {
    stopMotor2();
  }
}

void stopMotor1() {
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);
}

void stopMotor2() {
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);
}

void brakeMotor1() {
  analogWrite(M1_IN1, 255);
  analogWrite(M1_IN2, 255);
}

void brakeMotor2() {
  analogWrite(M2_IN1, 255);
  analogWrite(M2_IN2, 255);
}

// ----------------------
// Serial output
// ----------------------

void printData() {
  if (millis() - lastPlotTime >= PLOT_INTERVAL_MS) {
    lastPlotTime = millis();

    long currentPos2 = getMotor2Position();

    float pitchError = targetPitch2 - pitchAngle;

    Serial.print("targetPitch2:");
    Serial.print(targetPitch2);
    Serial.print(" ");

    Serial.print("pitchAngle:");
    Serial.print(pitchAngle);
    Serial.print(" ");

    Serial.print("pitchError:");
    Serial.print(pitchError);
    Serial.print(" ");

    Serial.print("rawPitch:");
    Serial.print(rawPitchAngle);
    Serial.print(" ");

    Serial.print("pos2:");
    Serial.print(currentPos2);
    Serial.print(" ");

    Serial.print("yawAngle:");
    Serial.print(yawAngle);
    Serial.print(" ");

    Serial.print("rollAngle:");
    Serial.println(rollAngle);
  }
}