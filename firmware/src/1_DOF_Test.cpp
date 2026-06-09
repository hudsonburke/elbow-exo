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

// If your physical +90 reads as raw -90, keep this as -1.
// If your physical +90 reads as raw +90, change this to 1.
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

const int MOTOR_SIGN_1 = 1;
const int MOTOR_SIGN_2 = 1;

// ----------------------
// PID constants
// ----------------------
float kp = 0.12;
float kd = 0.012;
float ki = 0.0;

// ----------------------
// Motor limits
// ----------------------
const int MIN_PWM = 125;
const int MAX_PWM = 255;

// ----------------------
// Position tolerance
// ----------------------
const int TOLERANCE_COUNTS = 10;

// ----------------------
// IMU angle control settings
// ----------------------
const float IMU_PITCH_TOLERANCE_DEG = 1.0;

// If it overshoots past 90, increase this to 2, 3, 5, etc.
const float IMU_STOP_LEAD_DEG = 0.0;

// Slow down when close to target angle
const float IMU_SLOW_ZONE_DEG = 15.0;
const int IMU_SLOW_PWM = 125;

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
// Motor 2 controller variables
// ----------------------
long target2 = 0;
float eprev2 = 0.0;
float eintegral2 = 0.0;
int activeMaxPwm2 = 220;

// ----------------------
// Function declarations
// ----------------------
void readEncoder1();
void readEncoder2();

void moveMotor1ByDegrees(float degrees, int direction, int maxPwm);
void moveMotor2ByDegrees(float degrees, int direction, int maxPwm);
void moveMotor2ToPitch(float targetPitch, int direction, int maxPwm, float encoderSafetyDegrees);

void holdTargetsFor(unsigned long holdTimeMs);

void setMotor1RelativeTarget(float degrees, int direction, int maxPwm);
void setMotor2RelativeTarget(float degrees, int direction, int maxPwm);

void updateBothPositionPID();
void updateMotor1PID(float dt);
void updateMotor2PID(float dt);

bool motor1TargetReached();
bool motor2TargetReached();
bool imuPitchTargetReached(float targetPitch, float startPitch);

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
  target2 = getMotor2Position();

  lastControlTime = micros();

  Serial.println("target2 pos2 error2 pitchAngle rawPitch yawAngle rollAngle");
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
  // Motion sequence using IMU control
  // ----------------------

  // Move until corrected IMU pitch reaches +90 degrees.
  // Encoder movement of 220 degrees is only a safety limit.
  moveMotor2ToPitch(90.0, FORWARD, 160, 220.0);
  holdTargetsFor(3000);

  // Move back until corrected IMU pitch reaches 0 degrees.
  // Encoder movement of 240 degrees is only a safety limit.
  moveMotor2ToPitch(0.0, REVERSE, 160, 240.0);
  holdTargetsFor(3000);

  Serial.println("Sequence complete. Holding final targets.");
}

void loop() {
  updateBothPositionPID();
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

  Serial.println("IMU recalibrated. Current position is now 0.");
}

void updateIMU() {
  if (!imuOK) {
    return;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  yawAngle = angleDifference(event.orientation.x, yawZero);

  // Raw pitch from IMU after zeroing
  rawPitchAngle = angleDifference(event.orientation.y, pitchZero);

  // Corrected pitch used by motor control
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
    updateBothPositionPID();
    delay(1);
  }

  brakeMotor1();
}

void moveMotor2ByDegrees(float degrees, int direction, int maxPwm) {
  setMotor2RelativeTarget(degrees, direction, maxPwm);

  while (!motor2TargetReached()) {
    updateBothPositionPID();
    delay(1);
  }

  brakeMotor2();
}

void moveMotor2ToPitch(float targetPitch, int direction, int maxPwm, float encoderSafetyDegrees) {
  updateIMU();

  float startPitch = pitchAngle;

  long startPos = getMotor2Position();
  long safetyMoveCounts = degreesToCounts(encoderSafetyDegrees);

  // Encoder target is now only a safety limit
  target2 = startPos + direction * safetyMoveCounts;

  activeMaxPwm2 = constrain(maxPwm, MIN_PWM, MAX_PWM);

  eintegral2 = 0.0;
  eprev2 = target2 - getMotor2Position();

  Serial.print("Moving motor 2 to corrected IMU pitch target: ");
  Serial.println(targetPitch);

  while (!imuPitchTargetReached(targetPitch, startPitch)) {
    checkSerialCommands();
    updateIMU();

    float pitchError = targetPitch - pitchAngle;
    float absPitchError = fabs(pitchError);

    // Slow down when close to IMU target
    if (absPitchError <= IMU_SLOW_ZONE_DEG) {
      activeMaxPwm2 = constrain(IMU_SLOW_PWM, MIN_PWM, maxPwm);
    } 
    else {
      activeMaxPwm2 = constrain(maxPwm, MIN_PWM, MAX_PWM);
    }

    // Encoder safety stop
    long encoderError = target2 - getMotor2Position();

    if (labs(encoderError) <= TOLERANCE_COUNTS) {
      Serial.println("Encoder safety limit reached before IMU target.");
      break;
    }

    updateBothPositionPID();
    delay(1);
  }

  brakeMotor2();

  // Lock target to current encoder position so PID does not continue moving
  target2 = getMotor2Position();
  eprev2 = 0.0;
  eintegral2 = 0.0;

  updateIMU();

  Serial.print("Motor 2 stopped by IMU. Final corrected pitchAngle: ");
  Serial.print(pitchAngle);
  Serial.print(" rawPitch: ");
  Serial.println(rawPitchAngle);
}

void holdTargetsFor(unsigned long holdTimeMs) {
  unsigned long startTime = millis();

  while (millis() - startTime < holdTimeMs) {
    updateBothPositionPID();
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

void setMotor2RelativeTarget(float degrees, int direction, int maxPwm) {
  long moveCounts = degreesToCounts(degrees);

  target2 = target2 + direction * moveCounts;
  activeMaxPwm2 = constrain(maxPwm, MIN_PWM, MAX_PWM);

  eintegral2 = 0.0;
  eprev2 = target2 - getMotor2Position();
}

// ----------------------
// PID update functions
// ----------------------

void updateBothPositionPID() {
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

  updateMotor1PID(dt);
  updateMotor2PID(dt);

  updateIMU();
  printData();
}

void updateMotor1PID(float dt) {
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

  float u = kp * error + kd * dedt + ki * eintegral1;

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

void updateMotor2PID(float dt) {
  long pos = getMotor2Position();

  float error = target2 - pos;
  float absError = fabs(error);

  if (absError <= TOLERANCE_COUNTS) {
    brakeMotor2();
    eintegral2 = 0.0;
    eprev2 = error;
    return;
  }

  float dedt = (error - eprev2) / dt;
  eintegral2 = eintegral2 + error * dt;

  if (eintegral2 > 300) {
    eintegral2 = 300;
  }

  if (eintegral2 < -300) {
    eintegral2 = -300;
  }

  float u = kp * error + kd * dedt + ki * eintegral2;

  int pwm = abs((int)u);

  if (pwm > activeMaxPwm2) {
    pwm = activeMaxPwm2;
  }

  if (pwm > 0 && pwm < MIN_PWM) {
    pwm = MIN_PWM;
  }

  int dir = FORWARD;

  if (u < 0) {
    dir = REVERSE;
  }

  dir = dir * MOTOR_SIGN_2;

  setMotor2(dir, pwm);

  eprev2 = error;
}

// ----------------------
// Target checks
// ----------------------

bool motor1TargetReached() {
  long error = target1 - getMotor1Position();
  return labs(error) <= TOLERANCE_COUNTS;
}

bool motor2TargetReached() {
  long error = target2 - getMotor2Position();
  return labs(error) <= TOLERANCE_COUNTS;
}

bool imuPitchTargetReached(float targetPitch, float startPitch) {
  if (targetPitch > startPitch) {
    return pitchAngle >= targetPitch - IMU_STOP_LEAD_DEG;
  }

  if (targetPitch < startPitch) {
    return pitchAngle <= targetPitch + IMU_STOP_LEAD_DEG;
  }

  return fabs(pitchAngle - targetPitch) <= IMU_PITCH_TOLERANCE_DEG;
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
    long error2 = target2 - currentPos2;

    Serial.print("target2:");
    Serial.print(target2);
    Serial.print(" ");

    Serial.print("pos2:");
    Serial.print(currentPos2);
    Serial.print(" ");

    Serial.print("error2:");
    Serial.print(error2);
    Serial.print(" ");

    Serial.print("pitchAngle:");
    Serial.print(pitchAngle);
    Serial.print(" ");

    Serial.print("rawPitch:");
    Serial.print(rawPitchAngle);
    Serial.print(" ");

    Serial.print("yawAngle:");
    Serial.print(yawAngle);
    Serial.print(" ");

    Serial.print("rollAngle:");
    Serial.println(rollAngle);
  }
}