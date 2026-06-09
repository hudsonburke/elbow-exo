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

// Corrected IMU angles
float yawAngle = 0.0;
float pitchAngle = 0.0;
float rollAngle = 0.0;

// ----------------------
// Motor 1 pins
// ----------------------
#define M1_IN1 5
#define M1_IN2 6
#define M1_ENCA 2   // Encoder 1 Yellow Wire, Uno interrupt pin
#define M1_ENCB 4   // Encoder 1 White Wire

// ----------------------
// Motor 2 pins
// ----------------------
#define M2_IN1 9
#define M2_IN2 10
#define M2_ENCA 3   // Encoder 2 Yellow Wire, Uno interrupt pin
#define M2_ENCB 7   // Encoder 2 White Wire

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
const int OUTPUT_COUNTS_PER_REV = MOTOR_COUNTS_PER_REV_FULL * GEAR_RATIO; // Check this value

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
// Timing
// ----------------------
const unsigned long CONTROL_PERIOD_US = 10000; // 10 ms

// Slower serial output
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
void holdTargetsFor(unsigned long holdTimeMs);

void setMotor1RelativeTarget(float degrees, int direction, int maxPwm);
void setMotor2RelativeTarget(float degrees, int direction, int maxPwm);

void updateBothPositionPID();
void updateMotor1PID(float dt);
void updateMotor2PID(float dt);

bool motor1TargetReached();
bool motor2TargetReached();

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

// IMU function declarations
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

  Serial.println("target1 pos1 error1 target2 pos2 error2 pitchAngle");
  Serial.println("Type r in the Serial Monitor to recalibrate IMU to 0.");

  // ----------------------
  // Motion sequence
  // ----------------------
  delay(5000);
  moveMotor2ByDegrees(220.0, FORWARD, 160);
  holdTargetsFor(3000);
  
  moveMotor2ByDegrees(200.0, REVERSE, 160);
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

  // Make current IMU position equal to 0
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

  // This sign is flipped so your physical +90 becomes +90
  pitchAngle = -angleDifference(event.orientation.y, pitchZero);

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

    long currentPos1 = getMotor1Position();
    long currentPos2 = getMotor2Position();

    long error1 = target1 - currentPos1;
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

    Serial.print("yawAngle:");
    Serial.print(yawAngle);

    Serial.print(" ");

    Serial.print("rollAngle:");
    Serial.println(rollAngle);
  }
}