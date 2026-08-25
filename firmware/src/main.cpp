#include <Arduino.h>
#include <Encoder.h>
#include <math.h>

const int CPR = 64;       // Counts per revolution
const int MAX_PWM = 255;  // Maximum PWM value

const float UPPER_LENGTH = 87.0f;
const float CUFF_LENGTH = 20.0f;
const float CABLE_REST_LENGTH = UPPER_LENGTH + CUFF_LENGTH;
const float SQUARED_TERM =
    UPPER_LENGTH * UPPER_LENGTH + CUFF_LENGTH * CUFF_LENGTH;
const float TWO_UPPER_CUFF = 2.0f * UPPER_LENGTH * CUFF_LENGTH;

struct DCMotor {
  const int IN1;
  const int IN2;
  Encoder encoder;
  const int ratio;  // Input to output ratio (e.g., 270 for 270:1 gear ratio)
  const float pulleyRadius;  // Radius of the pulley in the same units as length
                             // (e.g., cm)

  const float Kp;
  const float Ki;
  const float Kd;
  float integral;
  long previousError;
  unsigned long previousUpdateMicros;
};

// Kp=5.0, Ki=0.01 (was 1.0 — integral windup caused oscillation)
// Ki reduced by 100× since the loop runs at ~kHz; integral accumulates fast
DCMotor motor1 = {9, 10, Encoder(3, 7), 270, 1.25f, 10.0f, 0.01f, 0.0f, 0,
                  0, 0};
const float COUNTS_PER_LENGTH =
    motor1.ratio * CPR / (2.0f * PI * motor1.pulleyRadius);

// DCMotor motor2 = {6, 7, Encoder(8, 9), 19, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};
// DCMotor motor3 = {10, 11, Encoder(12, 13), 19, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};

DCMotor* motors[] = {&motor1};  //, &motor2, &motor3};

void setupMotor(DCMotor& motor);
int calculateMotorPWM(DCMotor& motor, long targetCounts);
void driveMotor(DCMotor& motor, int pwmValue);
void moveToCount(DCMotor& motor, long int targetCount, int tolerance);
long int elbowAngleToCounts(float angleDegrees) {
  return (CABLE_REST_LENGTH -
          sqrt(SQUARED_TERM - TWO_UPPER_CUFF * cos(radians(angleDegrees)))) *
         COUNTS_PER_LENGTH;
}
float countsToElbowAngle(long int counts) {
  float length = CABLE_REST_LENGTH - (counts / COUNTS_PER_LENGTH);
  float cosAngle =
      (SQUARED_TERM - length * length) / TWO_UPPER_CUFF;  // Law of cosines
  return degrees(acos(cosAngle));
}
void moveToElbowAngle(float angleDegrees, int toleranceDegrees) {
  long int upperBoundCounts =
      elbowAngleToCounts(angleDegrees + toleranceDegrees);
  long int lowerBoundCounts =
      elbowAngleToCounts(angleDegrees - toleranceDegrees);
  long int targetCounts = elbowAngleToCounts(angleDegrees);
  long int toleranceCounts = abs(upperBoundCounts - lowerBoundCounts) / 2;
  if (toleranceCounts < 1) toleranceCounts = 1;

  moveToCount(motor1, targetCounts, toleranceCounts);
}

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    setupMotor(*motors[i]);
  }

  Serial.println("Setup Complete");
  Serial.println("Moving to 90 degrees...");
  moveToElbowAngle(90.0f, 2.0f);
  Serial.println("Done");
}

void loop() {
  long int pos = motor1.encoder.read();
  Serial.print("Resting - Counts: ");
  Serial.print(String(pos));
  Serial.print("  Angle: ");
  Serial.print(String(countsToElbowAngle(pos)));
  Serial.println(" deg");
  delay(500);
}

void setupMotor(DCMotor& motor) {
  pinMode(motor.IN1, OUTPUT);
  pinMode(motor.IN2, OUTPUT);
  motor.encoder.write(0);
  motor.integral = 0.0f;
  motor.previousError = 0;
  motor.previousUpdateMicros = 0;
}

int calculateMotorPWM(DCMotor& motor, long targetCounts) {
  const long error = targetCounts - motor.encoder.read();
  const unsigned long now = micros();

  float dtSeconds = 0.0f;
  if (motor.previousUpdateMicros != 0) {
    dtSeconds = (now - motor.previousUpdateMicros) / 1000000.0f;
  }

  if (dtSeconds > 0.0f) {
    motor.integral += error * dtSeconds;

    if (motor.Ki > 0.0f) {
      const float maxIntegral = MAX_PWM / motor.Ki;
      motor.integral = constrain(motor.integral, -maxIntegral, maxIntegral);
    }
  }

  float derivative = 0.0f;
  if (dtSeconds > 0.0f) {
    derivative = (error - motor.previousError) / dtSeconds;
  }

  const float output =
      motor.Kp * error + motor.Ki * motor.integral + motor.Kd * derivative;

  motor.previousError = error;
  motor.previousUpdateMicros = now;

  return constrain(static_cast<long>(output), -MAX_PWM, MAX_PWM);
}

void driveMotor(DCMotor& motor, int pwmValue) {
  if (pwmValue > 0) {
    digitalWrite(motor.IN2, LOW);
    analogWrite(motor.IN1, min(pwmValue, MAX_PWM));
  } else if (pwmValue < 0) {
    digitalWrite(motor.IN1, LOW);
    analogWrite(motor.IN2, min(-pwmValue, MAX_PWM));
  } else {
    digitalWrite(motor.IN1, LOW);
    digitalWrite(motor.IN2, LOW);
  }
}

void moveToCount(DCMotor& motor, long int targetCounts, int toleranceCounts) {
  long error = targetCounts - motor.encoder.read();
  unsigned long startMicros = micros();
  unsigned long lastLog = 0;
  const unsigned long timeoutMicros = 5000000;

  while (abs(error) > toleranceCounts) {
    if (micros() - startMicros > timeoutMicros) {
      driveMotor(motor, 0);
      motor.integral = 0.0f;
      motor.previousError = 0;
      motor.previousUpdateMicros = 0;
      Serial.println("WARN: moveToCount timed out");
      return;
    }

    driveMotor(motor, calculateMotorPWM(motor, targetCounts));
    error = targetCounts - motor.encoder.read();

    unsigned long now = millis();
    if (now - lastLog >= 200) {
      lastLog = now;
      long int pos = motor.encoder.read();
      Serial.print("Counts: ");
      Serial.print(String(pos));
      Serial.print("  Angle: ");
      Serial.print(String(countsToElbowAngle(pos)));
      Serial.println(" deg");
    }
  }

  driveMotor(motor, 0);
  motor.integral = 0.0f;
  motor.previousError = 0;
  motor.previousUpdateMicros = 0;
  Serial.println("Reached target");
}
