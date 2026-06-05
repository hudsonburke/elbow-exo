#include <Arduino.h>
#include <Encoder.h>
#include <vector>

const int CPR = 64;         // Counts per revolution
const int PWM_FREQ = 20000; // PWM frequency in Hz
const int MAX_PWM = 255;    // Maximum PWM value
const int V_MOTOR = 12;     // Motor voltage in volts

struct DCMotor {
  int IN1;
  int IN2;
  Encoder encoder;
  int ratio; // Input to output ratio (e.g., 270 for 270:1 gear ratio)

  float Kp;
  float Ki;
  float Kd;
  float integral;
  long previousError;
  unsigned long previousUpdateMicros;
};

DCMotor motor1 = {2, 3, Encoder(4, 5), 270, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};
DCMotor motor2 = {6, 7, Encoder(8, 9), 19, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};
DCMotor motor3 = {10, 11, Encoder(12, 13), 19, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};

std::vector<DCMotor> motors = {motor1, motor2, motor3};

void setupMotor(DCMotor &motor);
int calculateMotorPwm(DCMotor &motor, long targetCounts);
void driveMotor(DCMotor &motor, int pwmValue);
void setPosition(DCMotor &motor, int targetPosition, int tolerance);

void setup() {
  for (int i = 0; i < motors.size(); i++) {
    setupMotor(motors[i]);
  }
}

void loop() {}

void setupMotor(DCMotor &motor) {
  pinMode(motor.IN1, OUTPUT);
  pinMode(motor.IN2, OUTPUT);
  motor.encoder.write(0); // Reset encoder position
  motor.integral = 0.0f;
  motor.previousError = 0;
  motor.previousUpdateMicros = 0;
}

int calculateMotorPwm(DCMotor &motor, long targetCounts) {
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

  return constrain(static_cast<int>(output), -MAX_PWM, MAX_PWM);
}

void driveMotor(DCMotor &motor, int pwmValue) {
  if (pwmValue > 0) {
    digitalWrite(motor.IN1, HIGH);
    digitalWrite(motor.IN2, LOW);
    analogWrite(motor.IN1, min(pwmValue, MAX_PWM));
  } else if (pwmValue < 0) {
    digitalWrite(motor.IN1, LOW);
    digitalWrite(motor.IN2, HIGH);
    analogWrite(motor.IN2, min(-pwmValue, MAX_PWM));
  } else {
    digitalWrite(motor.IN1, LOW);
    digitalWrite(motor.IN2, LOW);
  }
}

void setPosition(DCMotor &motor, int targetPosition, int tolerance) {
  const long targetCounts =
      static_cast<long>(targetPosition) * motor.ratio * CPR;
  const long toleranceCounts =
      static_cast<long>(tolerance < 0 ? -tolerance : tolerance) * motor.ratio *
      CPR;
  long error = targetCounts - motor.encoder.read();

  while ((error < 0 ? -error : error) > toleranceCounts) {
    driveMotor(motor, calculateMotorPwm(motor, targetCounts));
    error = targetCounts - motor.encoder.read();
    delay(1);
  }

  driveMotor(motor, 0);
  motor.integral = 0.0f;
  motor.previousError = 0;
  motor.previousUpdateMicros = 0;
}
