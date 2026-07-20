#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// FINAL DUAL-MOTOR, DUAL-IMU CONTROLLER

//
// PURPOSE
// -------
// This program controls two DC motors using feedback from two BNO055 IMUs.
// Each motor has its own PI/PID controller, target angle, encoder reading,
// safety timeout, and oscillation mode.
//
// The program was cleaned for the final product. Test-only features such as
// constant-PWM identification trials, trial IDs, and CSV events are not
// included. Manual arrow-key driving is kept because it is useful for setup,
// positioning, and controlled troubleshooting of the final system.
//
// HARDWARE USED
// -------------
// 1. Upper-arm BNO055 IMU
// 2. Forearm BNO055 IMU
// 3. Motor 1 and its encoder
// 4. Motor 2 and its encoder
// 5. A microcontroller with Wire and Wire1 I2C buses
//
// MAIN CONTROL IDEA
// -----------------
// 1. Read the two IMUs.
// 2. Calculate the upper-arm angle and the relative elbow angle.
// 3. Compare each motor's target angle with its measured angle.
// 4. Use PI/PID control to calculate the motor command.
// 5. Convert the controller output into a PWM value and direction.
// 6. Send the PWM command to the motor driver.
// 7. Repeat this process at 100 Hz.
//
// MOTOR FEEDBACK
// --------------
// Motor 1 currently uses the upper-arm angle.
// Motor 2 currently uses the relative elbow-joint angle.
// These choices can be changed in the "Feedback selection" section.
//
// AVAILABLE OPERATING MODES
// -------------------------
// - Idle: the motor is not moving.
// - Fixed target: the motor moves to a selected angle and holds it.
// - Oscillation: the target moves smoothly from 0 to 90 degrees and back.
// - Holding: the motor applies active braking near a fixed target.
// - Manual: the selected motor moves directly at a fixed PWM without PID.
//
// SERIAL COMMANDS
// ---------------
// j       Select Motor 1
// k       Select Motor 2
// 0-9     Move the selected motor to a preset angle
// x       Start or stop oscillation for the selected motor
// left/a  Move the selected motor in reverse/down manually
// right/d Move the selected motor forward/up manually
// s/e/p   Stop both motors
// space   Stop both motors
// r       Zero both IMUs and both encoders
// m       Print the command menu
//
// IMPORTANT SAFETY NOTES
// ----------------------
// - Test one motor at a time before testing both motors together.
// - Confirm the motor direction signs before connecting the system to a user.
// - Confirm that active braking is safe for the selected motor driver.
// - Manual arrow control is open-loop and does not stop at an angle target.
// - Always press a stop key before the mechanism reaches a physical limit.
// - Keep an emergency power-disconnect method available during testing.


// ----------------------
// Serial communication and timing
// ----------------------
// SERIAL_BAUD must match the Python monitor.
// The control period is 10,000 microseconds, so the controller runs at 100 Hz.
// Telemetry is printed every 50 milliseconds, which is 20 times per second.

const unsigned long SERIAL_BAUD = 230400;
const unsigned long CONTROL_PERIOD_US = 10000;  // 100 Hz
const unsigned long TELEMETRY_PERIOD_MS = 50;   // 20 Hz

// Set one of these values to true only when automatic startup is desired.
// Keeping both false is safer because the motors wait for a user command.
const bool AUTO_START_MOTOR_1_OSCILLATION = false;
const bool AUTO_START_MOTOR_2_OSCILLATION = false;

// ----------------------
// IMU setup
// ----------------------
// The upper-arm IMU uses the main Wire bus.
// The forearm IMU uses Wire1. Both sensors use I2C address 0x28 because they
// are connected to different I2C buses.

Adafruit_BNO055 bnoUpper(0, 0x28, &Wire);
Adafruit_BNO055 bnoForearm(1, 0x28, &Wire1);

// A quaternion describes 3D orientation without the problems that can occur
// with Euler angles. The BNO055 gives its orientation as w, x, y, and z.
struct Quat {
  float w;
  float x;
  float y;
  float z;
};

// Raw orientations received directly from the two IMUs.
Quat qUpperRaw = {1.0, 0.0, 0.0, 0.0};
Quat qForearmRaw = {1.0, 0.0, 0.0, 0.0};

// Reference orientations saved when the user zeros the system.
Quat qUpperZero = {1.0, 0.0, 0.0, 0.0};
Quat qForearmZero = {1.0, 0.0, 0.0, 0.0};

// Orientations measured relative to the saved zero position.
// qJointZeroed represents the forearm orientation relative to the upper arm.
Quat qUpperZeroed = {1.0, 0.0, 0.0, 0.0};
Quat qForearmZeroed = {1.0, 0.0, 0.0, 0.0};
Quat qJointZeroed = {1.0, 0.0, 0.0, 0.0};

// imuOk prevents motor control when either IMU did not start correctly.
bool imuOk = false;

// Main angle measurements used by the controllers.
float upperArmAngleDeg = 0.0;  // Upper arm relative to its zero position
float rawJointAngleDeg = 0.0;  // Unfiltered elbow angle
float jointAngleDeg = 0.0;     // Filtered elbow angle used for control

// ----------------------
// Elbow-angle spike filter
// ----------------------
// IMU readings can sometimes jump suddenly because of sensor noise.
// This filter rejects large, unusual jumps and smooths normal measurements.
// The controller uses the filtered elbow angle instead of the raw angle.

float filteredJointAngleDeg = 0.0;
float averageJointAngleDeg = 0.0;
float jointAngleVariance = 0.0;
float lastGoodJointAngleDeg = 0.0;

bool filterInitialized = false;
unsigned long rejectedSpikes = 0;

// Larger alpha values react faster but allow more noise.
const float FILTER_ALPHA = 0.25;
const float AVERAGE_ALPHA = 0.10;
const float VARIANCE_ALPHA = 0.10;

// A reading is considered suspicious when it jumps more than 25 degrees and
// is also much farther from the recent average than normal measurements.
const float MAX_ANGLE_JUMP_DEG = 25.0;
const float MIN_VARIANCE_LIMIT = 100.0;

// ----------------------
// Motor hardware
// ----------------------
// Each motor uses two PWM-capable driver inputs and two encoder inputs.
// Update these pin numbers when the wiring changes.

const int M1_IN1 = 4;
const int M1_IN2 = 5;
const int M1_ENC_A = 30;
const int M1_ENC_B = 31;

const int M2_IN1 = 2;
const int M2_IN2 = 3;
const int M2_ENC_A = 28;
const int M2_ENC_B = 29;

Encoder encoder1(M1_ENC_A, M1_ENC_B);
Encoder encoder2(M2_ENC_A, M2_ENC_B);

// These values make motor direction easier to read in the code.
const int FORWARD = 1;
const int REVERSE = -1;

// Direction signs correct differences in physical wiring.
// Change only one sign at a time during testing.
// - Motor sign: change to -1 when the motor turns opposite to the command.
// - Encoder sign: change to -1 when encoder counts have the wrong sign.
const int M1_MOTOR_DIRECTION_SIGN = 1;
const int M2_MOTOR_DIRECTION_SIGN = 1;
const int M1_ENCODER_SIGN = 1;
const int M2_ENCODER_SIGN = 1;

// Manual arrow-key speed. This is the raw PWM used while an arrow command is
// active. Lower this value when slower manual positioning is needed.
const int MANUAL_PWM = 125;

// ----------------------
// Feedback selection
// ----------------------
// Feedback is the angle that a motor tries to control.
// FEEDBACK_UPPER_ARM uses only the upper IMU.
// FEEDBACK_ELBOW_JOINT uses the relative angle between both IMUs.

enum FeedbackSource {
  FEEDBACK_UPPER_ARM,
  FEEDBACK_ELBOW_JOINT
};

// Current final-product assignment:
// Motor 1 controls the upper-arm angle.
// Motor 2 controls the elbow-joint angle.
const FeedbackSource MOTOR_1_FEEDBACK = FEEDBACK_UPPER_ARM;
const FeedbackSource MOTOR_2_FEEDBACK = FEEDBACK_ELBOW_JOINT;

// ----------------------
// Motor controller settings and runtime state
// ----------------------
// One MotorController object stores everything needed to control one motor.
// This lets the same PID function control Motor 1 and Motor 2 independently.

struct MotorController {
  // Basic motor identification
  const char* name;

  // Hardware connections and direction corrections
  int in1;
  int in2;
  Encoder* encoder;
  int encoderSign;
  int motorDirectionSign;
  FeedbackSource feedbackSource;

  // PID gains
  // kp reacts to the current position error.
  // ki reacts to error that continues over time.
  // kd reacts to how quickly the measurement changes.
  // uFull is the controller-output value treated as full effort.
  float kp;
  float ki;
  float kd;
  float uFull;

  // PWM limits
  // minPwm helps overcome the motor deadband.
  // maxPwm is the largest command allowed.
  // slowPwm limits speed when the motor is near a fixed target.
  int minPwm;
  int maxPwm;
  int slowPwm;

  // Fixed-target behavior and safety
  float toleranceDeg;
  float slowZoneDeg;
  unsigned long fixedTargetTimeoutMs;

  // Values saved between PID updates
  float targetDeg;
  float previousError;
  float integralError;
  float previousMeasurement;
  float previousOutput;

  // Most recent motor command, also used for telemetry
  float normalizedEffort;
  int pwm;
  float signedPwmCommand;

  // Operating state
  bool active;
  bool holding;
  bool oscillationEnabled;

  // Manual mode bypasses PID and directly drives the motor at MANUAL_PWM.
  bool manualEnabled;
  int manualDirection;

  unsigned long targetStartMs;
  unsigned long oscillationStartMs;
};

// Motor 1 tuning and limits.
// Current gains make this a PI controller because kd is zero.
MotorController motor1 = {
  "M1",
  M1_IN1, M1_IN2, &encoder1,
  M1_ENCODER_SIGN, M1_MOTOR_DIRECTION_SIGN, MOTOR_1_FEEDBACK,

  0.90, 0.05, 0.0, 20.0,
  150, 255, 150,
  1.0, 5.0, 15000,

  0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0, 0.0,
  false, false, false, false, 0, 0, 0
};

// Motor 2 tuning and limits.
// Current gains make this a PI controller because kd is zero.
MotorController motor2 = {
  "M2",
  M2_IN1, M2_IN2, &encoder2,
  M2_ENCODER_SIGN, M2_MOTOR_DIRECTION_SIGN, MOTOR_2_FEEDBACK,

  1.21, 0.096, 0.0, 20.0,
  150, 255, 150,
  1.0, 5.0, 15000,

  0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0, 0.0,
  false, false, false, false, 0, 0, 0
};

// Existing target mapping is preserved: key 0 means 5 degrees.
const float PRESET_TARGETS_DEG[10] = {
  5.0, 10.0, 20.0, 30.0, 40.0,
  50.0, 60.0, 70.0, 80.0, 90.0
};

// Target, oscillation, and manual arrow commands affect the selected motor.
// Motor 2 is selected when the program starts.
MotorController* selectedMotor = &motor2;

// ANSI arrow keys arrive as three serial bytes: ESC, [, and C or D.
// This variable remembers which part of that sequence was received.
int serialEscapeState = 0;

// ----------------------
// Oscillation settings
// ----------------------
// The target follows a smooth cosine wave from 0 to 90 degrees and back.
// At 0.05 Hz, one complete cycle takes 20 seconds.

const float OSCILLATION_FREQUENCY_HZ = 0.05;
const float OSCILLATION_CENTER_DEG = 45.0;
const float OSCILLATION_AMPLITUDE_DEG = 45.0;

// ----------------------
// Runtime timing
// ----------------------
// These variables remember when control and telemetry last ran.
// unsigned long arithmetic also handles the normal micros()/millis() rollover.

unsigned long lastControlUs = 0;
unsigned long lastTelemetryMs = 0;


// Function declarations

// These declarations tell the compiler which functions are defined later.
// They also provide a quick list of the program's main tasks.

Quat normalizeQuaternion(Quat q);
Quat conjugateQuaternion(Quat q);
Quat multiplyQuaternions(Quat a, Quat b);
Quat fromBnoQuaternion(imu::Quaternion q);
float quaternionAngleDeg(Quat q);

void resetJointAngleFilter(float startAngleDeg);
float filterJointAngle(float rawDeg);

bool startImus();
void readImus();
void zeroImus();

float feedbackAngle(const MotorController& motor);
const char* feedbackName(const MotorController& motor);
long encoderCounts(const MotorController& motor);

void driveMotor(MotorController& motor, int direction, int pwm);
void motorOff(MotorController& motor);
void motorHold(MotorController& motor);

float clamp01(float value);
void resetControllerState(MotorController& motor, bool deactivate);
void initializeControllerForTarget(MotorController& motor, float targetDeg);
void updatePid(MotorController& motor, float dtSeconds);

float calculateOscillationTarget(float timeSeconds);
void startOscillation(MotorController& motor);
void stopOscillation(MotorController& motor);
void updateOscillationTarget(MotorController& motor);

void setFixedTarget(MotorController& motor, float targetDeg);
void startManualDrive(MotorController& motor, int direction);
void updateManualDrive(MotorController& motor);
void stopMotor(MotorController& motor, const char* reason);
void stopAllMotion(const char* reason);

void checkSerialCommands();
void handleSerialCommand(char command);
void handleArrowCommand(char arrowCode);

const char* controlModeName(const MotorController& motor);
void printQuaternion(const char* label, Quat q);
void printTelemetry();
void printMenu();


// Quaternion math


// Makes a quaternion have a length of 1.
// Normalization is required before using it for orientation calculations.
Quat normalizeQuaternion(Quat q) {
  float magnitude = sqrt(
      q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

  if (magnitude < 0.000001) {
    return {1.0, 0.0, 0.0, 0.0};
  }

  q.w /= magnitude;
  q.x /= magnitude;
  q.y /= magnitude;
  q.z /= magnitude;
  return q;
}

// Returns the inverse rotation for a normalized quaternion.
// It is used to compare a new orientation with the saved zero orientation.
Quat conjugateQuaternion(Quat q) {
  q = normalizeQuaternion(q);
  return {q.w, -q.x, -q.y, -q.z};
}

// Combines two rotations. Quaternion multiplication order is important.
Quat multiplyQuaternions(Quat a, Quat b) {
  Quat result;

  result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

  return normalizeQuaternion(result);
}

// Converts the Adafruit library quaternion into the program's Quat type.
Quat fromBnoQuaternion(imu::Quaternion q) {
  return normalizeQuaternion({
    (float)q.w(),
    (float)q.x(),
    (float)q.y(),
    (float)q.z()
  });
}

// Converts a quaternion rotation into one positive angle in degrees.
// This gives the size of the rotation, not a signed rotation direction.
float quaternionAngleDeg(Quat q) {
  q = normalizeQuaternion(q);
  float w = constrain(fabsf(q.w), 0.0f, 1.0f);
  return 2.0 * acos(w) * 180.0 / PI;
}


// IMU filtering


// Starts or restarts the elbow filter at a known angle.
void resetJointAngleFilter(float startAngleDeg) {
  rawJointAngleDeg = startAngleDeg;
  filteredJointAngleDeg = startAngleDeg;
  averageJointAngleDeg = startAngleDeg;
  jointAngleVariance = 0.0;
  lastGoodJointAngleDeg = startAngleDeg;
  filterInitialized = true;
  rejectedSpikes = 0;
}

// Checks a new elbow reading for a spike and then applies smoothing.
// A rejected spike is replaced with the last trusted filtered value.
float filterJointAngle(float rawDeg) {
  if (!filterInitialized) {
    resetJointAngleFilter(rawDeg);
    return rawDeg;
  }

  float jump = fabs(rawDeg - filteredJointAngleDeg);
  float differenceFromAverage = rawDeg - averageJointAngleDeg;
  float instantaneousVariance =
      differenceFromAverage * differenceFromAverage;

  float varianceLimit = max(
      jointAngleVariance * 3.0f,
      MIN_VARIANCE_LIMIT
  );

  bool isSpike =
      (jump > MAX_ANGLE_JUMP_DEG) &&
      (instantaneousVariance > varianceLimit);

  if (isSpike) {
    rejectedSpikes++;
    filteredJointAngleDeg = lastGoodJointAngleDeg;
    return filteredJointAngleDeg;
  }

  averageJointAngleDeg =
      AVERAGE_ALPHA * rawDeg +
      (1.0 - AVERAGE_ALPHA) * averageJointAngleDeg;

  float newDifference = rawDeg - averageJointAngleDeg;
  float newVariance = newDifference * newDifference;

  jointAngleVariance =
      VARIANCE_ALPHA * newVariance +
      (1.0 - VARIANCE_ALPHA) * jointAngleVariance;

  filteredJointAngleDeg =
      FILTER_ALPHA * rawDeg +
      (1.0 - FILTER_ALPHA) * filteredJointAngleDeg;

  lastGoodJointAngleDeg = filteredJointAngleDeg;
  return filteredJointAngleDeg;
}


// IMU functions


// Starts both I2C buses and both BNO055 sensors.
// The function returns false when either sensor cannot be detected.
bool startImus() {
  Wire.begin();
  Wire1.begin();
  delay(100);

  bool upperOk = bnoUpper.begin();
  bool forearmOk = bnoForearm.begin();

  if (!upperOk) {
    Serial.println("ERROR: Upper BNO055 not detected.");
  }

  if (!forearmOk) {
    Serial.println("ERROR: Forearm BNO055 not detected.");
  }

  if (!upperOk || !forearmOk) {
    imuOk = false;
    return false;
  }

  delay(1000);
  bnoUpper.setExtCrystalUse(true);
  bnoForearm.setExtCrystalUse(true);

  imuOk = true;
  readImus();
  zeroImus();

  Serial.println("IMUs started and zeroed.");
  return true;
}

// Reads both IMUs and calculates the angles used for feedback.
// The elbow angle is found from forearm orientation relative to upper-arm
// orientation. This removes motion that both arm sections share.
void readImus() {
  if (!imuOk) {
    return;
  }

  qUpperRaw = fromBnoQuaternion(bnoUpper.getQuat());
  qForearmRaw = fromBnoQuaternion(bnoForearm.getQuat());

  qUpperZeroed = multiplyQuaternions(
      conjugateQuaternion(qUpperZero), qUpperRaw);

  qForearmZeroed = multiplyQuaternions(
      conjugateQuaternion(qForearmZero), qForearmRaw);

  qJointZeroed = multiplyQuaternions(
      conjugateQuaternion(qUpperZeroed), qForearmZeroed);

  upperArmAngleDeg = quaternionAngleDeg(qUpperZeroed);
  rawJointAngleDeg = quaternionAngleDeg(qJointZeroed);
  jointAngleDeg = filterJointAngle(rawJointAngleDeg);
}

// Saves the current arm position as zero and clears the encoders.
// All motor motion is stopped first so the reference is taken safely.
void zeroImus() {
  if (!imuOk) {
    Serial.println("Cannot zero IMUs: IMUs are not ready.");
    return;
  }

  stopAllMotion("imu_zero");

  qUpperRaw = fromBnoQuaternion(bnoUpper.getQuat());
  qForearmRaw = fromBnoQuaternion(bnoForearm.getQuat());

  qUpperZero = qUpperRaw;
  qForearmZero = qForearmRaw;

  qUpperZeroed = {1.0, 0.0, 0.0, 0.0};
  qForearmZeroed = {1.0, 0.0, 0.0, 0.0};
  qJointZeroed = {1.0, 0.0, 0.0, 0.0};

  upperArmAngleDeg = 0.0;
  rawJointAngleDeg = 0.0;
  jointAngleDeg = 0.0;
  resetJointAngleFilter(0.0);

  encoder1.write(0);
  encoder2.write(0);

  motor1.targetDeg = 0.0;
  motor2.targetDeg = 0.0;

  Serial.println("IMUs and encoders zeroed.");
}


// Feedback and motor helpers


// Returns the angle selected for this motor's feedback source.
float feedbackAngle(const MotorController& motor) {
  if (motor.feedbackSource == FEEDBACK_UPPER_ARM) {
    return upperArmAngleDeg;
  }

  return jointAngleDeg;
}

// Returns a readable feedback name for status messages.
const char* feedbackName(const MotorController& motor) {
  if (motor.feedbackSource == FEEDBACK_UPPER_ARM) {
    return "UpperArm";
  }

  return "ElbowJoint";
}

// Reads the motor encoder and applies its configured sign correction.
// The PID currently uses IMU angle feedback; encoder counts are sent as
// telemetry and remain available for future speed or position control.
long encoderCounts(const MotorController& motor) {
  return motor.encoderSign * motor.encoder->read();
}

// Sends direction and PWM to one motor driver.
// The motorDirectionSign is applied here so the PID logic can use the same
// FORWARD and REVERSE meanings for both motors.
void driveMotor(MotorController& motor, int direction, int pwm) {
  pwm = constrain(pwm, 0, 255);

  if (pwm <= 0 || direction == 0) {
    motorOff(motor);
    return;
  }

  int actualDirection = direction * motor.motorDirectionSign;

  if (actualDirection > 0) {
    analogWrite(motor.in1, pwm);
    analogWrite(motor.in2, 0);
  } else {
    analogWrite(motor.in1, 0);
    analogWrite(motor.in2, pwm);
  }
}

// Removes voltage commands from both driver inputs so the motor can coast.
void motorOff(MotorController& motor) {
  analogWrite(motor.in1, 0);
  analogWrite(motor.in2, 0);
}

// Applies active braking by setting both driver inputs high.
// Confirm that this behavior is supported by the motor driver hardware.
void motorHold(MotorController& motor) {
  // Active braking. Replace with motorOff(motor) if the driver should coast.
  analogWrite(motor.in1, 255);
  analogWrite(motor.in2, 255);
}


// PI/PID controller


// Limits a value to the range 0.0 through 1.0.
float clamp01(float value) {
  if (value < 0.0) {
    return 0.0;
  }

  if (value > 1.0) {
    return 1.0;
  }

  return value;
}

// Clears saved PID values and the last motor command.
// When deactivate is true, the motor will stay off until a new target starts.
void resetControllerState(MotorController& motor, bool deactivate) {
  if (deactivate) {
    motor.active = false;
  }

  motor.holding = false;
  motor.manualEnabled = false;
  motor.manualDirection = 0;
  motor.previousError = 0.0;
  motor.integralError = 0.0;
  motor.previousMeasurement = feedbackAngle(motor);
  motor.previousOutput = 0.0;
  motor.normalizedEffort = 0.0;
  motor.pwm = 0;
  motor.signedPwmCommand = 0.0;
  motor.targetStartMs = millis();
}

// Prepares one motor to begin controlling a new target.
// Clearing the old integral and derivative history prevents an old command
// from affecting the new movement.
void initializeControllerForTarget(
    MotorController& motor,
    float targetDeg
) {
  float current = feedbackAngle(motor);

  motor.targetDeg = targetDeg;
  motor.active = true;
  motor.holding = false;
  motor.manualEnabled = false;
  motor.manualDirection = 0;
  motor.previousError = targetDeg - current;
  motor.integralError = 0.0;
  motor.previousMeasurement = current;
  motor.previousOutput = 0.0;
  motor.normalizedEffort = 0.0;
  motor.pwm = 0;
  motor.signedPwmCommand = 0.0;
  motor.targetStartMs = millis();
}

// Runs one complete PI/PID update for one motor.
//
// The important steps are:
// 1. Read the selected feedback angle.
// 2. Calculate target error.
// 3. Check target tolerance and safety timeout.
// 4. Update the integral term with anti-windup protection.
// 5. Calculate P + I + D output.
// 6. Convert output magnitude to PWM and output sign to direction.
void updatePid(MotorController& motor, float dtSeconds) {
  // Manual mode directly controls the motor, so PID must not overwrite it.
  if (motor.manualEnabled) {
    return;
  }

  // Never drive a motor without valid IMU feedback or an active target.
  if (!imuOk || !motor.active) {
    motorOff(motor);
    motor.normalizedEffort = 0.0;
    motor.pwm = 0;
    motor.signedPwmCommand = 0.0;
    return;
  }

  // Positive error means the measured angle is below the target.
  // Negative error means the measured angle is above the target.
  float current = feedbackAngle(motor);
  float error = motor.targetDeg - current;
  float absoluteError = fabs(error);

  bool errorChangedSign =
      (error > 0.0 && motor.previousError < 0.0) ||
      (error < 0.0 && motor.previousError > 0.0);

  // Reset the integral when the motor passes the target. This reduces the
  // chance that stored integral error keeps pushing in the old direction.
  if (errorChangedSign) {
    motor.integralError = 0.0;
  }

  bool atFixedTarget = false;

  if (!motor.oscillationEnabled) {
    float exitTolerance = motor.toleranceDeg * 1.5;
    atFixedTarget = motor.holding
        ? (absoluteError <= exitTolerance)
        : (absoluteError <= motor.toleranceDeg);
  }

  // Fixed targets use a tolerance area. Oscillation does not stop at each
  // temporary target because its target is always moving.
  if (atFixedTarget) {
    motorHold(motor);
    motor.normalizedEffort = 0.0;
    motor.pwm = 0;
    motor.signedPwmCommand = 0.0;
    motor.integralError = 0.0;
    motor.previousError = error;
    motor.previousMeasurement = current;
    motor.previousOutput = 0.0;
    motor.holding = true;
    motor.targetStartMs = millis();
    return;
  }

  if (motor.holding) {
    motor.holding = false;
    motor.integralError = 0.0;
    motor.previousError = error;
    motor.previousMeasurement = current;
    motor.previousOutput = 0.0;
    motor.targetStartMs = millis();
  }

  // Stop a fixed-target movement if it takes too long. This helps protect
  // the user and hardware when the system is blocked or cannot reach target.
  if (!motor.oscillationEnabled &&
      millis() - motor.targetStartMs > motor.fixedTargetTimeoutMs) {
    stopMotor(motor, "fixed_target_timeout");
    return;
  }

  // Use a lower PWM limit near a fixed target for smoother stopping.
  int pwmLimit = motor.maxPwm;

  if (!motor.oscillationEnabled &&
      absoluteError <= motor.slowZoneDeg) {
    pwmLimit = motor.slowPwm;
  }

  // Derivative on measurement avoids a derivative kick after target changes.
  float measurementRate =
      (current - motor.previousMeasurement) / dtSeconds;
  float derivativeError = -measurementRate;

  // Anti-windup check: when output is already at full effort in the same
  // direction as the error, more integral error would not help.
  bool outputSaturated =
      (fabs(motor.previousOutput) >= fabs(motor.uFull)) &&
      ((motor.previousOutput > 0.0) == (error > 0.0));

  // Conditional integration provides anti-windup.
  if (!outputSaturated) {
    motor.integralError += error * dtSeconds;
  }

  if (fabs(motor.ki) > 0.000001) {
    float integralLimit = fabs(motor.uFull / motor.ki);
    motor.integralError = constrain(
        motor.integralError,
        -integralLimit,
        integralLimit
    );
  } else {
    motor.integralError = 0.0;
  }

  // This is the PID equation: output = P + I + D.
  float controllerOutput =
      motor.kp * error +
      motor.ki * motor.integralError +
      motor.kd * derivativeError;

  motor.previousError = error;
  motor.previousMeasurement = current;
  motor.previousOutput = controllerOutput;

  // Convert controller output to a value from 0 to 1, then map that value
  // between the motor's minimum and maximum allowed PWM.
  float fullOutput = max(fabsf(motor.uFull), 0.000001f);
  float normalizedEffort =
      clamp01(fabs(controllerOutput) / fullOutput);

  int minimumPwm = min(motor.minPwm, pwmLimit);
  int pwm = minimumPwm +
      (int)((pwmLimit - minimumPwm) * normalizedEffort);

  if (fabs(controllerOutput) < 0.0001) {
    pwm = 0;
    normalizedEffort = 0.0;
  }

  // The sign controls direction. The size controls PWM effort.
  int direction = controllerOutput >= 0.0 ? FORWARD : REVERSE;
  float signedPwm =
      direction * motor.motorDirectionSign * pwm;

  motor.normalizedEffort = normalizedEffort;
  motor.pwm = pwm;
  motor.signedPwmCommand = signedPwm;

  // Send the final command to the physical motor driver.
  driveMotor(motor, direction, pwm);
}


// Fixed targets and oscillation


// Calculates the moving target for oscillation.
// At time zero the target is 0 degrees. It rises smoothly to 90 degrees,
// returns smoothly to 0 degrees, and then repeats.
float calculateOscillationTarget(float timeSeconds) {
  return OSCILLATION_CENTER_DEG -
      OSCILLATION_AMPLITUDE_DEG *
      cos(2.0 * PI * OSCILLATION_FREQUENCY_HZ * timeSeconds);
}

// Starts oscillation for one motor and initializes its PID state.
void startOscillation(MotorController& motor) {
  motor.manualEnabled = false;
  motor.manualDirection = 0;
  motor.oscillationEnabled = true;
  motor.oscillationStartMs = millis();
  initializeControllerForTarget(
      motor,
      calculateOscillationTarget(0.0)
  );

  Serial.print(motor.name);
  Serial.print(" oscillation started at ");
  Serial.print(OSCILLATION_FREQUENCY_HZ, 3);
  Serial.println(" Hz.");
}

// Stops changing the target. This function does not stop motor output by
// itself; stopMotor() is used when the motor must also be turned off.
void stopOscillation(MotorController& motor) {
  motor.oscillationEnabled = false;
  motor.oscillationStartMs = 0;
}

// Updates the motor's target angle during every control cycle.
void updateOscillationTarget(MotorController& motor) {
  if (!motor.oscillationEnabled) {
    return;
  }

  float elapsedSeconds =
      (millis() - motor.oscillationStartMs) / 1000.0;

  motor.targetDeg = calculateOscillationTarget(elapsedSeconds);
  motor.active = true;
  motor.holding = false;
}

// Stops oscillation and starts movement toward one fixed angle.
// The allowed target range is limited to 0 through 90 degrees.
void setFixedTarget(MotorController& motor, float targetDeg) {
  motor.manualEnabled = false;
  motor.manualDirection = 0;
  stopOscillation(motor);
  targetDeg = constrain(targetDeg, 0.0f, 90.0f);
  initializeControllerForTarget(motor, targetDeg);

  Serial.print(motor.name);
  Serial.print(" fixed target set to ");
  Serial.print(targetDeg, 2);
  Serial.print(" degrees using ");
  Serial.print(feedbackName(motor));
  Serial.println(" feedback.");
}


// Starts manual open-loop movement for the selected motor.
// Unlike PID control, this mode does not use an angle target and will keep
// moving until another command changes the mode or stops the motors.
void startManualDrive(MotorController& motor, int direction) {
  if (!imuOk) {
    motorOff(motor);
    Serial.print("Manual control blocked for ");
    Serial.print(motor.name);
    Serial.println(": IMUs are not ready.");
    return;
  }

  stopOscillation(motor);

  motor.active = false;
  motor.holding = false;
  motor.manualEnabled = true;
  motor.manualDirection = direction >= 0 ? FORWARD : REVERSE;

  motor.targetDeg = feedbackAngle(motor);
  motor.previousError = 0.0;
  motor.integralError = 0.0;
  motor.previousMeasurement = feedbackAngle(motor);
  motor.previousOutput = 0.0;

  // Apply the command immediately instead of waiting for the next loop cycle.
  updateManualDrive(motor);

  Serial.print(motor.name);
  Serial.print(" manual mode: ");
  Serial.print(motor.manualDirection == FORWARD ? "forward/up" : "reverse/down");
  Serial.print(" at PWM ");
  Serial.println(MANUAL_PWM);
}

// Keeps the manual PWM command active and updates the telemetry values.
void updateManualDrive(MotorController& motor) {
  if (!motor.manualEnabled) {
    return;
  }

  if (!imuOk) {
    motorOff(motor);
    motor.pwm = 0;
    motor.normalizedEffort = 0.0;
    motor.signedPwmCommand = 0.0;
    return;
  }

  int pwm = constrain(MANUAL_PWM, 0, motor.maxPwm);
  float normalized = motor.maxPwm > 0
      ? (float)pwm / (float)motor.maxPwm
      : 0.0;

  motor.pwm = pwm;
  motor.normalizedEffort = clamp01(normalized);
  motor.signedPwmCommand =
      motor.manualDirection * motor.motorDirectionSign * pwm;
  motor.targetDeg = feedbackAngle(motor);

  driveMotor(motor, motor.manualDirection, pwm);
}

// Safely stops and deactivates one motor.
// The reason is printed so the monitor can show why the motor stopped.
void stopMotor(MotorController& motor, const char* reason) {
  stopOscillation(motor);
  motorOff(motor);
  resetControllerState(motor, true);
  motor.targetDeg = feedbackAngle(motor);

  Serial.print("STOP,");
  Serial.print(motor.name);
  Serial.print(",");
  Serial.println(reason);
}

// Emergency-stop helper for both motors.
void stopAllMotion(const char* reason) {
  stopOscillation(motor1);
  stopOscillation(motor2);

  motorOff(motor1);
  motorOff(motor2);

  resetControllerState(motor1, true);
  resetControllerState(motor2, true);

  motor1.targetDeg = feedbackAngle(motor1);
  motor2.targetDeg = feedbackAngle(motor2);

  Serial.print("STOP,ALL,");
  Serial.println(reason);
}


// Serial commands


// Reads every available serial character without blocking the control loop.
// Arrow keys are received as ANSI escape sequences: ESC [ D for left and
// ESC [ C for right. Other commands are single characters.
void checkSerialCommands() {
  while (Serial.available() > 0) {
    char command = Serial.read();

    if (serialEscapeState == 0) {
      if (command == 27) {
        serialEscapeState = 1;
      } else if (command != '\n' && command != '\r') {
        handleSerialCommand(command);
      }
      continue;
    }

    if (serialEscapeState == 1) {
      serialEscapeState = command == '[' ? 2 : 0;
      continue;
    }

    if (serialEscapeState == 2) {
      handleArrowCommand(command);
      serialEscapeState = 0;
    }
  }
}

// Connects each keyboard/serial command to a controller action.
void handleSerialCommand(char command) {
  if (command == 'j' || command == 'J') {
    selectedMotor = &motor1;
    Serial.println("Selected Motor 1.");
    return;
  }

  if (command == 'k' || command == 'K') {
    selectedMotor = &motor2;
    Serial.println("Selected Motor 2.");
    return;
  }

  if (command >= '0' && command <= '9') {
    int index = command - '0';
    setFixedTarget(*selectedMotor, PRESET_TARGETS_DEG[index]);
    return;
  }

  if (command == 'x' || command == 'X') {
    if (selectedMotor->oscillationEnabled) {
      stopMotor(*selectedMotor, "oscillation_stopped");
    } else {
      startOscillation(*selectedMotor);
    }
    return;
  }

  // The Python visualizer sends simple a/d commands for the arrow keys.
  // Single-byte commands are more dependable than terminal escape sequences.
  if (command == 'a' || command == 'A') {
    startManualDrive(*selectedMotor, REVERSE);
    return;
  }

  if (command == 'd' || command == 'D') {
    startManualDrive(*selectedMotor, FORWARD);
    return;
  }

  if (command == 's' || command == 'S' ||
      command == 'e' || command == 'E' ||
      command == 'p' || command == 'P' ||
      command == ' ') {
    stopAllMotion("user_emergency_stop");
    return;
  }

  if (command == 'r' || command == 'R') {
    Serial.println("Re-zeroing IMUs and encoders...");
    zeroImus();
    return;
  }

  if (command == 'm' || command == 'M') {
    printMenu();
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(command);
}

// Converts the final byte of an ANSI arrow-key sequence into manual motion.
// D is left/reverse. C is right/forward.
void handleArrowCommand(char arrowCode) {
  if (arrowCode == 'D') {
    startManualDrive(*selectedMotor, REVERSE);
    return;
  }

  if (arrowCode == 'C') {
    startManualDrive(*selectedMotor, FORWARD);
  }
}


// Telemetry


// Converts internal motor state into a readable mode name.
const char* controlModeName(const MotorController& motor) {
  if (motor.manualEnabled) {
    return "Manual";
  }

  if (motor.oscillationEnabled) {
    return "Oscillation";
  }

  if (motor.holding) {
    return "Holding";
  }

  if (motor.active) {
    return "FixedTarget";
  }

  return "Idle";
}

// Prints one quaternion for the Python 3D arm display.
void printQuaternion(const char* label, Quat q) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print(q.w, 6);
  Serial.print(", ");
  Serial.print(q.x, 6);
  Serial.print(", ");
  Serial.print(q.y, 6);
  Serial.print(", ");
  Serial.println(q.z, 6);
}

// Sends motor and sensor information to the computer at 20 Hz.
// Control still runs at 100 Hz, so printing does not happen every control step.
void printTelemetry() {
  unsigned long now = millis();

  if (now - lastTelemetryMs < TELEMETRY_PERIOD_MS) {
    return;
  }

  lastTelemetryMs = now;

  float m1Current = feedbackAngle(motor1);
  float m2Current = feedbackAngle(motor2);

  float m1Error = motor1.active && !motor1.manualEnabled
      ? motor1.targetDeg - m1Current
      : 0.0;

  float m2Error = motor2.active && !motor2.manualEnabled
      ? motor2.targetDeg - m2Current
      : 0.0;

  // Parsed by final_dual_motor_monitor.py.
  // STATE,time_ms,selected,
  // m1_mode,m1_target,m1_current,m1_error,m1_pwm,m1_u,m1_counts,
  // m2_mode,m2_target,m2_current,m2_error,m2_pwm,m2_u,m2_counts,
  // upper_angle,elbow_angle,rejected_spikes
  Serial.print("STATE,");
  Serial.print(now);
  Serial.print(",");
  Serial.print(selectedMotor->name);
  Serial.print(",");

  Serial.print(controlModeName(motor1));
  Serial.print(",");
  Serial.print(motor1.targetDeg, 4);
  Serial.print(",");
  Serial.print(m1Current, 4);
  Serial.print(",");
  Serial.print(m1Error, 4);
  Serial.print(",");
  Serial.print(motor1.pwm);
  Serial.print(",");
  Serial.print(motor1.signedPwmCommand, 2);
  Serial.print(",");
  Serial.print(encoderCounts(motor1));
  Serial.print(",");

  Serial.print(controlModeName(motor2));
  Serial.print(",");
  Serial.print(motor2.targetDeg, 4);
  Serial.print(",");
  Serial.print(m2Current, 4);
  Serial.print(",");
  Serial.print(m2Error, 4);
  Serial.print(",");
  Serial.print(motor2.pwm);
  Serial.print(",");
  Serial.print(motor2.signedPwmCommand, 2);
  Serial.print(",");
  Serial.print(encoderCounts(motor2));
  Serial.print(",");

  Serial.print(upperArmAngleDeg, 4);
  Serial.print(",");
  Serial.print(jointAngleDeg, 4);
  Serial.print(",");
  Serial.println(rejectedSpikes);

  printQuaternion("qUpperZeroed", qUpperZeroed);
  printQuaternion("qForearmZeroed", qForearmZeroed);
  printQuaternion("qJointZeroed", qJointZeroed);
}

// Prints the available serial commands and current feedback assignments.
void printMenu() {
  Serial.println();
  Serial.println("========== FINAL DUAL-MOTOR CONTROLLER ==========");
  Serial.println("j       : Select Motor 1");
  Serial.println("k       : Select Motor 2");
  Serial.println("0-9     : Set selected motor to a preset target");
  Serial.println("x       : Start/stop selected motor oscillation");
  Serial.println("left / a : Selected motor reverse/down at manual PWM");
  Serial.println("right / d: Selected motor forward/up at manual PWM");
  Serial.println("s/e/p   : Emergency stop both motors");
  Serial.println("space   : Emergency stop both motors");
  Serial.println("r       : Re-zero both IMUs and encoders");
  Serial.println("m       : Print this menu");
  Serial.println();
  Serial.print("Motor 1 feedback: ");
  Serial.println(feedbackName(motor1));
  Serial.print("Motor 2 feedback: ");
  Serial.println(feedbackName(motor2));
  Serial.print("Currently selected: ");
  Serial.println(selectedMotor->name);
  Serial.println("=================================================");
  Serial.println();
}


// Arduino setup and loop


// setup() runs once after the board powers on or resets.
// It starts serial communication, prepares the motor pins, starts the IMUs,
// clears both controllers, and prints the command menu.
void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  motorOff(motor1);
  motorOff(motor2);

  delay(1500);
  Serial.println("Starting final dual-motor, dual-IMU controller...");

  if (!startImus()) {
    Serial.println("IMU startup failed. Both motors remain disabled.");
  }

  resetControllerState(motor1, true);
  resetControllerState(motor2, true);

  lastControlUs = micros();
  lastTelemetryMs = 0;

  printMenu();

  if (imuOk && AUTO_START_MOTOR_1_OSCILLATION) {
    startOscillation(motor1);
  }

  if (imuOk && AUTO_START_MOTOR_2_OSCILLATION) {
    startOscillation(motor2);
  }
}

// loop() runs continuously while the board is powered.
// Serial commands are checked as often as possible. The full sensor and PID
// update runs every 10 milliseconds, which produces a 100 Hz control rate.
void loop() {
  checkSerialCommands();

  unsigned long nowUs = micros();

  // Continue printing telemetry while waiting for the next 100 Hz control
  // update. Returning here prevents the PID from running too quickly.
  if (nowUs - lastControlUs < CONTROL_PERIOD_US) {
    printTelemetry();
    return;
  }

  float dtSeconds =
      (nowUs - lastControlUs) / 1000000.0;
  lastControlUs = nowUs;

  if (dtSeconds <= 0.0) {
    dtSeconds = CONTROL_PERIOD_US / 1000000.0;
  }

  // Limit an unusually large dt after a pause or delay. A very large dt can
  // create a large integral update and an unsafe output change.
  if (dtSeconds > 0.05) {
    dtSeconds = 0.05;
  }

  // Sensor readings must be updated before target error is calculated.
  readImus();

  // Oscillation changes the targets before the PID controllers run.
  updateOscillationTarget(motor1);
  updateOscillationTarget(motor2);

  // Manual mode bypasses PID and applies a direct PWM command. A motor that
  // is not in manual mode continues using its normal PID controller.
  updateManualDrive(motor1);
  updateManualDrive(motor2);

  // Each motor uses the same PID function but has independent settings and
  // independent saved state. updatePid() returns immediately in manual mode.
  updatePid(motor1, dtSeconds);
  updatePid(motor2, dtSeconds);

  printTelemetry();
}