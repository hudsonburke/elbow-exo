#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ======================================================
// FINAL DUAL-MOTOR CONTROLLER
// ======================================================
//
// Name guide:
// - ang = angle
// - deg = degrees
// - vel = velocity
// - tgt = target
// - pwm = motor command
// - int = integral
// - prev = previous
// - cnt = count
// - lim = limit
//
// All custom variable names are 10 characters or less.
//
// This program controls two motors with two BNO055 IMUs.
// Motor 1 uses the upper-arm angle. Motor 2 uses the elbow angle.
//
// Fixed targets use PI control. Oscillation also uses velocity feedback.
// Motor 2 includes two protections:
// 1. A limit on the velocity correction during an IMU spike.
// 2. A small delay before changing motor direction near zero effort.
//
// This final version does not run test trials or collect CSV data.
// It still sends live serial values to the Python monitor.
//
// Main keys:
// j/k     Select Motor 1 or Motor 2
// 0-9     Send a preset target angle
// x       Start or stop oscillation
// a/d     Manual reverse or forward
// s       Stop both motors
// r       Zero the IMUs and encoders
// m       Print the command menu
//
// Safety:
// - Test the system without a person first.
// - Check motor and encoder directions before automatic movement.
// - Keep an emergency power switch close during testing.
// - Confirm the elbow limits before using the system with a person.
// ======================================================

// ----------------------
// Serial communication and timing
// ----------------------
// BAUD_RATE must match the Python monitor.
// The control period is 10,000 microseconds, so the controller runs at 100 Hz.
// Telemetry is printed every 50 milliseconds, which is 20 times per second.

const unsigned long BAUD_RATE = 230400;
const unsigned long CTRL_US = 10000;  // 100 Hz
const unsigned long SEND_MS = 50;   // 20 Hz

// Set one of these values to true only when automatic startup is desired.
// Keeping both false is safer because the motors wait for a user command.
const bool AUTO_M1 = false;
const bool AUTO_M2 = false;

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
struct QuaternionData {
  float w;
  float x;
  float y;
  float z;
};

// Raw orientations received directly from the two IMUs.
QuaternionData upRawQ = {1.0, 0.0, 0.0, 0.0};
QuaternionData frRawQ = {1.0, 0.0, 0.0, 0.0};

// Reference orientations saved when the user zeros the system.
QuaternionData upZeroQ = {1.0, 0.0, 0.0, 0.0};
QuaternionData frZeroQ = {1.0, 0.0, 0.0, 0.0};

// Orientations measured relative to the saved zero position.
// elbRelQ represents the forearm orientation relative to the upper arm.
QuaternionData upRelQ = {1.0, 0.0, 0.0, 0.0};
QuaternionData frRelQ = {1.0, 0.0, 0.0, 0.0};
QuaternionData elbRelQ = {1.0, 0.0, 0.0, 0.0};

// imusReady prevents motor control when either IMU did not start correctly.
bool imusReady = false;

// Main angle measurements used by the controllers.
float upAngDeg = 0.0;  // Upper arm relative to its zero position
float rawJntDeg = 0.0;  // Unfiltered elbow angle before drift correction
float jntAngDeg = 0.0;     // Corrected elbow angle used for control

// The BNO055 relative angle can slowly drift even when the real elbow is at
// full extension. This offset is updated only when the Motor 2 encoder
// confirms that the cable is at its known lower mechanical limit.
float elbZeroDeg = 0.0;

// BNO055 calibration values range from 0 (not calibrated) to 3 (fully
// calibrated). They are sent to live serial data for diagnosis. They do not
// automatically stop the system in this version.
uint8_t upSysCal = 0;
uint8_t upGyrCal = 0;
uint8_t upAccCal = 0;
uint8_t upMagCal = 0;

uint8_t frSysCal = 0;
uint8_t frGyrCal = 0;
uint8_t frAccCal = 0;
uint8_t frMagCal = 0;

// ----------------------
// Elbow-angle spike filter
// ----------------------
// IMU readings can sometimes jump suddenly because of sensor noise.
// This filter rejects large, unusual jumps and smooths normal measurements.
// The controller uses the filtered elbow angle instead of the raw angle.

float filtJntDeg = 0.0;
float avgJntDeg = 0.0;
float jntVar = 0.0;
float lastJntDeg = 0.0;

bool filtReady = false;
unsigned long spikeCnt = 0;

// Larger alpha values react faster but allow more noise.
const float FILT_A = 0.25;
const float AVG_A = 0.10;
const float VAR_A = 0.10;

// A reading is considered suspicious when it jumps more than 25 degrees and
// is also much farther from the recent average than normal measurements.
const float MAX_JUMP = 25.0;
const float MIN_VAR = 100.0;

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
const int M1_DIR = 1;
const int M2_DIR = 1;
const int M1_ENCSGN = 1;
const int M2_ENCSGN = 1;

// Manual arrow-key speed. This is the raw PWM used while an arrow command is
// active. Lower this value when slower manual positioning is needed.
const int MANUAL_PWM = 125;

// ----------------------
// Elbow mechanical limits and drift correction
// ----------------------
//
// IMPORTANT ENCODER NOTE
// ----------------------
// After zeroImus() is called at full extension, Motor 2 encoder counts should
// move away from zero as the elbow flexes and return near zero at extension.
// The lower-limit check below uses the absolute distance from zero, so it works
// whether flexion produces positive or negative counts. M2_ENCSGN still
// controls the sign shown in the monitor. It is also important if the optional encoder
// upper limit is enabled later.
//
// The lower limit uses encoder counts because the encoder does not experience
// the same orientation drift as the IMUs. The upper limit uses the corrected
// elbow angle and can optionally also use an encoder count after calibration.

const float ELB_MIN = 0.0;
const float ELB_MAX = 100.0;
const float ELB_REL = 98.0;

// At full extension, encoder2 is set to zero.
// The enter and release margins stop the limit from turning on and off quickly.
const long M2_LOW = 0;
const long M2_LOW_IN = 100;
const long M2_LOW_OUT = 300;

// Optional encoder-based upper limit.
// Leave false until M2_UP has been measured safely.
const bool USE_M2_UP = false;
const long M2_UP = 90000;
const long M2_UP_REL = 89000;

// Automatic elbow-only zero correction.
// The encoder must remain at the lower limit while the elbow is nearly still.
// This corrects IMU drift without changing Motor 1's IMU reference.
const bool AUTO_ZERO = true;
const unsigned long LOW_HOLDMS = 400;
const unsigned long ZERO_WAIT = 2000;
const float LOW_VEL = 1.0;
const float MIN_ZERO = 0.50;

// Runtime safety state.
bool m2LowOn = false;
bool m2UpOn = false;
unsigned long lowStartMs = 0;
unsigned long lastZeroMs = 0;
unsigned long zeroCnt = 0;

// ----------------------
// Feedback selection
// ----------------------
// Feedback is the angle that a motor tries to control.
// FB_UPPER uses only the upper IMU.
// FB_ELBOW uses the relative angle between both IMUs.

enum FeedbackSource {
  FB_UPPER,
  FB_ELBOW
};

// Current final-product assignment:
// Motor 1 controls the upper-arm angle.
// Motor 2 controls the elbow-joint angle.
const FeedbackSource M1_FB = FB_UPPER;
const FeedbackSource M2_FB = FB_ELBOW;

// ----------------------
// Motor controller settings and runtime state
// ----------------------
// One MotorController object stores everything needed to control one motor.
// This lets the same PID function control Motor 1 and Motor 2 independently.

struct MotorController {
  // Basic motor identification
  const char* name;

  // Hardware connections and direction corrections
  int inputPin1;
  int inputPin2;
  Encoder* encoder;
  int encSign;
  int motSign;
  FeedbackSource fbSrc;

  // Control gains
  // kpPos reacts to the current position error.
  // kiInt reacts to error that continues over time.
  // kvVel multiplies desired-minus-measured velocity only while the
  // oscillation trajectory is running. Fixed-target control remains PI.
  // velLim protects the motor from a bad IMU velocity spike.
  // The effort deadband and direction threshold stop fast direction changes
  // when the control output is close to zero.
  // uFull is the signed controller-output magnitude treated as full effort.
  float kpPos;
  float kiInt;
  float kvVel;
  float velLim;
  float oscDead;
  float dirThres;
  float uFull;

  // PWM limits
  // minimumPwm helps overcome the motor deadband.
  // maximumPwm is the largest command allowed.
  // nearPwm limits speed when the motor is near a fixed target.
  int minimumPwm;
  int maximumPwm;
  int nearPwm;

  // Fixed-target behavior and safety
  float tgtTolDeg;
  float nearDeg;
  unsigned long tgtToutMs;

  // Values saved between PID updates
  float tgtAngDeg;
  float prevErrDeg;
  float intErr;
  float prevAngDeg;
  float prevOut;

  // Velocity tracking values in degrees per second.
  // During oscillation, desVel comes from the exact
  // derivative of the oscillation function. measVel is
  // calculated from the IMU angle and filtered to reduce noise.
  float desVel;
  float measVel;

  // Most recent motor command, also shown in the monitor
  float normEff;
  int pwmCommand;
  float pwmCmd;

  // Operating state
  bool ctrlOn;
  bool brakeHold;
  bool oscOn;

  // Manual mode bypasses PID and directly drives the motor at MANUAL_PWM.
  bool manOn;
  int manDir;

  unsigned long tgtStartMs;
  unsigned long oscStartMs;

  // Oscillation startup state. The motor first moves to the lower endpoint,
  // settles there, and only then starts the time-varying cosine trajectory.
  bool oscPrep;
  unsigned long oscReadyMs;

  // Extra Motor 2 values sent to the Python monitor.
  float pTerm;
  float iTerm;
  float rawVTerm;
  float vTerm;
  bool velLimOn;
  bool dirProtOn;
  int oscDir;
  float rawOut;
  float satOut;
  int satState;  // -1 low, 0 none, +1 high
  int pwmLim;
  bool nearOn;
  bool brakeOn;
  bool settled;
  bool longGap;
  float ctrlDt;
  float intDt;
  unsigned long setStartMs;
};

// Motor 1 tuning and limits.
// kvVel is used only during oscillation; fixed targets use PI control.
MotorController motor1 = {
  "M1",
  M1_IN1, M1_IN2, &encoder1,
  M1_ENCSGN, M1_DIR, M1_FB,

  // Motor 1 retains its previous behavior. A zero velocity-term limit means
  // no limit, and zero thresholds turn off the new direction protection.
  0.90, 0.05, 0.10, 0.0, 0.0, 0.0, 20.0,
  150, 255, 150,
  1.0, 5.0, 15000,

  0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0,
  0.0, 0, 0.0,
  false, false, false, false, 0, 0, 0
};

// Motor 2 tuning and limits.
// Fixed-target gains: Kp=0.75, Ki=0.055. Oscillation additionally uses
// Kv=0.05 on desired-minus-measured angular velocity.
MotorController motor2 = {
  "M2",
  M2_IN1, M2_IN2, &encoder2,
  M2_ENCSGN, M2_DIR, M2_FB,

  // Clamp the oscillation velocity contribution to +/-1.5 controller units.
  // Around zero effort, coast below 0.20 and require at least 0.75 before
  // changing the saved motor direction.
  0.75, 0.055, 0.05, 1.50, 0.20, 0.75, 20.0,
  100, 255, 130,
  1.0, 5.0, 15000,

  0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0,
  0.0, 0, 0.0,
  false, false, false, false, 0, 0, 0
};

// Existing target mapping is preserved: key 0 means 5 degrees.
const float TGT_LIST[10] = {
  5.0, 10.0, 20.0, 30.0, 40.0,
  50.0, 60.0, 70.0, 80.0, 90.0
};

// Target, oscillation, and manual arrow commands affect the selected motor.
// Motor 2 is selected when the program starts.
MotorController* selMotor = &motor2;

// ANSI arrow keys arrive as three serial bytes: ESC, [, and C or D.
// This variable remembers which part of that sequence was received.
int keyState = 0;

// ----------------------
// Oscillation settings
// ----------------------
// The target follows a smooth cosine wave. With center=45 and amplitude=35,
// it moves from 10 to 80 degrees and back. At 0.05 Hz, one complete cycle
// takes 20 seconds.

const float OSC_HZ = 0.05;
const float OSC_CTR = 45.0;
const float OSC_AMP = 35.0;

// The measured angular velocity is calculated by differentiating the IMU
// angle. Differentiation can amplify sensor noise, so this low-pass filter is
// applied before velocity error is used by the controller. A smaller value is
// smoother; a larger value reacts faster.
const float VEL_FILT = 0.20;

// Timing and safety settings.
// Actual dt is preserved for velocity estimation. Integration is skipped after
// a long gap so a delayed loop cannot create a large integral jump.
const float MAX_INT_DT = 0.05;
const float GAP_SEC = 0.05;

// A fixed target is reported as settled only after the brake state remains
// inside the target band with low measured velocity for this confirmation time.
const float SET_VEL = 2.0;
const unsigned long SET_MS = 200;

// Oscillation begins only after the lower endpoint has settled.
const unsigned long PREP_MS = 300;

// ----------------------
// Runtime timing
// ----------------------
// These variables remember when control and serial printing last ran.
// unsigned long arithmetic also handles the normal micros()/millis() rollover.

unsigned long lastCtrlUs = 0;
unsigned long lastSendMs = 0;

// ======================================================
// Function declarations
// ======================================================
// These declarations tell the compiler which functions are defined later.
// They also provide a quick list of the program's main tasks.

QuaternionData normalizeQuaternion(QuaternionData q);
QuaternionData conjugateQuaternion(QuaternionData q);
QuaternionData multiplyQuaternions(QuaternionData a, QuaternionData b);
QuaternionData fromBnoQuaternion(imu::Quaternion q);
float quaternionAngleDeg(QuaternionData q);

void resetJointAngleFilter(float startDeg);
float filterJointAngle(float rawDeg);

bool startImus();
void readImus();
void zeroImus();
void readImuCalibrationStatus();

float getFeedbackAngle(const MotorController& motor);
const char* getFeedbackName(const MotorController& motor);
long getEncoderCounts(const MotorController& motor);

bool isMotor2(const MotorController& motor);
bool readMotor2LowerLimit();
bool readMotor2UpperLimit();
bool motorDirectionBlockedByElbowSafety(
    const MotorController& motor,
    int direction
);
void clearMotorCommandAtLimit(MotorController& motor);
void correctElbowZeroAtLowerLimit();
void updateElbowSafetyState();

void driveMotor(MotorController& motor, int direction, int pwmCommand);
void motorOff(MotorController& motor);
void motorHold(MotorController& motor);

float clamp01(float value);
void resetControllerState(MotorController& motor, bool deactivate);
void initializeControllerForTarget(MotorController& motor, float tgtAngDeg);
void updateMotorController(
    MotorController& motor,
    float ctrlDt,
    float intDt,
    bool longGap
);

float calculateOscillationTargetAngle(float timeSec);
float calculateOscillationVelocity(float timeSec);
void startOscillation(MotorController& motor);
void stopOscillation(MotorController& motor);
void updateOscillationTarget(MotorController& motor);

void setFixedTarget(MotorController& motor, float tgtAngDeg);
void startManualDrive(MotorController& motor, int direction);
void updateManualDrive(MotorController& motor);
void stopMotor(MotorController& motor, const char* reason);
void stopAllMotion(const char* reason);

void checkSerialCommands();
void handleSerialCommand(char command);
void handleArrowCommand(char arrowCode);

const char* getControlModeName(const MotorController& motor);
void printQuaternion(const char* label, QuaternionData q);
void printTelemetry();
void printMenu();

// ======================================================
// Quaternion math
// ======================================================

// Makes a quaternion have a length of 1.
// Normalization is required before using it for orientation calculations.
QuaternionData normalizeQuaternion(QuaternionData q) {
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
QuaternionData conjugateQuaternion(QuaternionData q) {
  q = normalizeQuaternion(q);
  return {q.w, -q.x, -q.y, -q.z};
}

// Combines two rotations. Quaternion multiplication order is important.
QuaternionData multiplyQuaternions(QuaternionData a, QuaternionData b) {
  QuaternionData result;

  result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

  return normalizeQuaternion(result);
}

// Converts the Adafruit library quaternion into the program's QuaternionData type.
QuaternionData fromBnoQuaternion(imu::Quaternion q) {
  return normalizeQuaternion({
    (float)q.w(),
    (float)q.x(),
    (float)q.y(),
    (float)q.z()
  });
}

// Converts a quaternion rotation into one positive angle in degrees.
// This gives the size of the rotation, not a signed rotation direction.
float quaternionAngleDeg(QuaternionData q) {
  q = normalizeQuaternion(q);
  float w = constrain(fabsf(q.w), 0.0f, 1.0f);
  return 2.0 * acos(w) * 180.0 / PI;
}

// ======================================================
// IMU filtering
// ======================================================

// Starts or restarts the elbow filter at a known angle.
void resetJointAngleFilter(float startDeg) {
  rawJntDeg = startDeg;
  filtJntDeg = startDeg;
  avgJntDeg = startDeg;
  jntVar = 0.0;
  lastJntDeg = startDeg;
  filtReady = true;
  spikeCnt = 0;
}

// Checks a new elbow reading for a spike and then applies smoothing.
// A rejected spike is replaced with the last trusted filtered value.
float filterJointAngle(float rawDeg) {
  if (!filtReady) {
    resetJointAngleFilter(rawDeg);
    return rawDeg;
  }

  float jump = fabs(rawDeg - filtJntDeg);
  float avgDiff = rawDeg - avgJntDeg;
  float instVar =
      avgDiff * avgDiff;

  float varLim = max(
      jntVar * 3.0f,
      MIN_VAR
  );

  bool isSpike =
      (jump > MAX_JUMP) &&
      (instVar > varLim);

  if (isSpike) {
    spikeCnt++;
    filtJntDeg = lastJntDeg;
    return filtJntDeg;
  }

  avgJntDeg =
      AVG_A * rawDeg +
      (1.0 - AVG_A) * avgJntDeg;

  float newDiff = rawDeg - avgJntDeg;
  float newVar = newDiff * newDiff;

  jntVar =
      VAR_A * newVar +
      (1.0 - VAR_A) * jntVar;

  filtJntDeg =
      FILT_A * rawDeg +
      (1.0 - FILT_A) * filtJntDeg;

  lastJntDeg = filtJntDeg;
  return filtJntDeg;
}

// ======================================================
// IMU functions
// ======================================================

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
    imusReady = false;
    return false;
  }

  delay(1000);
  bnoUpper.setExtCrystalUse(true);
  bnoForearm.setExtCrystalUse(true);

  imusReady = true;
  readImus();
  zeroImus();

  Serial.println("IMUs started and zeroed.");
  return true;
}

// Reads both IMUs and calculates the angles used for feedback.
// The elbow angle is found from forearm orientation relative to upper-arm
// orientation. This removes motion that both arm sections share.
void readImus() {
  if (!imusReady) {
    return;
  }

  upRawQ = fromBnoQuaternion(bnoUpper.getQuat());
  frRawQ = fromBnoQuaternion(bnoForearm.getQuat());

  upRelQ = multiplyQuaternions(
      conjugateQuaternion(upZeroQ), upRawQ);

  frRelQ = multiplyQuaternions(
      conjugateQuaternion(frZeroQ), frRawQ);

  elbRelQ = multiplyQuaternions(
      conjugateQuaternion(upRelQ), frRelQ);

  upAngDeg = quaternionAngleDeg(upRelQ);
  rawJntDeg = quaternionAngleDeg(elbRelQ);

  // First filter the uncorrected relative IMU angle. Then subtract the
  // encoder-confirmed drift offset. This keeps filtering and drift correction
  // as two separate operations.
  float filtRawDeg =
      filterJointAngle(rawJntDeg);

  jntAngDeg =
      filtRawDeg - elbZeroDeg;

  // The elbow cannot physically extend below zero. Only clamp small negative
  // values caused by normal filter noise; larger negative values remain
  // visible so a bad offset can be diagnosed.
  if (jntAngDeg < 0.0 && jntAngDeg > -2.0) {
    jntAngDeg = 0.0;
  }
}

// Reads the BNO055 calibration values for the Python monitor.
// Each value ranges from 0 to 3.
void readImuCalibrationStatus() {
  if (!imusReady) {
    upSysCal = 0;
    upGyrCal = 0;
    upAccCal = 0;
    upMagCal = 0;

    frSysCal = 0;
    frGyrCal = 0;
    frAccCal = 0;
    frMagCal = 0;
    return;
  }

  bnoUpper.getCalibration(
      &upSysCal,
      &upGyrCal,
      &upAccCal,
      &upMagCal
  );

  bnoForearm.getCalibration(
      &frSysCal,
      &frGyrCal,
      &frAccCal,
      &frMagCal
  );
}

// Saves the current arm position as zero and clears the encoders.
// All motor motion is stopped first so the reference is taken safely.
void zeroImus() {
  if (!imusReady) {
    Serial.println("Cannot zero IMUs: IMUs are not ready.");
    return;
  }

  stopAllMotion("imu_zero");

  upRawQ = fromBnoQuaternion(bnoUpper.getQuat());
  frRawQ = fromBnoQuaternion(bnoForearm.getQuat());

  upZeroQ = upRawQ;
  frZeroQ = frRawQ;

  upRelQ = {1.0, 0.0, 0.0, 0.0};
  frRelQ = {1.0, 0.0, 0.0, 0.0};
  elbRelQ = {1.0, 0.0, 0.0, 0.0};

  upAngDeg = 0.0;
  rawJntDeg = 0.0;
  jntAngDeg = 0.0;
  elbZeroDeg = 0.0;
  resetJointAngleFilter(0.0);

  encoder1.write(0);
  encoder2.write(0);

  m2LowOn = true;
  m2UpOn = false;
  lowStartMs = 0;
  lastZeroMs = millis();
  zeroCnt = 0;

  motor1.tgtAngDeg = 0.0;
  motor2.tgtAngDeg = 0.0;

  Serial.println("IMUs and encoders zeroed.");
}

// ======================================================
// Feedback and motor helpers
// ======================================================

// Returns the angle selected for this motor's feedback source.
float getFeedbackAngle(const MotorController& motor) {
  if (motor.fbSrc == FB_UPPER) {
    return upAngDeg;
  }

  return jntAngDeg;
}

// Returns a readable feedback name for status messages.
const char* getFeedbackName(const MotorController& motor) {
  if (motor.fbSrc == FB_UPPER) {
    return "UpperArm";
  }

  return "ElbowJoint";
}

// Reads the motor encoder and applies its configured sign correction.
// The PID currently uses IMU angle feedback; encoder counts are sent as
// the monitor. They can also be used for future speed or position control.
long getEncoderCounts(const MotorController& motor) {
  return motor.encSign * motor.encoder->read();
}

// Returns true only for the controller connected to the elbow cable motor.
bool isMotor2(const MotorController& motor) {
  return &motor == &motor2;
}

// Reads the encoder-based lower limit with a small release margin.
// Full extension is a small WINDOW around the saved zero count. Using the
// absolute distance from zero is important: the old <= comparison treated
// every negative encoder value as being at the lower limit, which could block
// reverse/down movement during the entire range of motion when the encoder
// direction was negative.
bool readMotor2LowerLimit() {
  long curCnt = getEncoderCounts(motor2);
  long lowDist = labs(
      curCnt - M2_LOW
  );

  if (m2LowOn) {
    return lowDist <=
        M2_LOW_OUT;
  }

  return lowDist <=
      M2_LOW_IN;
}

// Reads the upper elbow safety limit with a small release margin.
// The IMU angle limit is always enabled. The encoder upper limit is optional.
bool readMotor2UpperLimit() {
  bool angleLimit;

  if (m2UpOn) {
    angleLimit =
        jntAngDeg >= ELB_REL;
  } else {
    angleLimit =
        jntAngDeg >= ELB_MAX;
  }

  bool encLim = false;

  if (USE_M2_UP) {
    long curCnt = getEncoderCounts(motor2);

    if (m2UpOn) {
      encLim =
          curCnt >= M2_UP_REL;
    } else {
      encLim =
          curCnt >= M2_UP;
    }
  }

  return angleLimit || encLim;
}

// Blocks only the unsafe direction.
// At the lower limit, Motor 2 may still move forward/up.
// At the upper limit, Motor 2 may still move reverse/down.
bool motorDirectionBlockedByElbowSafety(
    const MotorController& motor,
    int direction
) {
  if (!isMotor2(motor)) {
    return false;
  }

  if (direction == REVERSE && m2LowOn) {
    return true;
  }

  if (direction == FORWARD && m2UpOn) {
    return true;
  }

  return false;
}

// Clears motor output and all stored control effort when a mechanical limit
// blocks motion. The current target is kept, so the controller can
// move safely away from the limit when the target changes direction.
void clearMotorCommandAtLimit(MotorController& motor) {
  motorOff(motor);

  float current = getFeedbackAngle(motor);

  motor.intErr = 0.0;
  motor.prevOut = 0.0;
  motor.prevAngDeg = current;
  motor.prevErrDeg = motor.tgtAngDeg - current;
  motor.measVel = 0.0;

  motor.normEff = 0.0;
  motor.pwmCommand = 0;
  motor.pwmCmd = 0.0;
  motor.brakeHold = false;
  motor.brakeOn = false;
  motor.settled = false;
  motor.setStartMs = 0;
  motor.pTerm = 0.0;
  motor.iTerm = 0.0;
  motor.rawVTerm = 0.0;
  motor.vTerm = 0.0;
  motor.velLimOn = false;
  motor.dirProtOn = false;
  motor.oscDir = 0;
  motor.rawOut = 0.0;
  motor.satOut = 0.0;
  motor.satState = 0;
}

// Corrects only the elbow feedback zero.
// It does not replace the upper-arm or forearm quaternion zero references and
// does not stop Motor 1.
void correctElbowZeroAtLowerLimit() {
  float oldAngDeg = jntAngDeg;

  // Avoid repeated corrections for very small normal noise.
  if (fabs(oldAngDeg) <
      MIN_ZERO) {
    return;
  }

  motorOff(motor2);

  // filtJntDeg is the filtered angle before offset subtraction.
  // Saving it as the offset makes the corrected elbow angle equal to zero.
  elbZeroDeg = filtJntDeg;
  jntAngDeg = ELB_MIN;

  // Re-anchor the encoder at the known mechanical lower limit.
  encoder2.write(0);

  // A manual reverse command must not continue against the limit.
  if (motor2.manOn &&
      motor2.manDir == REVERSE) {
    motor2.manOn = false;
    motor2.manDir = 0;
  }

  clearMotorCommandAtLimit(motor2);

  lastZeroMs = millis();
  zeroCnt++;

  Serial.print("ELBOW_ZERO_CORRECTED,");
  Serial.print(lastZeroMs);
  Serial.print(",");
  Serial.print(oldAngDeg, 4);
  Serial.print(",");
  Serial.print(elbZeroDeg, 4);
  Serial.print(",");
  Serial.println(zeroCnt);
}

// Updates lower/upper limit states and performs confirmed elbow drift
// correction while the mechanism is resting at full extension.
void updateElbowSafetyState() {
  bool prevLow = m2LowOn;
  bool prevUp = m2UpOn;

  m2LowOn = readMotor2LowerLimit();
  m2UpOn = readMotor2UpperLimit();

  if (m2LowOn != prevLow) {
    Serial.print("LIMIT,M2,LOWER,");
    Serial.println(m2LowOn ? 1 : 0);
  }

  if (m2UpOn != prevUp) {
    Serial.print("LIMIT,M2,UPPER,");
    Serial.println(m2UpOn ? 1 : 0);
  }

  // Never change the elbow zero while Motor 2 is under direct manual control.
  // Manual movement is often used to leave the lower limit, and an automatic
  // zero correction during that movement can briefly remove the motor output.
  if (!AUTO_ZERO ||
      !m2LowOn ||
      motor2.manOn) {
    lowStartMs = 0;
    return;
  }

  bool elbStill =
      fabs(motor2.measVel) <=
      LOW_VEL;

  if (!elbStill) {
    lowStartMs = 0;
    return;
  }

  unsigned long now = millis();

  if (lowStartMs == 0) {
    lowStartMs = now;
    return;
  }

  bool heldLong =
      now - lowStartMs >=
      LOW_HOLDMS;

  bool intvOk =
      now - lastZeroMs >=
      ZERO_WAIT;

  if (heldLong && intvOk) {
    correctElbowZeroAtLowerLimit();

    // Require another complete confirmation period before a later correction.
    lowStartMs = now;
  }
}

// Sends direction and PWM to one motor driver.
// The motSign is applied here so the PID logic can use the same
// FORWARD and REVERSE meanings for both motors.
void driveMotor(MotorController& motor, int direction, int pwmCommand) {
  pwmCommand = constrain(pwmCommand, 0, 255);

  if (pwmCommand <= 0 || direction == 0) {
    motorOff(motor);
    return;
  }

  int actDir = direction * motor.motSign;

  if (actDir > 0) {
    analogWrite(motor.inputPin1, pwmCommand);
    analogWrite(motor.inputPin2, 0);
  } else {
    analogWrite(motor.inputPin1, 0);
    analogWrite(motor.inputPin2, pwmCommand);
  }
}

// Removes voltage commands from both driver inputs so the motor can coast.
void motorOff(MotorController& motor) {
  analogWrite(motor.inputPin1, 0);
  analogWrite(motor.inputPin2, 0);
}

// Applies active braking by setting both driver inputs high.
// DRV8871 truth table: IN1=1 and IN2=1 selects brake / slow decay.
void motorHold(MotorController& motor) {
  // Active braking. Replace with motorOff(motor) if the driver should coast.
  analogWrite(motor.inputPin1, 255);
  analogWrite(motor.inputPin2, 255);
}

// ======================================================
// PI and oscillation velocity-tracking controller
// ======================================================

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
    motor.ctrlOn = false;
    motor.oscOn = false;
    motor.oscPrep = false;
    motor.oscStartMs = 0;
  }

  motor.brakeHold = false;
  motor.manOn = false;
  motor.manDir = 0;
  motor.prevErrDeg = 0.0;
  motor.intErr = 0.0;
  motor.prevAngDeg = getFeedbackAngle(motor);
  motor.prevOut = 0.0;
  motor.desVel = 0.0;
  motor.measVel = 0.0;
  motor.normEff = 0.0;
  motor.pwmCommand = 0;
  motor.pwmCmd = 0.0;
  motor.tgtStartMs = millis();
  motor.oscReadyMs = 0;
  motor.pTerm = 0.0;
  motor.iTerm = 0.0;
  motor.rawVTerm = 0.0;
  motor.vTerm = 0.0;
  motor.velLimOn = false;
  motor.dirProtOn = false;
  motor.oscDir = 0;
  motor.rawOut = 0.0;
  motor.satOut = 0.0;
  motor.satState = 0;
  motor.pwmLim = motor.maximumPwm;
  motor.nearOn = false;
  motor.brakeOn = false;
  motor.settled = false;
  motor.longGap = false;
  motor.ctrlDt = 0.0;
  motor.intDt = 0.0;
  motor.setStartMs = 0;
}

// Prepares one motor to begin controlling a new target.
// Clearing the old integral and derivative history prevents an old command
// from affecting the new movement.
void initializeControllerForTarget(
    MotorController& motor,
    float tgtAngDeg
) {
  float current = getFeedbackAngle(motor);

  motor.tgtAngDeg = tgtAngDeg;
  motor.ctrlOn = true;
  motor.brakeHold = false;
  motor.manOn = false;
  motor.manDir = 0;
  motor.prevErrDeg = tgtAngDeg - current;
  motor.intErr = 0.0;
  motor.prevAngDeg = current;
  motor.prevOut = 0.0;
  motor.desVel = 0.0;
  motor.measVel = 0.0;
  motor.normEff = 0.0;
  motor.pwmCommand = 0;
  motor.pwmCmd = 0.0;
  motor.tgtStartMs = millis();
  motor.pTerm = 0.0;
  motor.iTerm = 0.0;
  motor.rawVTerm = 0.0;
  motor.vTerm = 0.0;
  motor.velLimOn = false;
  motor.dirProtOn = false;
  motor.oscDir = 0;
  motor.rawOut = 0.0;
  motor.satOut = 0.0;
  motor.satState = 0;
  motor.pwmLim = motor.maximumPwm;
  motor.nearOn = false;
  motor.brakeOn = false;
  motor.settled = false;
  motor.longGap = false;
  motor.ctrlDt = 0.0;
  motor.intDt = 0.0;
  motor.setStartMs = 0;
}

// Runs one complete PI/velocity-tracking update for one motor.
//
// The important steps are:
// 1. Read the selected feedback angle.
// 2. Calculate target error.
// 3. Check target tolerance and safety timeout.
// 4. Calculate measured angular velocity from the IMU angle.
// 5. Compare measured velocity with the desired trajectory velocity.
// 6. Update the integral term with anti-windup protection.
// 7. Calculate PI output plus oscillation-only velocity effort.
// 8. Convert output magnitude to PWM and output sign to direction.
void updateMotorController(
    MotorController& motor,
    float ctrlDt,
    float intDt,
    bool longGap
) {
  // Save timing values for synchronized diagnostics.
  motor.ctrlDt = ctrlDt;
  motor.intDt = intDt;
  motor.longGap = longGap;

  // Manual mode directly controls the motor, so feedback control must not
  // overwrite its command.
  if (motor.manOn) {
    return;
  }

  // Start every feedback-control update with clear diagnostic flags.
  motor.pTerm = 0.0;
  motor.iTerm = motor.kiInt * motor.intErr;
  motor.rawVTerm = 0.0;
  motor.vTerm = 0.0;
  motor.velLimOn = false;
  motor.dirProtOn = false;
  motor.rawOut = 0.0;
  motor.satOut = 0.0;
  motor.satState = 0;
  motor.pwmLim = motor.maximumPwm;
  motor.nearOn = false;
  motor.brakeOn = false;

  // Never drive a motor without valid IMU feedback or an active target.
  if (!imusReady || !motor.ctrlOn) {
    motorOff(motor);
    motor.normEff = 0.0;
    motor.pwmCommand = 0;
    motor.pwmCmd = 0.0;
    motor.settled = false;
    motor.setStartMs = 0;
    return;
  }

  // Positive error means the measured angle is below the target.
  // Negative error means the measured angle is above the target.
  float current = getFeedbackAngle(motor);
  float error = motor.tgtAngDeg - current;
  float absErr = fabs(error);

  // Calculate measured angular velocity before checking the target band so
  // The monitor still receives a real velocity value while the brake is active.
  if (longGap || ctrlDt <= 0.0) {
    motor.measVel = 0.0;
  } else {
    float rawMeasV =
        (current - motor.prevAngDeg) / ctrlDt;

    motor.measVel =
        VEL_FILT * rawMeasV +
        (1.0 - VEL_FILT) *
            motor.measVel;
  }

  float velErr =
      motor.desVel -
      motor.measVel;

  bool errFlip =
      (error > 0.0 && motor.prevErrDeg < 0.0) ||
      (error < 0.0 && motor.prevErrDeg > 0.0);

  // Reset integral on a fixed-target crossing, but not while tracking the
  // moving oscillation reference. Repeated resets during oscillation would
  // create discontinuities and prevent correction of persistent bias.
  if (!motor.oscOn && errFlip) {
    motor.intErr = 0.0;
  }

  bool fixedTgt = false;

  if (!motor.oscOn) {
    float exitTol = motor.tgtTolDeg * 1.5;
    fixedTgt = motor.brakeHold
        ? (absErr <= exitTol)
        : (absErr <= motor.tgtTolDeg);
  }

  // Fixed targets and oscillation pre-positioning use the DRV8871 brake state
  // inside the target band. Running oscillation never stops at each moving
  // target because its reference is continuously changing.
  if (fixedTgt) {
    motorHold(motor);
    motor.normEff = 0.0;
    motor.pwmCommand = 0;
    motor.pwmCmd = 0.0;
    motor.intErr = 0.0;
    motor.iTerm = 0.0;
    motor.prevErrDeg = error;
    motor.prevAngDeg = current;
    motor.prevOut = 0.0;
    motor.brakeHold = true;
    motor.brakeOn = true;
    motor.pwmLim = 0;
    motor.tgtStartMs = millis();

    // Settled is a reporting condition only; braking begins immediately when
    // position enters the tolerance band.
    if (!longGap &&
        fabs(motor.measVel) <=
            SET_VEL) {
      if (motor.setStartMs == 0) {
        motor.setStartMs = millis();
      }

      motor.settled =
          millis() - motor.setStartMs >= SET_MS;
    } else {
      motor.setStartMs = 0;
      motor.settled = false;
    }

    return;
  }

  if (motor.brakeHold) {
    motor.brakeHold = false;
    motor.intErr = 0.0;
    motor.prevErrDeg = error;
    motor.prevAngDeg = current;
    motor.prevOut = 0.0;
    motor.tgtStartMs = millis();
  }

  motor.setStartMs = 0;
  motor.settled = false;

  // Stop a fixed-target or oscillation-preposition movement if it takes too
  // long. Running oscillation is exempt because it is intentionally continuous.
  if (!motor.oscOn &&
      millis() - motor.tgtStartMs > motor.tgtToutMs) {
    stopMotor(motor, "fixed_target_timeout");
    return;
  }

  // Use a lower PWM range near a fixed target. Running oscillation retains the
  // full PWM range so its position and velocity terms can track the trajectory.
  int pwmLimit = motor.maximumPwm;

  if (!motor.oscOn &&
      absErr <= motor.nearDeg) {
    pwmLimit = motor.nearPwm;
    motor.nearOn = true;
  }

  motor.pwmLim = pwmLimit;
  motor.pTerm = motor.kpPos * error;

  // Velocity tracking is active only during oscillation.
  // The raw velocity term is saved for the monitor, then limited before it can
  // affect anti-windup, saturation calculations, or the physical motor command.
  motor.rawVTerm = motor.oscOn
      ? motor.kvVel * velErr
      : 0.0;

  float velLim = max(
      fabsf(motor.velLim),
      0.0f
  );

  if (motor.oscOn && velLim > 0.0f) {
    motor.vTerm = constrain(
        motor.rawVTerm,
        -velLim,
        velLim
    );
  } else {
    motor.vTerm = motor.rawVTerm;
  }

  motor.velLimOn =
      fabsf(motor.rawVTerm - motor.vTerm) > 0.000001f;

  // Current-cycle, sign-aware conditional integration. First calculate the
  // integral and output that would result if this cycle were accepted.
  float intTry = motor.intErr;

  if (intDt > 0.0) {
    intTry += error * intDt;
  }

  if (fabs(motor.kiInt) > 0.000001) {
    float intLim = fabs(motor.uFull / motor.kiInt);
    intTry = constrain(
        intTry,
        -intLim,
        intLim
    );
  } else {
    intTry = 0.0;
    motor.intErr = 0.0;
  }

  float outTry =
      motor.pTerm +
      motor.kiInt * intTry +
      motor.vTerm;

  bool satHigh = outTry > motor.uFull;
  bool satLow = outTry < -motor.uFull;

  // Normal integration is allowed when unsaturated. While saturated, permit
  // only an error direction that moves the controller back toward the range.
  bool intOk =
      intDt > 0.0 &&
      ((!satHigh && !satLow) ||
       (satHigh && error < 0.0) ||
       (satLow && error > 0.0));

  if (intOk) {
    motor.intErr = intTry;
  }

  // Retain a hard integral limit as a backup to conditional anti-windup.
  if (fabs(motor.kiInt) > 0.000001) {
    float intLim = fabs(motor.uFull / motor.kiInt);
    motor.intErr = constrain(
        motor.intErr,
        -intLim,
        intLim
    );
  } else {
    motor.intErr = 0.0;
  }

  motor.iTerm = motor.kiInt * motor.intErr;
  motor.rawOut =
      motor.pTerm +
      motor.iTerm +
      motor.vTerm;

  if (motor.rawOut > motor.uFull) {
    motor.satState = 1;
  } else if (motor.rawOut < -motor.uFull) {
    motor.satState = -1;
  } else {
    motor.satState = 0;
  }

  // Explicitly clamp the signed controller effort before PWM mapping. This
  // keeps the monitor values and the real motor command consistent.
  motor.satOut = constrain(
      motor.rawOut,
      -motor.uFull,
      motor.uFull
  );

  float ctrlOut = motor.satOut;

  // Running oscillation uses a small output deadband and a larger reversal
  // threshold. This prevents a tiny sign change from immediately becoming a
  // full minimum-PWM command in the opposite direction. While the controller
  // is inside either protection region, the driver coasts. The integral is
  // still allowed to build enough meaningful effort to leave the region.
  if (motor.oscOn) {
    float absOut = fabsf(ctrlOut);
    int reqDir = ctrlOut > 0.0 ? FORWARD : REVERSE;

    bool inDead =
        absOut < motor.oscDead;

    bool weakRev =
        motor.oscDir != 0 &&
        reqDir != motor.oscDir &&
        absOut < motor.dirThres;

    if (inDead || weakRev) {
      motorOff(motor);
      motor.dirProtOn = true;
      motor.prevErrDeg = error;
      motor.prevAngDeg = current;
      motor.prevOut = 0.0;
      motor.normEff = 0.0;
      motor.pwmCommand = 0;
      motor.pwmCmd = 0.0;
      return;
    }

    motor.oscDir = reqDir;
  } else {
    motor.oscDir = 0;
  }

  if (fabs(ctrlOut) < 0.0001) {
    motorOff(motor);
    motor.prevErrDeg = error;
    motor.prevAngDeg = current;
    motor.prevOut = 0.0;
    motor.normEff = 0.0;
    motor.pwmCommand = 0;
    motor.pwmCmd = 0.0;
    return;
  }

  int direction = ctrlOut > 0.0 ? FORWARD : REVERSE;

  // If Motor 2 is at a mechanical limit, block only the direction that would
  // push farther into the limit and erase all stored controller effort.
  if (motorDirectionBlockedByElbowSafety(motor, direction)) {
    clearMotorCommandAtLimit(motor);
    return;
  }

  motor.prevErrDeg = error;
  motor.prevAngDeg = current;
  motor.prevOut = ctrlOut;

  float fullOutput = max(fabsf(motor.uFull), 0.000001f);
  float normEff =
      clamp01(fabs(ctrlOut) / fullOutput);

  int minimumPwm = min(motor.minimumPwm, pwmLimit);
  int pwmCommand = minimumPwm +
      (int)((pwmLimit - minimumPwm) * normEff);

  float signedPwm =
      direction * motor.motSign * pwmCommand;

  motor.normEff = normEff;
  motor.pwmCommand = pwmCommand;
  motor.pwmCmd = signedPwm;

  driveMotor(motor, direction, pwmCommand);
}

// ======================================================
// Fixed targets and oscillation
// ======================================================

// Calculates the moving target for oscillation.
// At time zero the target is center-amplitude. It rises smoothly to
// center+amplitude, returns, and then repeats. With the current settings,
// the planned range is 10 to 80 degrees.
float calculateOscillationTargetAngle(float timeSec) {
  return OSC_CTR -
      OSC_AMP *
      cos(2.0 * PI * OSC_HZ * timeSec);
}

// Calculates desired angular velocity by differentiating the target function.
//
// target(t) = center - amplitude * cos(2*pi*f*t)
// velocity(t) = amplitude * 2*pi*f * sin(2*pi*f*t)
//
// The result is in degrees per second because amplitude is in degrees.
float calculateOscillationVelocity(float timeSec) {
  float angFreq =
      2.0 * PI * OSC_HZ;

  return OSC_AMP *
      angFreq *
      sin(angFreq * timeSec);
}

// Starts oscillation by first moving to the lower endpoint. The actual cosine
// trajectory begins only after the endpoint is settled for a short time.
void startOscillation(MotorController& motor) {
  motor.manOn = false;
  motor.manDir = 0;
  motor.oscOn = false;
  motor.oscPrep = true;
  motor.oscStartMs = 0;
  motor.oscReadyMs = 0;

  initializeControllerForTarget(
      motor,
      calculateOscillationTargetAngle(0.0)
  );
  motor.desVel = 0.0;

  Serial.print(motor.name);
  Serial.print(" oscillation preparing at ");
  Serial.print(motor.tgtAngDeg, 2);
  Serial.println(" degrees.");
}

// Clears both oscillation-running and oscillation-preparation states.
void stopOscillation(MotorController& motor) {
  motor.oscOn = false;
  motor.oscPrep = false;
  motor.oscStartMs = 0;
  motor.oscReadyMs = 0;
  motor.desVel = 0.0;
  motor.rawVTerm = 0.0;
  motor.vTerm = 0.0;
  motor.velLimOn = false;
  motor.dirProtOn = false;
  motor.oscDir = 0;
}

// Updates preparation or the running oscillation target every control cycle.
void updateOscillationTarget(MotorController& motor) {
  if (motor.oscPrep) {
    motor.tgtAngDeg = calculateOscillationTargetAngle(0.0);
    motor.desVel = 0.0;
    motor.ctrlOn = true;

    if (motor.brakeHold && motor.settled) {
      if (motor.oscReadyMs == 0) {
        motor.oscReadyMs = millis();
      }

      if (millis() - motor.oscReadyMs >=
          PREP_MS) {
        motor.oscPrep = false;
        motor.oscOn = true;
        motor.oscStartMs = millis();
        motor.oscReadyMs = 0;

        initializeControllerForTarget(
            motor,
            calculateOscillationTargetAngle(0.0)
        );
        motor.desVel =
            calculateOscillationVelocity(0.0);

        Serial.print(motor.name);
        Serial.print(" oscillation running at ");
        Serial.print(OSC_HZ, 3);
        Serial.print(" Hz. Maximum desired speed: ");
        Serial.print(
            OSC_AMP *
            2.0 * PI * OSC_HZ,
            2
        );
        Serial.println(" deg/s.");
      }
    } else {
      motor.oscReadyMs = 0;
    }

    return;
  }

  if (!motor.oscOn) {
    return;
  }

  float runSec =
      (millis() - motor.oscStartMs) / 1000.0;

  motor.tgtAngDeg = calculateOscillationTargetAngle(runSec);
  motor.desVel =
      calculateOscillationVelocity(runSec);
  motor.ctrlOn = true;
  motor.brakeHold = false;
}

// Stops oscillation and starts movement toward one fixed angle.
// The allowed target range is limited to 0 through 90 degrees.
void setFixedTarget(MotorController& motor, float tgtAngDeg) {
  motor.manOn = false;
  motor.manDir = 0;
  stopOscillation(motor);
  tgtAngDeg = constrain(tgtAngDeg, 0.0f, 90.0f);
  initializeControllerForTarget(motor, tgtAngDeg);

  Serial.print(motor.name);
  Serial.print(" fixed target set to ");
  Serial.print(tgtAngDeg, 2);
  Serial.print(" degrees using ");
  Serial.print(getFeedbackName(motor));
  Serial.println(" feedback.");
}


// Starts manual open-loop movement for the selected motor.
// Unlike PID control, this mode does not use an angle target and will keep
// moving until another command changes the mode or stops the motors.
void startManualDrive(MotorController& motor, int direction) {
  if (!imusReady) {
    motorOff(motor);
    Serial.print("Manual control blocked for ");
    Serial.print(motor.name);
    Serial.println(": IMUs are not ready.");
    return;
  }

  // Convert every manual request into one of the two valid logical directions.
  int reqDir = direction >= 0 ? FORWARD : REVERSE;

  // Refresh the sensor and limit states immediately. This avoids using a stale
  // limit flag when a manual command arrives between normal 100 Hz updates.
  readImus();
  updateElbowSafetyState();

  // Block only a direction that would push Motor 2 farther into a mechanical
  // limit. The opposite direction remains available so the user can move away.
  if (motorDirectionBlockedByElbowSafety(
          motor,
          reqDir
      )) {
    motorOff(motor);
    resetControllerState(motor, true);
    motor.tgtAngDeg = getFeedbackAngle(motor);

    Serial.print("MANUAL_BLOCKED,");
    Serial.print(motor.name);
    Serial.print(",");
    Serial.print(
        reqDir == FORWARD
            ? "upper_limit"
            : "lower_limit"
    );
    Serial.print(",counts=");
    Serial.print(getEncoderCounts(motor));
    Serial.print(",angle=");
    Serial.println(getFeedbackAngle(motor), 3);
    return;
  }

  stopOscillation(motor);

  motor.ctrlOn = false;
  motor.brakeHold = false;
  motor.manOn = true;
  motor.manDir = reqDir;

  motor.tgtAngDeg = getFeedbackAngle(motor);
  motor.prevErrDeg = 0.0;
  motor.intErr = 0.0;
  motor.prevAngDeg = getFeedbackAngle(motor);
  motor.prevOut = 0.0;
  motor.desVel = 0.0;
  motor.measVel = 0.0;

  // Apply the command immediately instead of waiting for the next loop cycle.
  updateManualDrive(motor);

  Serial.print(motor.name);
  Serial.print(" manual mode: ");
  Serial.print(motor.manDir == FORWARD ? "forward/up" : "reverse/down");
  Serial.print(" at PWM ");
  Serial.println(MANUAL_PWM);
}

// Keeps the manual PWM command active and updates the monitor values.
void updateManualDrive(MotorController& motor) {
  if (!motor.manOn) {
    return;
  }

  if (!imusReady) {
    motorOff(motor);
    motor.pwmCommand = 0;
    motor.normEff = 0.0;
    motor.pwmCmd = 0.0;
    return;
  }

  if (motorDirectionBlockedByElbowSafety(
          motor,
          motor.manDir
      )) {
    int blockDir = motor.manDir;

    motor.manOn = false;
    motor.manDir = 0;
    clearMotorCommandAtLimit(motor);

    Serial.print("MANUAL_LIMIT_STOP,");
    Serial.print(motor.name);
    Serial.print(",");
    Serial.print(
        blockDir == FORWARD
            ? "upper_limit"
            : "lower_limit"
    );
    Serial.print(",counts=");
    Serial.print(getEncoderCounts(motor));
    Serial.print(",angle=");
    Serial.println(getFeedbackAngle(motor), 3);
    return;
  }

  int pwmCommand = constrain(MANUAL_PWM, 0, motor.maximumPwm);
  float normalized = motor.maximumPwm > 0
      ? (float)pwmCommand / (float)motor.maximumPwm
      : 0.0;

  motor.pwmCommand = pwmCommand;
  motor.normEff = clamp01(normalized);
  motor.pwmCmd =
      motor.manDir * motor.motSign * pwmCommand;
  motor.tgtAngDeg = getFeedbackAngle(motor);
  motor.pTerm = 0.0;
  motor.iTerm = 0.0;
  motor.rawVTerm = 0.0;
  motor.vTerm = 0.0;
  motor.velLimOn = false;
  motor.dirProtOn = false;
  motor.oscDir = 0;
  motor.rawOut = 0.0;
  motor.satOut = 0.0;
  motor.satState = 0;
  motor.pwmLim = motor.maximumPwm;
  motor.nearOn = false;
  motor.brakeOn = false;
  motor.settled = false;
  motor.setStartMs = 0;

  driveMotor(motor, motor.manDir, pwmCommand);
}

// Safely stops and deactivates one motor.
// The reason is printed so the monitor can show why the motor stopped.
void stopMotor(MotorController& motor, const char* reason) {
  stopOscillation(motor);
  motorOff(motor);
  resetControllerState(motor, true);
  motor.tgtAngDeg = getFeedbackAngle(motor);

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

  motor1.tgtAngDeg = getFeedbackAngle(motor1);
  motor2.tgtAngDeg = getFeedbackAngle(motor2);

  Serial.print("STOP,ALL,");
  Serial.println(reason);
}

// ======================================================
// Serial commands
// ======================================================

// Reads every available serial character without blocking the control loop.
// Arrow keys are received as ANSI escape sequences: ESC [ D for left and
// ESC [ C for right. Other commands are single characters.
void checkSerialCommands() {
  while (Serial.available() > 0) {
    char command = Serial.read();

    if (keyState == 0) {
      if (command == 27) {
        keyState = 1;
      } else if (command != '\n' && command != '\r') {
        handleSerialCommand(command);
      }
      continue;
    }

    if (keyState == 1) {
      keyState = command == '[' ? 2 : 0;
      continue;
    }

    if (keyState == 2) {
      handleArrowCommand(command);
      keyState = 0;
    }
  }
}

// Connects each keyboard/serial command to a controller action.
void handleSerialCommand(char command) {
  if (command == 'j' || command == 'J') {
    selMotor = &motor1;
    Serial.println("Selected Motor 1.");
    return;
  }

  if (command == 'k' || command == 'K') {
    selMotor = &motor2;
    Serial.println("Selected Motor 2.");
    return;
  }

  if (command >= '0' && command <= '9') {
    int index = command - '0';
    setFixedTarget(*selMotor, TGT_LIST[index]);
    return;
  }

  if (command == 'x' || command == 'X') {
    if (selMotor->oscOn ||
        selMotor->oscPrep) {
      stopMotor(*selMotor, "oscillation_stopped");
    } else {
      startOscillation(*selMotor);
    }
    return;
  }

  // The Python visualizer sends simple a/d commands for the arrow keys.
  // Single-byte commands are more dependable than terminal escape sequences.
  if (command == 'a' || command == 'A') {
    startManualDrive(*selMotor, REVERSE);
    return;
  }

  if (command == 'd' || command == 'D') {
    startManualDrive(*selMotor, FORWARD);
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
    startManualDrive(*selMotor, REVERSE);
    return;
  }

  if (arrowCode == 'C') {
    startManualDrive(*selMotor, FORWARD);
  }
}

// ======================================================
// Telemetry
// ======================================================

// Converts internal motor state into a readable mode name.
const char* getControlModeName(const MotorController& motor) {
  if (motor.manOn) {
    return "Manual";
  }

  if (motor.oscPrep) {
    return "OscPrep";
  }

  if (motor.oscOn) {
    return "Oscillation";
  }

  if (motor.brakeHold) {
    return "HoldBrake";
  }

  if (motor.ctrlOn) {
    return "FixedTarget";
  }

  return "Idle";
}

// Prints one quaternion for the Python 3D arm display.
void printQuaternion(const char* label, QuaternionData q) {
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

  if (now - lastSendMs < SEND_MS) {
    return;
  }

  lastSendMs = now;

  float m1Current = getFeedbackAngle(motor1);
  float m2Current = getFeedbackAngle(motor2);

  float m1Error = motor1.ctrlOn && !motor1.manOn
      ? motor1.tgtAngDeg - m1Current
      : 0.0;

  float m2Error = motor2.ctrlOn && !motor2.manOn
      ? motor2.tgtAngDeg - m2Current
      : 0.0;

  readImuCalibrationStatus();

  // Calibration data is separate so the existing STATE parser remains
  // compatible with older Python visualizers.
  //
  // CALIBRATION,time_ms,
  // upper_system,upper_gyro,upper_accel,upper_mag,
  // forearm_system,forearm_gyro,forearm_accel,forearm_mag
  Serial.print("CALIBRATION,");
  Serial.print(now);
  Serial.print(",");
  Serial.print(upSysCal);
  Serial.print(",");
  Serial.print(upGyrCal);
  Serial.print(",");
  Serial.print(upAccCal);
  Serial.print(",");
  Serial.print(upMagCal);
  Serial.print(",");
  Serial.print(frSysCal);
  Serial.print(",");
  Serial.print(frGyrCal);
  Serial.print(",");
  Serial.print(frAccCal);
  Serial.print(",");
  Serial.println(frMagCal);

  // Safety data:
  // SAFETY,time_ms,lower_active,upper_active,encoder_counts,
  // raw_joint,filtered_uncorrected,zero_offset,corrected_joint,corrections
  Serial.print("SAFETY,");
  Serial.print(now);
  Serial.print(",");
  Serial.print(m2LowOn ? 1 : 0);
  Serial.print(",");
  Serial.print(m2UpOn ? 1 : 0);
  Serial.print(",");
  Serial.print(getEncoderCounts(motor2));
  Serial.print(",");
  Serial.print(rawJntDeg, 4);
  Serial.print(",");
  Serial.print(filtJntDeg, 4);
  Serial.print(",");
  Serial.print(elbZeroDeg, 4);
  Serial.print(",");
  Serial.print(jntAngDeg, 4);
  Serial.print(",");
  Serial.println(zeroCnt);

  // Send the exact velocity values used inside the controller.
  // The Python monitor reads this line directly.
  // Sending it before STATE makes sure the matching velocity values are
  // available when the Python program receives the new state sample.
  //
  // VELOCITY,time_ms,
  // m1_desired_velocity,m1_measured_velocity,
  // m2_desired_velocity,m2_measured_velocity
  Serial.print("VELOCITY,");
  Serial.print(now);
  Serial.print(",");
  Serial.print(motor1.desVel, 4);
  Serial.print(",");
  Serial.print(motor1.measVel, 4);
  Serial.print(",");
  Serial.print(motor2.desVel, 4);
  Serial.print(",");
  Serial.println(motor2.measVel, 4);

  // Motor 2 controller diagnostics. This record is timestamp-matched with the
  // following STATE line by the Python visualizer and live monitor.
  // CONTROL2,time_ms,p_term,i_term,velocity_term,integral_state,
  // unsaturated_output,saturated_output,saturation_state,active_pwm_limit,
  // slow_zone,brake_active,settled,actual_dt,integration_dt,long_gap,
  // raw_velocity_term,velocity_limit_active,direction_protection_active,
  // oscillation_drive_direction
  Serial.print("CONTROL2,");
  Serial.print(now);
  Serial.print(",");
  Serial.print(motor2.pTerm, 6);
  Serial.print(",");
  Serial.print(motor2.iTerm, 6);
  Serial.print(",");
  Serial.print(motor2.vTerm, 6);
  Serial.print(",");
  Serial.print(motor2.intErr, 6);
  Serial.print(",");
  Serial.print(motor2.rawOut, 6);
  Serial.print(",");
  Serial.print(motor2.satOut, 6);
  Serial.print(",");
  Serial.print(motor2.satState);
  Serial.print(",");
  Serial.print(motor2.pwmLim);
  Serial.print(",");
  Serial.print(motor2.nearOn ? 1 : 0);
  Serial.print(",");
  Serial.print(motor2.brakeOn ? 1 : 0);
  Serial.print(",");
  Serial.print(motor2.settled ? 1 : 0);
  Serial.print(",");
  Serial.print(motor2.ctrlDt, 6);
  Serial.print(",");
  Serial.print(motor2.intDt, 6);
  Serial.print(",");
  Serial.print(motor2.longGap ? 1 : 0);
  Serial.print(",");
  Serial.print(motor2.rawVTerm, 6);
  Serial.print(",");
  Serial.print(motor2.velLimOn ? 1 : 0);
  Serial.print(",");
  Serial.print(motor2.dirProtOn ? 1 : 0);
  Serial.print(",");
  Serial.println(motor2.oscDir);

  // Read by final_dual_motor_monitor.py.
  // STATE,time_ms,selected,
  // m1_mode,m1_target,m1_current,m1_error,m1_pwm,m1_u,m1_counts,
  // m2_mode,m2_target,m2_current,m2_error,m2_pwm,m2_u,m2_counts,
  // upper_angle,elbow_angle,rejected_spikes
  Serial.print("STATE,");
  Serial.print(now);
  Serial.print(",");
  Serial.print(selMotor->name);
  Serial.print(",");

  Serial.print(getControlModeName(motor1));
  Serial.print(",");
  Serial.print(motor1.tgtAngDeg, 4);
  Serial.print(",");
  Serial.print(m1Current, 4);
  Serial.print(",");
  Serial.print(m1Error, 4);
  Serial.print(",");
  Serial.print(motor1.pwmCommand);
  Serial.print(",");
  Serial.print(motor1.pwmCmd, 2);
  Serial.print(",");
  Serial.print(getEncoderCounts(motor1));
  Serial.print(",");

  Serial.print(getControlModeName(motor2));
  Serial.print(",");
  Serial.print(motor2.tgtAngDeg, 4);
  Serial.print(",");
  Serial.print(m2Current, 4);
  Serial.print(",");
  Serial.print(m2Error, 4);
  Serial.print(",");
  Serial.print(motor2.pwmCommand);
  Serial.print(",");
  Serial.print(motor2.pwmCmd, 2);
  Serial.print(",");
  Serial.print(getEncoderCounts(motor2));
  Serial.print(",");

  Serial.print(upAngDeg, 4);
  Serial.print(",");
  Serial.print(jntAngDeg, 4);
  Serial.print(",");
  Serial.println(spikeCnt);

  printQuaternion("qUpperZeroed", upRelQ);
  printQuaternion("qForearmZeroed", frRelQ);
  printQuaternion("qJointZeroed", elbRelQ);
}

// Prints the available serial commands and current feedback assignments.
void printMenu() {
  Serial.println();
  Serial.println("========== DUAL-MOTOR CONTROLLER ==========");
  Serial.println("j       : Select Motor 1");
  Serial.println("k       : Select Motor 2");
  Serial.println("0-9     : Set selected motor to a preset target");
  Serial.println("x       : Start or stop smooth oscillation");
  Serial.println("left / a : Selected motor reverse/down at manual PWM");
  Serial.println("right / d: Selected motor forward/up at manual PWM");
  Serial.println("s/e/p   : Emergency stop both motors");
  Serial.println("space   : Emergency stop both motors");
  Serial.println("r       : Re-zero both IMUs and encoders");
  Serial.println("m       : Print this menu");
  Serial.println();
  Serial.print("Motor 1 feedback: ");
  Serial.println(getFeedbackName(motor1));
  Serial.print("Motor 2 feedback: ");
  Serial.println(getFeedbackName(motor2));
  Serial.print("Currently selected: ");
  Serial.println(selMotor->name);
  Serial.print("Motor 2 lower encoder limit counts: ");
  Serial.println(M2_LOW);
  Serial.print("Motor 2 upper safe angle: ");
  Serial.print(ELB_MAX, 1);
  Serial.println(" deg");
  Serial.print("Elbow automatic zero correction: ");
  Serial.println(AUTO_ZERO ? "enabled" : "disabled");
  Serial.println("=================================================");
  Serial.println();
}

// ======================================================
// Arduino setup and loop
// ======================================================

// setup() runs once after the board powers on or resets.
// It starts serial communication, prepares the motor pins, starts the IMUs,
// clears both controllers, and prints the command menu.
void setup() {
  Serial.begin(BAUD_RATE);

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

  lastCtrlUs = micros();
  lastSendMs = 0;

  printMenu();

  if (imusReady && AUTO_M1) {
    startOscillation(motor1);
  }

  if (imusReady && AUTO_M2) {
    startOscillation(motor2);
  }
}

// loop() runs continuously while the board is powered.
// Serial commands are checked as often as possible. The full sensor and PID
// update runs every 10 milliseconds, which produces a 100 Hz control rate.
void loop() {
  checkSerialCommands();

  unsigned long nowUs = micros();

  // Continue printing live data while waiting for the next 100 Hz control
  // update. Returning here prevents the PID from running too quickly.
  if (nowUs - lastCtrlUs < CTRL_US) {
    printTelemetry();
    return;
  }

  float ctrlDt =
      (nowUs - lastCtrlUs) / 1000000.0;
  lastCtrlUs = nowUs;

  if (ctrlDt <= 0.0) {
    ctrlDt = CTRL_US / 1000000.0;
  }

  bool longGap =
      ctrlDt > GAP_SEC;

  // Preserve actual dt for velocity. Skip integration after a long gap so a
  // delayed loop cannot create an abrupt stored-error increase.
  float intDt = longGap
      ? 0.0
      : min(ctrlDt, MAX_INT_DT);

  // Sensor readings must be updated before target error is calculated.
  readImus();

  // Oscillation changes the targets before the PID controllers run.
  updateOscillationTarget(motor1);
  updateOscillationTarget(motor2);

  // Update encoder/angle limit states before either manual or PID output is
  // allowed to reach the motor driver. This also performs confirmed
  // elbow-only drift correction at full extension.
  updateElbowSafetyState();

  // Manual mode bypasses PID and applies a direct PWM command. A motor that
  // is not in manual mode continues using its normal PID controller.
  updateManualDrive(motor1);
  updateManualDrive(motor2);

  // Each motor uses the same feedback-control function but has independent
  // settings and saved state. updateMotorController() returns immediately in manual mode.
  updateMotorController(
      motor1,
      ctrlDt,
      intDt,
      longGap
  );
  updateMotorController(
      motor2,
      ctrlDt,
      intDt,
      longGap
  );

  printTelemetry();
}