// #include <Arduino.h>
// #include <math.h>
// #include <ctype.h>

// #include <Wire.h>
// #include <Encoder.h>

// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>

// // =========================
// // IMU / JOINT CALCULATION
// // =========================
// // Motor-control backbone is preserved, but the joint measurement now uses
// // the same relative-IMU method as the angle-difference test file:
// //   1. read each IMU quaternion through the TCA9548A multiplexer
// //   2. zero each IMU individually at the reference pose
// //   3. compute joint quaternion from consecutive zeroed IMUs
// //      rel = q_moving_zeroed * conjugate(q_base_zeroed)
// //   4. extract a signed angle about the selected hinge axis
// //
// // This makes the commanded angle depend on the relative joint rotation between
// // adjacent IMUs instead of the absolute orientation of either IMU in world axes.

// #define TCA_ADDR 0x70
// #define MAX_IMUS 8
// #define NUM_IMUS 3

// // Which consecutive pair controls the motor target.
// // 1 = Joint 1-0, 2 = Joint 2-1, etc.
// // For shoulder / bicep / forearm placement, 2 usually means the elbow-like
// // joint between IMU 1 and IMU 2.
// #define CONTROL_JOINT_INDEX 2

// static_assert(NUM_IMUS <= MAX_IMUS, "NUM_IMUS cannot be greater than MAX_IMUS.");
// static_assert(MAX_IMUS <= 8, "One TCA9548A supports only 8 channels.");
// static_assert(CONTROL_JOINT_INDEX >= 1, "CONTROL_JOINT_INDEX must be at least 1.");
// static_assert(CONTROL_JOINT_INDEX < NUM_IMUS, "CONTROL_JOINT_INDEX must be less than NUM_IMUS.");

// Adafruit_BNO055 bnos[MAX_IMUS] = {
//   Adafruit_BNO055(0, 0x28),
//   Adafruit_BNO055(1, 0x28),
//   Adafruit_BNO055(2, 0x28),
//   Adafruit_BNO055(3, 0x28),
//   Adafruit_BNO055(4, 0x28),
//   Adafruit_BNO055(5, 0x28),
//   Adafruit_BNO055(6, 0x28),
//   Adafruit_BNO055(7, 0x28)
// };

// bool imu_ok[NUM_IMUS] = {false};
// bool have_event[NUM_IMUS] = {false};
// bool zeroed = false;

// imu::Quaternion current_q[NUM_IMUS];
// imu::Quaternion zero_q[NUM_IMUS];
// imu::Quaternion zeroed_q[NUM_IMUS];
// imu::Quaternion jointRelQ[NUM_IMUS];

// float jointRawDeg[NUM_IMUS] = {0.0};
// float jointAngleDeg[NUM_IMUS] = {0.0};

// // Hinge-axis selection for the 1-DOF joint.
// // Choose this from the Serial Monitor using x, y, or z.
// enum HingeAxisId {
//   HINGE_AXIS_X = 0,
//   HINGE_AXIS_Y = 1,
//   HINGE_AXIS_Z = 2
// };

// struct HingeAxis {
//   const char* name;
//   char key;
//   int sign;
//   float x;
//   float y;
//   float z;
// };

// // If the selected axis reads backwards, flip that axis sign here.
// HingeAxis hingeAxes[3] = {
//   {"X", 'x',  1, 1.0, 0.0, 0.0},
//   {"Y", 'y',  1, 0.0, 1.0, 0.0},
//   {"Z", 'z',  1, 0.0, 0.0, 1.0}
// };

// HingeAxisId hingeAxis = HINGE_AXIS_X;

// // =========================
// // MOTOR PINS
// // =========================

// // Motor 1
// #define M1_IN1 4
// #define M1_IN2 5
// #define M1_ENC_A 30
// #define M1_ENC_B 31

// // Motor 2
// #define M2_IN1 2
// #define M2_IN2 3
// #define M2_ENC_A 28
// #define M2_ENC_B 29

// Encoder enc1(M1_ENC_A, M1_ENC_B);
// Encoder enc2(M2_ENC_A, M2_ENC_B);

// // =========================
// // SETTINGS
// // =========================

// const int FWD = 1;
// const int REV = -1;

// const int M1_ENC_SIGN = 1;
// const int M2_ENC_SIGN = 1;

// const int M1_MOT_SIGN = 1;
// const int M2_MOT_SIGN = 1;

// const int M1_ROLL_SIGN = 1;
// const int M2_ROLL_SIGN = -1;

// const int MIN_PWM = 125;
// const int MAX_PWM = 255;
// const int CMD_PWM = 220;

// const unsigned long CTRL_US = 10000;   // 10 ms
// const unsigned long PRINT_MS = 250;

// const float KEY_TARGETS[10] = {
//   0.0, 10.0, 20.0, 30.0, 40.0,
//   50.0, 60.0, 70.0, 80.0, 90.0
// };

// // =========================
// // MOTOR STRUCT
// // =========================

// struct Motor {
//   const char* name;

//   int in1;
//   int in2;
//   Encoder* enc;

//   int encSign;
//   int motSign;
//   int rollSign;

//   float kp;
//   float kd;
//   float ki;

//   int minPwm;
//   int maxPwm;
//   int slowPwm;

//   float tol;
//   float slowZone;
//   unsigned long timeout;

//   float target;
//   float lastErr;
//   float sumErr;

//   bool active;
//   bool holding;
//   bool printed;

//   unsigned long startMs;
// };

// Motor m1 = {
//   "Motor 1",
//   M1_IN1, M1_IN2, &enc1,
//   M1_ENC_SIGN, M1_MOT_SIGN, M1_ROLL_SIGN,
//   1.75, 0.0, 0.125,
//   150, 225, 150,
//   1.0, 5.0, 12000,
//   0.0, 0.0, 0.0,
//   false, false, false,
//   0
// };

// Motor m2 = {
//   "Motor 2",
//   M2_IN1, M2_IN2, &enc2,
//   M2_ENC_SIGN, M2_MOT_SIGN, M2_ROLL_SIGN,
//   1.75, 0.0, 0.125,
//   150, 225, 150,
//   1.0, 5.0, 12000,
//   0.0, 0.0, 0.0,
//   false, false, false,
//   0
// };

// unsigned long lastCtrlUs = 0;
// unsigned long lastPrintMs = 0;

// // =========================
// // FUNCTION DECLARATIONS
// // =========================

// void imuStart();
// void imuRead();
// void imuZero();
// bool imuReady();

// void tcaselect(uint8_t channel);
// imu::Quaternion readQuaternionFromIMU(int imu_index);
// imu::Quaternion normalizeQ(imu::Quaternion q);
// imu::Quaternion applyZeroReference(const imu::Quaternion& current, const imu::Quaternion& zero_ref);
// imu::Quaternion computeRelativeOrientation(const imu::Quaternion& q_base, const imu::Quaternion& q_moving);
// float signedHingeAngleDeg(const imu::Quaternion& q, float fallbackDeg);

// void hingeAxisMenu();
// void pickHingeAxis();
// bool setHingeAxis(char c);
// const char* hingeAxisName();

// float angDiff(float nowAng, float zeroAng);
// float errDeg(float target, float current);
// float axisVal();
// float axisRaw();
// const char* axisName();

// void waitCal();
// void menu();

// void updateAll();
// void pid(Motor& m, float dt, bool enabled);
// bool motorEnabled(Motor& m);

// void setTarget(Motor& m, float targetDeg, int pwm);
// void resetMotor(Motor& m);
// void resetTargets();

// long counts(Motor& m);

// void drive(Motor& m, int dir, int pwm);
// void off(Motor& m);
// void hold(Motor& m);

// void serialCheck();
// void handleCmd(char c);
// void printData();

// // =========================
// // SETUP / LOOP
// // =========================

// void setup() {
//   Serial.begin(9600);
//   delay(2000);

//   pinMode(M1_IN1, OUTPUT);
//   pinMode(M1_IN2, OUTPUT);
//   pinMode(M2_IN1, OUTPUT);
//   pinMode(M2_IN2, OUTPUT);

//   off(m1);
//   off(m2);

//   enc1.write(0);
//   enc2.write(0);

//   imuStart();

//   Serial.println();
//   Serial.println("System ready.");

//   pickHingeAxis();

//   Serial.println();
//   Serial.println("Place the mechanism at the zero position.");
//   Serial.print("Selected measurement: ");
//   Serial.println(axisName());
//   Serial.print("Selected hinge axis: ");
//   Serial.println(hingeAxisName());
//   Serial.println("Type r and press Enter to recalibrate the current joint position to 0.");
//   Serial.println();

//   waitCal();

//   imuRead();

//   m1.target = axisVal();
//   m2.target = axisVal();

//   menu();

//   lastCtrlUs = micros();
// }

// void loop() {
//   updateAll();
// }

// // =========================
// // IMU FUNCTIONS
// // =========================

// void tcaselect(uint8_t channel) {
//   if (channel > 7) return;

//   Wire.beginTransmission(TCA_ADDR);
//   Wire.write(1 << channel);
//   Wire.endTransmission();
// }

// void imuStart() {
//   Serial.println("Starting BNO055 IMUs through TCA9548A multiplexer...");

//   Wire.begin();
//   Wire.setClock(400000);

//   for (int i = 0; i < NUM_IMUS; i++) {
//     tcaselect(i);
//     delay(100);

//     imu_ok[i] = bnos[i].begin();

//     if (imu_ok[i]) {
//       Serial.print("IMU ");
//       Serial.print(i);
//       Serial.println(" detected.");
//       delay(500);
//       bnos[i].setExtCrystalUse(true);
//     } else {
//       Serial.print("IMU ");
//       Serial.print(i);
//       Serial.println(" NOT detected.");
//     }
//   }
// }

// bool imuReady() {
//   return imu_ok[CONTROL_JOINT_INDEX] && imu_ok[CONTROL_JOINT_INDEX - 1];
// }

// imu::Quaternion readQuaternionFromIMU(int imu_index) {
//   tcaselect(imu_index);
//   return bnos[imu_index].getQuat();
// }

// imu::Quaternion normalizeQ(imu::Quaternion q) {
//   float mag = sqrt(
//     q.w()*q.w() +
//     q.x()*q.x() +
//     q.y()*q.y() +
//     q.z()*q.z()
//   );

//   if (isnan(mag) || mag < 0.000001) {
//     return imu::Quaternion(1.0, 0.0, 0.0, 0.0);
//   }

//   q.normalize();
//   return q;
// }

// imu::Quaternion applyZeroReference(const imu::Quaternion& current, const imu::Quaternion& zero_ref) {
//   // Rotation from this IMU's zero pose to its current pose.
//   imu::Quaternion zeroInv = zero_ref.conjugate();
//   return normalizeQ(zeroInv * current);
// }

// imu::Quaternion computeRelativeOrientation(const imu::Quaternion& q_base, const imu::Quaternion& q_moving) {
//   // Relative rotation of the moving/distal IMU with respect to the base/proximal IMU.
//   // This matches the angle-difference file's method.
//   imu::Quaternion baseInv = q_base.conjugate();
//   return normalizeQ(q_moving * baseInv);
// }

// float signedHingeAngleDeg(const imu::Quaternion& q, float fallbackDeg) {
//   float w = q.w();
//   float x = q.x();
//   float y = q.y();
//   float z = q.z();
//   const HingeAxis& h = hingeAxes[hingeAxis];

//   if (isnan(w) || isnan(x) || isnan(y) || isnan(z)) {
//     return fallbackDeg;
//   }

//   float axisComponent = x * h.x + y * h.y + z * h.z;

//   if (isnan(axisComponent)) {
//     return fallbackDeg;
//   }

//   float angleRad = 2.0 * atan2(axisComponent, w);

//   if (isnan(angleRad)) {
//     return fallbackDeg;
//   }

//   float angleDeg = angleRad * 180.0 / PI;

//   while (angleDeg > 180.0) angleDeg -= 360.0;
//   while (angleDeg < -180.0) angleDeg += 360.0;

//   return h.sign * angleDeg;
// }

// void imuRead() {
//   for (int i = 0; i < NUM_IMUS; i++) {
//     have_event[i] = false;

//     if (imu_ok[i]) {
//       current_q[i] = normalizeQ(readQuaternionFromIMU(i));
//       have_event[i] = true;

//       if (zeroed) {
//         zeroed_q[i] = applyZeroReference(current_q[i], zero_q[i]);
//       } else {
//         zeroed_q[i] = current_q[i];
//       }
//     }
//   }

//   for (int i = 1; i < NUM_IMUS; i++) {
//     if (have_event[i] && have_event[i - 1]) {
//       jointRelQ[i] = computeRelativeOrientation(zeroed_q[i - 1], zeroed_q[i]);
//       jointRawDeg[i] = signedHingeAngleDeg(jointRelQ[i], jointRawDeg[i]);
//       jointAngleDeg[i] = jointRawDeg[i];
//     }
//   }
// }

// void imuZero() {
//   bool anyZeroed = false;

//   for (int i = 0; i < NUM_IMUS; i++) {
//     if (imu_ok[i]) {
//       zero_q[i] = normalizeQ(readQuaternionFromIMU(i));
//       zeroed_q[i] = imu::Quaternion(1.0, 0.0, 0.0, 0.0);
//       anyZeroed = true;
//     }
//   }

//   if (!anyZeroed) {
//     Serial.println("Cannot recalibrate: no IMUs are ready.");
//     return;
//   }

//   zeroed = true;

//   for (int i = 1; i < NUM_IMUS; i++) {
//     jointRawDeg[i] = 0.0;
//     jointAngleDeg[i] = 0.0;
//   }

//   resetTargets();

//   Serial.println("IMUs recalibrated. Current relative joint positions are now 0.");
// }

// float angDiff(float nowAng, float zeroAng) {
//   float diff = nowAng - zeroAng;

//   while (diff > 180.0) diff -= 360.0;
//   while (diff < -180.0) diff += 360.0;

//   return diff;
// }

// float errDeg(float target, float current) {
//   return angDiff(target, current);
// }

// float axisVal() {
//   return jointAngleDeg[CONTROL_JOINT_INDEX];
// }

// float axisRaw() {
//   return jointRawDeg[CONTROL_JOINT_INDEX];
// }

// const char* axisName() {
//   return "relative quaternion joint angle";
// }

// void hingeAxisMenu() {
//   Serial.println("Choose which quaternion hinge axis to use:");
//   Serial.println("x = hinge rotation about relative quaternion X axis");
//   Serial.println("y = hinge rotation about relative quaternion Y axis");
//   Serial.println("z = hinge rotation about relative quaternion Z axis");
//   Serial.println();
// }

// void pickHingeAxis() {
//   hingeAxisMenu();

//   while (true) {
//     if (Serial.available() > 0) {
//       char c = Serial.read();

//       if (c == '\n' || c == '\r') continue;

//       if (setHingeAxis(c)) {
//         Serial.println();
//         Serial.print("Hinge axis selected: ");
//         Serial.println(hingeAxisName());
//         Serial.println();
//         return;
//       }

//       Serial.println("Please choose x, y, or z first.");
//     }
//   }
// }

// bool setHingeAxis(char c) {
//   c = tolower(c);

//   for (int i = 0; i < 3; i++) {
//     if (c == hingeAxes[i].key) {
//       hingeAxis = (HingeAxisId)i;
//       return true;
//     }
//   }

//   return false;
// }

// const char* hingeAxisName() {
//   return hingeAxes[hingeAxis].name;
// }

// // =========================
// // MENU FUNCTIONS
// // =========================

// void waitCal() {
//   while (true) {
//     if (Serial.available() > 0) {
//       char c = Serial.read();

//       if (c == '\n' || c == '\r') continue;

//       if (c == 'x' || c == 'X' ||
//           c == 'y' || c == 'Y' ||
//           c == 'z' || c == 'Z') {
//         if (setHingeAxis(c)) {
//           Serial.println();
//           Serial.print("Hinge axis switched to ");
//           Serial.print(hingeAxisName());
//           Serial.println(".");
//           Serial.println("Now type r and press Enter to recalibrate with this hinge axis.");
//           Serial.println();
//         }
//         continue;
//       }

//       if (c == 'r' || c == 'R') {
//         imuZero();
//         Serial.println("Calibration complete.");
//         Serial.println();
//         return;
//       }

//       Serial.println("Please type r to recalibrate, or x/y/z to switch hinge axis first.");
//     }
//   }
// }

// void menu() {
//   Serial.println("Serial control mode is ON.");
//   Serial.print("Active measurement: ");
//   Serial.println(axisName());
//   Serial.print("Controlled joint pair: IMU ");
//   Serial.print(CONTROL_JOINT_INDEX);
//   Serial.print(" relative to IMU ");
//   Serial.println(CONTROL_JOINT_INDEX - 1);
//   Serial.print("Selected hinge axis: ");
//   Serial.println(hingeAxisName());
//   Serial.println("Type a number and press Enter to move Motor 2 to that joint angle:");
//   Serial.println("Motor 2 controls the selected relative quaternion-derived joint angle.");
//   Serial.println("Motor 1 is kept stopped in this version.");
//   Serial.println("0 = 0 deg");
//   Serial.println("1 = 10 deg");
//   Serial.println("2 = 20 deg");
//   Serial.println("3 = 30 deg");
//   Serial.println("4 = 40 deg");
//   Serial.println("5 = 50 deg");
//   Serial.println("6 = 60 deg");
//   Serial.println("7 = 70 deg");
//   Serial.println("8 = 80 deg");
//   Serial.println("9 = 90 deg");
//   Serial.println("x/y/z = switch quaternion hinge axis");
//   Serial.println("s = stop both motors");
//   Serial.println("r = recalibrate joint angle to 0");
//   Serial.println("m = print menu");
//   Serial.println();
// }

// // =========================
// // PID CONTROL
// // =========================

// void updateAll() {
//   serialCheck();

//   unsigned long now = micros();

//   if (now - lastCtrlUs < CTRL_US) {
//     return;
//   }

//   float dt = (now - lastCtrlUs) / 1000000.0;
//   lastCtrlUs = now;

//   if (dt <= 0) dt = 0.001;

//   imuRead();

//   pid(m1, dt, motorEnabled(m1));
//   pid(m2, dt, motorEnabled(m2));

//   if (m1.active || m2.active) {
//     printData();
//   }
// }

// bool motorEnabled(Motor& m) {
//   if (&m == &m1) {
//     return false;
//   }

//   return true;
// }

// void pid(Motor& m, float dt, bool enabled) {
//   if (!imuReady() || !enabled) {
//     off(m);
//     resetMotor(m);
//     return;
//   }

//   if (!m.active) {
//     off(m);
//     return;
//   }

//   float error = errDeg(m.target, axisVal());
//   float absErr = fabs(error);

//   if (absErr <= m.tol) {
//     hold(m);

//     m.sumErr = 0.0;
//     m.lastErr = error;
//     m.holding = true;
//     m.startMs = millis();

//     if (!m.printed) {
//       Serial.println();
//       Serial.print(m.name);
//       Serial.println(" reached target. PID hold is active.");

//       Serial.print("Measurement: ");
//       Serial.print(axisName());
//       Serial.print(" | JointPair: ");
//       Serial.print(CONTROL_JOINT_INDEX);
//       Serial.print("-");
//       Serial.print(CONTROL_JOINT_INDEX - 1);
//       Serial.print(" | TargetDeg: ");
//       Serial.print(m.target, 2);
//       Serial.print(" | CurrentDeg: ");
//       Serial.print(axisVal(), 2);
//       Serial.print(" | ErrorDeg: ");
//       Serial.print(error, 2);
//       Serial.print(" | M1Counts: ");
//       Serial.print(counts(m1));
//       Serial.print(" | M2Counts: ");
//       Serial.println(counts(m2));

//       Serial.println("If the mechanism is pushed away, PID will correct it.");
//       Serial.println();

//       m.printed = true;
//     }

//     return;
//   }

//   if (m.holding) {
//     m.holding = false;
//     m.printed = false;
//     m.sumErr = 0.0;
//     m.lastErr = error;
//     m.startMs = millis();

//     Serial.println();
//     Serial.print(m.name);
//     Serial.println(" moved away from target. PID is correcting.");
//     Serial.println();
//   }

//   if (millis() - m.startMs > m.timeout) {
//     off(m);
//     resetMotor(m);

//     Serial.println();
//     Serial.print(m.name);
//     Serial.println(" move timed out. Motor is fully OFF.");

//     Serial.print("Measurement: ");
//     Serial.print(axisName());
//     Serial.print(" | HingeAxis: ");
//     Serial.print(hingeAxisName());
//     Serial.print(" | TargetDeg: ");
//     Serial.print(m.target, 2);
//     Serial.print(" | CurrentDeg: ");
//     Serial.print(axisVal(), 2);
//     Serial.print(" | ErrorDeg: ");
//     Serial.println(errDeg(m.target, axisVal()), 2);

//     Serial.println("Choose another target or recalibrate with r.");
//     Serial.println();

//     return;
//   }

//   float dErr = (error - m.lastErr) / dt;

//   m.sumErr += error * dt;
//   m.sumErr = constrain(m.sumErr, -100.0, 100.0);

//   float out = m.kp * error
//             + m.kd * dErr
//             + m.ki * m.sumErr;

//   int pwm = abs((int)out);
//   int pwmLimit = m.maxPwm;

//   if (absErr <= m.slowZone) {
//     pwmLimit = m.slowPwm;
//   }

//   pwm = constrain(pwm, 0, pwmLimit);

//   if (pwm > 0 && pwm < m.minPwm) {
//     pwm = m.minPwm;
//   }

//   int dir = FWD;

//   if (out < 0) {
//     dir = REV;
//   }

//   dir *= m.motSign;

//   drive(m, dir, pwm);

//   m.lastErr = error;
// }

// // =========================
// // MOTOR STATE
// // =========================

// void setTarget(Motor& m, float targetDeg, int pwm) {
//   imuRead();

//   m.target = targetDeg;
//   m.maxPwm = constrain(pwm, MIN_PWM, MAX_PWM);

//   m.sumErr = 0.0;
//   m.lastErr = errDeg(m.target, axisVal());

//   m.active = true;
//   m.holding = false;
//   m.printed = false;
//   m.startMs = millis();

//   Serial.println();
//   Serial.print("Moving ");
//   Serial.print(m.name);
//   Serial.print(" to ");
//   Serial.print(m.target, 2);
//   Serial.print(" degrees using ");
//   Serial.print(axisName());
//   Serial.print(" for joint pair ");
//   Serial.print(CONTROL_JOINT_INDEX);
//   Serial.print("-");
//   Serial.print(CONTROL_JOINT_INDEX - 1);
//   Serial.print(" around hinge axis ");
//   Serial.print(hingeAxisName());
//   Serial.println(".");
//   Serial.println();
// }

// void resetMotor(Motor& m) {
//   m.active = false;
//   m.holding = false;
//   m.printed = false;
//   m.sumErr = 0.0;
//   m.lastErr = 0.0;
// }

// void resetTargets() {
//   imuRead();

//   m1.target = axisVal();
//   m2.target = axisVal();

//   resetMotor(m1);
//   resetMotor(m2);

//   off(m1);
//   off(m2);
// }

// long counts(Motor& m) {
//   return m.enc->read() * m.encSign;
// }

// // =========================
// // MOTOR DRIVER
// // =========================

// void drive(Motor& m, int dir, int pwm) {
//   pwm = constrain(pwm, 0, 255);

//   if (dir == FWD) {
//     analogWrite(m.in1, pwm);
//     analogWrite(m.in2, 0);
//   } else if (dir == REV) {
//     analogWrite(m.in1, 0);
//     analogWrite(m.in2, pwm);
//   } else {
//     off(m);
//   }
// }

// void off(Motor& m) {
//   analogWrite(m.in1, 0);
//   analogWrite(m.in2, 0);

//   digitalWrite(m.in1, LOW);
//   digitalWrite(m.in2, LOW);
// }

// void hold(Motor& m) {
//   analogWrite(m.in1, 255);
//   analogWrite(m.in2, 255);
// }

// // =========================
// // SERIAL INPUT / OUTPUT
// // =========================

// void serialCheck() {
//   while (Serial.available() > 0) {
//     char c = Serial.read();

//     if (c == '\n' || c == '\r') continue;

//     handleCmd(c);
//   }
// }

// void handleCmd(char c) {
//   if (c >= '0' && c <= '9') {
//     int key = c - '0';
//     float target = KEY_TARGETS[key];

//     imuRead();

//     off(m1);
//     resetMotor(m1);
//     m1.target = axisVal();

//     setTarget(m2, target, CMD_PWM);

//     Serial.println("Joint-angle mode active: Motor 2 is controlling the selected relative quaternion joint angle. Motor 1 is stopped.");
//     Serial.println();
//     return;
//   }

//   if (c == 'x' || c == 'X' ||
//       c == 'y' || c == 'Y' ||
//       c == 'z' || c == 'Z') {
//     if (setHingeAxis(c)) {
//       resetTargets();

//       Serial.println();
//       Serial.print("Hinge axis switched to ");
//       Serial.print(hingeAxisName());
//       Serial.println(".");
//       Serial.println("Targets were reset to the current relative quaternion joint angle.");
//       Serial.println("Recalibrate with r if this axis change should define a new zero pose.");
//       Serial.println();
//     }

//     return;
//   }

//   if (c == 'r' || c == 'R') {
//     imuZero();

//     Serial.println("Target reset to current joint angle after IMU zero.");
//     Serial.println("Choose a number target when ready.");
//     Serial.println();
//     return;
//   }

//   if (c == 's' || c == 'S') {
//     resetTargets();

//     Serial.println();
//     Serial.print("Motor 1 and Motor 2 stopped. Current ");
//     Serial.print(axisName());
//     Serial.print(": ");
//     Serial.println(axisVal(), 2);
//     Serial.println("Choose another target.");
//     Serial.println();
//     return;
//   }

//   if (c == 'm' || c == 'M') {
//     menu();
//     return;
//   }

//   Serial.print("Unknown command: ");
//   Serial.println(c);
// }

// void printData() {
//   if (millis() - lastPrintMs >= PRINT_MS) {
//     lastPrintMs = millis();

//     float targetDeg = m2.target;
//     float currentDeg = axisVal();
//     float errorDeg = errDeg(targetDeg, currentDeg);

//     // Teleplot output
//     Serial.print(">targetDeg:");
//     Serial.println(targetDeg, 2);

//     Serial.print(">currentDeg:");
//     Serial.println(currentDeg, 2);

//     Serial.print(">errorDeg:");
//     Serial.println(errorDeg, 2);

//     // Optional joint debug output
//     for (int i = 1; i < NUM_IMUS; i++) {
//       Serial.print(">joint");
//       Serial.print(i);
//       Serial.print("minus");
//       Serial.print(i - 1);
//       Serial.print("Deg:");
//       Serial.println(jointAngleDeg[i], 2);
//     }

//     // Readable output
//     Serial.print("Measurement: ");
//     Serial.print(axisName());

//     Serial.print(" | JointPair: ");
//     Serial.print(CONTROL_JOINT_INDEX);
//     Serial.print("-");
//     Serial.print(CONTROL_JOINT_INDEX - 1);

//     Serial.print(" | HingeAxis: ");
//     Serial.print(hingeAxisName());

//     Serial.print(" | TargetDeg: ");
//     Serial.print(targetDeg, 2);

//     Serial.print(" | CurrentDeg: ");
//     Serial.print(currentDeg, 2);

//     Serial.print(" | ErrorDeg: ");
//     Serial.print(errorDeg, 2);

//     Serial.print(" | M1Counts: ");
//     Serial.print(counts(m1));

//     Serial.print(" | M2Counts: ");
//     Serial.println(counts(m2));
//   }
// }
