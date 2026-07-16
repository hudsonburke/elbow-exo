// #include <Arduino.h>
// #include <math.h>
// #include <ctype.h>
// #include <Wire.h>
// #include <Encoder.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>

// // ======================================================
// // Km Identification Test Program
// //
// // Goal:
// //   Estimate motor-cable gain Km using encoder counts.
// //
// // Model:
// //   l_dot(t) = Km * u(t)
// //
// // Test:
// //   Press n to start one fixed-time constant-PWM trial.
// //   Motor 2 runs at constant PWM for TEST_TIME_MS.
// //   Encoder counts are used to calculate cable displacement,
// //   cable speed, Km, and deadband-subtracted Km.
// //
// // Fast data line for Python:
// //   DATA,time_ms,theta_deg,m1_counts,m2_counts,target_pwm,pwm,u_cmd,mode,trial_id
// //
// // Result line:
// //   RESULT,trial_id,pwm,u_cmd,test_time_s,c0,ct,delta_c,delta_l_m,cable_speed_mps,km_raw,km_deadband,early_stop
// // ======================================================

// // ---------------------
// // IMU setup
// // ---------------------

// Adafruit_BNO055 bnoUpper(0, 0x28, &Wire);
// Adafruit_BNO055 bnoForearm(1, 0x28, &Wire1);

// struct Quat {
//   float w;
//   float x;
//   float y;
//   float z;
// };

// Quat qUpRaw = {1.0, 0.0, 0.0, 0.0};
// Quat qForeRaw = {1.0, 0.0, 0.0, 0.0};

// Quat qUpZero = {1.0, 0.0, 0.0, 0.0};
// Quat qForeZero = {1.0, 0.0, 0.0, 0.0};

// Quat qUpZeroed = {1.0, 0.0, 0.0, 0.0};
// Quat qForeZeroed = {1.0, 0.0, 0.0, 0.0};
// Quat qJointZeroed = {1.0, 0.0, 0.0, 0.0};

// float ang = 0.0;
// float rawAng = 0.0;

// bool imuOk = false;

// // ---------------------
// // IMU filter
// // ---------------------

// float filtAng = 0.0;
// float avgAng = 0.0;
// float varAng = 0.0;
// float goodAng = 0.0;

// bool filtOn = false;
// unsigned long rejSpks = 0;

// const float FILT_A = 0.25;
// const float AVG_A = 0.10;
// const float VAR_A = 0.10;

// const float JMP_MAX = 25.0;
// const float VAR_MAX = 100.0;

// // ---------------------
// // Motor pins
// // ---------------------

// const int M1_IN1 = 4;
// const int M1_IN2 = 5;
// const int M1_ENC_A = 30;
// const int M1_ENC_B = 31;

// const int M2_IN1 = 2;
// const int M2_IN2 = 3;
// const int M2_ENC_A = 28;
// const int M2_ENC_B = 29;

// Encoder enc1(M1_ENC_A, M1_ENC_B);
// Encoder enc2(M2_ENC_A, M2_ENC_B);

// // ---------------------
// // Direction and encoder signs
// // ---------------------

// const int FWD = 1;
// const int REV = -1;

// const int M1_ENC_SIGN = 1;
// const int M2_ENC_SIGN = 1;

// const int M2_MOT_SIGN = 1;

// // Change this if the Km test moves in the wrong direction.
// const int TEST_DIR = FWD;

// // ---------------------
// // Km identification constants
// // ---------------------

// // Your spool inner diameter was 23.7 mm, so radius = 11.85 mm = 0.01185 m.
// const float R_SPOOL_M = 0.01185;

// // CQRobot 270:1 motor with 64 CPR encoder:
// // 64 * 270 = 17280 counts per output shaft revolution.
// const float COUNTS_PER_REV = 17280.0;

// // Deadband value from worksheet method.
// const int U_DEAD = 150;

// // Fixed test time for each Km trial.
// const unsigned long TEST_TIME_MS = 2000;

// // Wait after reset before motion begins.
// const unsigned long START_DELAY_MS = 1000;

// // Safety angle limit. The test stops early if this is exceeded.
// const float SAFETY_ANGLE_DEG = 95.0;

// // PWM values from the worksheet.
// const int PWM_VALUES[] = {150, 170, 190, 210, 230, 250};
// const int PWM_VALUE_COUNT = sizeof(PWM_VALUES) / sizeof(PWM_VALUES[0]);

// int pwmIndex = 0;
// int TEST_PWM = PWM_VALUES[0];

// // Manual emergency control.
// const int MAN_PWM = 125;
// bool manMode = false;
// int manDir = 0;

// // ---------------------
// // Timing
// // ---------------------

// const unsigned long CTRL_US = 10000;        // 10 ms = about 100 Hz
// const unsigned long DATA_MS = 10;           // 10 ms = about 100 Hz data
// const unsigned long DEBUG_MS = 100;         // readable status
// const unsigned long QUAT_MS = 100;          // quaternion visualizer update

// unsigned long lastCtrlUs = 0;
// unsigned long lastDataMs = 0;
// unsigned long lastDebugMs = 0;
// unsigned long lastQuatMs = 0;

// // ---------------------
// // Trial state
// // ---------------------

// enum TestState {
//   TEST_IDLE,
//   TEST_WAITING,
//   TEST_RUNNING,
//   TEST_DONE
// };

// TestState testState = TEST_IDLE;

// unsigned int trialId = 0;

// unsigned long trialStartMs = 0;
// unsigned long motionStartMs = 0;
// unsigned long motionEndMs = 0;

// long trialC0 = 0;
// long trialCT = 0;

// int trialPwm = 0;
// int trialUCmd = 0;

// float lastUNorm = 0.0;
// int lastPwm = 0;
// float lastU = 0.0;

// int escState = 0;

// // ======================================================
// // Function declarations
// // ======================================================

// Quat normQ(Quat q);
// Quat conjQ(Quat q);
// Quat mulQ(Quat a, Quat b);
// Quat fromBno(imu::Quaternion q);
// float angleQ(Quat q);

// float filtJoint(float raw);
// void resetFilt(float start);

// bool imuStart();
// bool imuReady();
// void imuRead();
// void imuZero();

// float axisVal();
// long countsM1();
// long countsM2();

// void driveM2(int dir, int pwm);
// void offM1();
// void offM2();
// void offAll();

// const char* stateName();
// void resetTrialStateOnly();
// void startKmTrial();
// void beginMotion();
// void finishKmTrial(bool earlyStop);
// void updateKmTrial();
// void emergencyStop(const char* reason);

// void serialCheck();
// void handleChar(char c);
// void handleArrow(char c);

// void printData();
// void printFastData();
// void printDebug();
// void printQuat(const char* label, Quat q);
// void printMenu();

// void printFloatOrNan(float value, int decimals);

// // ======================================================
// // Quaternion math
// // ======================================================

// Quat normQ(Quat q) {
//   float n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

//   if (n < 0.000001) {
//     return {1.0, 0.0, 0.0, 0.0};
//   }

//   q.w /= n;
//   q.x /= n;
//   q.y /= n;
//   q.z /= n;

//   return q;
// }

// Quat conjQ(Quat q) {
//   q = normQ(q);
//   return {q.w, -q.x, -q.y, -q.z};
// }

// Quat mulQ(Quat a, Quat b) {
//   Quat q;

//   q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
//   q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
//   q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
//   q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

//   return normQ(q);
// }

// Quat fromBno(imu::Quaternion q) {
//   Quat out = {
//     (float)q.w(),
//     (float)q.x(),
//     (float)q.y(),
//     (float)q.z()
//   };

//   return normQ(out);
// }

// float angleQ(Quat q) {
//   q = normQ(q);

//   float w = fabs(q.w);
//   w = constrain(w, -1.0, 1.0);

//   return 2.0 * acos(w) * 180.0 / PI;
// }

// // ======================================================
// // IMU filter
// // ======================================================

// float filtJoint(float raw) {
//   if (!filtOn) {
//     resetFilt(raw);
//     return raw;
//   }

//   float jump = fabs(raw - filtAng);
//   float dAvg = raw - avgAng;
//   float instVar = dAvg * dAvg;

//   float vLim = varAng * 3.0;

//   if (vLim < VAR_MAX) {
//     vLim = VAR_MAX;
//   }

//   bool spike = (jump > JMP_MAX) && (instVar > vLim);

//   if (spike) {
//     rejSpks++;
//     filtAng = goodAng;
//     return filtAng;
//   }

//   avgAng = AVG_A * raw + (1.0 - AVG_A) * avgAng;

//   float dNew = raw - avgAng;
//   float newVar = dNew * dNew;

//   varAng = VAR_A * newVar + (1.0 - VAR_A) * varAng;

//   filtAng = FILT_A * raw + (1.0 - FILT_A) * filtAng;
//   goodAng = filtAng;

//   return filtAng;
// }

// void resetFilt(float start) {
//   rawAng = start;
//   filtAng = start;
//   avgAng = start;
//   varAng = 0.0;
//   goodAng = start;
//   filtOn = true;
//   rejSpks = 0;
// }

// // ======================================================
// // IMU functions
// // ======================================================

// bool imuStart() {
//   Wire.begin();
//   Wire1.begin();

//   delay(100);

//   bool upOk = bnoUpper.begin();
//   bool foreOk = bnoForearm.begin();

//   if (!upOk) {
//     Serial.println("ERROR: Upper BNO055 not detected.");
//   }

//   if (!foreOk) {
//     Serial.println("ERROR: Forearm BNO055 not detected.");
//   }

//   if (!upOk || !foreOk) {
//     imuOk = false;
//     return false;
//   }

//   delay(1000);

//   bnoUpper.setExtCrystalUse(true);
//   bnoForearm.setExtCrystalUse(true);

//   imuOk = true;

//   imuRead();
//   imuZero();

//   Serial.println("IMUs started and zeroed.");

//   return true;
// }

// bool imuReady() {
//   return imuOk;
// }

// void imuRead() {
//   if (!imuOk) {
//     return;
//   }

//   qUpRaw = fromBno(bnoUpper.getQuat());
//   qForeRaw = fromBno(bnoForearm.getQuat());

//   qUpZeroed = mulQ(conjQ(qUpZero), qUpRaw);
//   qForeZeroed = mulQ(conjQ(qForeZero), qForeRaw);

//   qJointZeroed = mulQ(conjQ(qUpZeroed), qForeZeroed);

//   rawAng = angleQ(qJointZeroed);
//   ang = filtJoint(rawAng);
// }

// void imuZero() {
//   if (!imuOk) {
//     Serial.println("Cannot zero IMUs. IMU not ready.");
//     return;
//   }

//   qUpRaw = fromBno(bnoUpper.getQuat());
//   qForeRaw = fromBno(bnoForearm.getQuat());

//   qUpZero = qUpRaw;
//   qForeZero = qForeRaw;

//   qUpZeroed = {1.0, 0.0, 0.0, 0.0};
//   qForeZeroed = {1.0, 0.0, 0.0, 0.0};
//   qJointZeroed = {1.0, 0.0, 0.0, 0.0};

//   rawAng = 0.0;
//   ang = 0.0;

//   resetFilt(0.0);

//   Serial.println("IMU recalibrated. Current joint angle is now 0.");
// }

// // ======================================================
// // Helpers
// // ======================================================

// float axisVal() {
//   return ang;
// }

// long countsM1() {
//   return M1_ENC_SIGN * enc1.read();
// }

// long countsM2() {
//   return M2_ENC_SIGN * enc2.read();
// }

// void driveM2(int dir, int pwm) {
//   pwm = constrain(pwm, 0, 255);

//   int actualDir = dir * M2_MOT_SIGN;

//   if (pwm <= 0 || actualDir == 0) {
//     offM2();
//     return;
//   }

//   if (actualDir > 0) {
//     analogWrite(M2_IN1, pwm);
//     analogWrite(M2_IN2, 0);
//   } else {
//     analogWrite(M2_IN1, 0);
//     analogWrite(M2_IN2, pwm);
//   }

//   lastPwm = pwm;
//   lastUNorm = (float)pwm / 255.0;
//   lastU = actualDir * pwm;
// }

// void offM1() {
//   analogWrite(M1_IN1, 0);
//   analogWrite(M1_IN2, 0);
// }

// void offM2() {
//   analogWrite(M2_IN1, 0);
//   analogWrite(M2_IN2, 0);

//   lastPwm = 0;
//   lastUNorm = 0.0;
//   lastU = 0.0;
// }

// void offAll() {
//   offM1();
//   offM2();
// }

// // ======================================================
// // Km trial logic
// // ======================================================

// const char* stateName() {
//   if (manMode) {
//     return "Manual";
//   }

//   if (testState == TEST_IDLE) {
//     return "Idle";
//   }

//   if (testState == TEST_WAITING) {
//     return "KmWait";
//   }

//   if (testState == TEST_RUNNING) {
//     return "KmRun";
//   }

//   if (testState == TEST_DONE) {
//     return "KmDone";
//   }

//   return "Unknown";
// }

// void resetTrialStateOnly() {
//   testState = TEST_IDLE;
//   trialStartMs = 0;
//   motionStartMs = 0;
//   motionEndMs = 0;
//   trialC0 = 0;
//   trialCT = 0;
//   trialPwm = 0;
//   trialUCmd = 0;
// }

// void startKmTrial() {
//   emergencyStop("new_trial_reset");

//   trialId++;

//   enc1.write(0);
//   enc2.write(0);
//   imuZero();
//   enc1.write(0);
//   enc2.write(0);

//   TEST_PWM = constrain(TEST_PWM, 0, 255);

//   trialPwm = TEST_PWM;
//   trialUCmd = TEST_DIR * M2_MOT_SIGN * trialPwm;

//   trialStartMs = millis();
//   motionStartMs = trialStartMs + START_DELAY_MS;
//   motionEndMs = 0;

//   trialC0 = countsM2();
//   trialCT = trialC0;

//   testState = TEST_WAITING;

//   Serial.print("EVENT,trial_start,");
//   Serial.print(trialId);
//   Serial.print(",pwm,");
//   Serial.print(trialPwm);
//   Serial.print(",u_cmd,");
//   Serial.print(trialUCmd);
//   Serial.print(",test_time_ms,");
//   Serial.print(TEST_TIME_MS);
//   Serial.print(",c0,");
//   Serial.println(trialC0);

//   Serial.println("Km trial started. Encoder and IMU reset. Waiting before motion...");
// }

// void beginMotion() {
//   motionStartMs = millis();
//   trialC0 = countsM2();
//   testState = TEST_RUNNING;

//   Serial.print("EVENT,motion_start,");
//   Serial.print(trialId);
//   Serial.print(",time_ms,0,c0,");
//   Serial.println(trialC0);
// }

// void finishKmTrial(bool earlyStop) {
//   offM2();

//   motionEndMs = millis();
//   testState = TEST_DONE;

//   trialCT = countsM2();

//   unsigned long elapsedMs = motionEndMs - motionStartMs;

//   if (elapsedMs == 0) {
//     elapsedMs = 1;
//   }

//   float T = elapsedMs / 1000.0;
//   long deltaC = trialCT - trialC0;

//   float deltaPhi = (2.0 * PI / COUNTS_PER_REV) * (float)deltaC;
//   float deltaL = R_SPOOL_M * deltaPhi;
//   float cableSpeed = deltaL / T;

//   float kmRaw = NAN;
//   float kmDead = NAN;

//   if (abs(trialUCmd) > 0) {
//     kmRaw = deltaL / (T * (float)trialUCmd);
//   }

//   float uEff = 0.0;

//   if (abs(trialUCmd) > U_DEAD) {
//     if (trialUCmd > 0) {
//       uEff = (float)(trialUCmd - U_DEAD);
//     } else {
//       uEff = (float)(trialUCmd + U_DEAD);
//     }
//   }

//   if (fabs(uEff) > 0.000001) {
//     kmDead = deltaL / (T * uEff);
//   }

//   Serial.print("EVENT,trial_end,");
//   Serial.print(trialId);
//   Serial.print(",elapsed_ms,");
//   Serial.print(elapsedMs);
//   Serial.print(",theta,");
//   Serial.print(axisVal(), 3);
//   Serial.print(",ct,");
//   Serial.print(trialCT);
//   Serial.print(",early_stop,");
//   Serial.println(earlyStop ? 1 : 0);

//   Serial.print("RESULT,");
//   Serial.print(trialId);
//   Serial.print(",");
//   Serial.print(trialPwm);
//   Serial.print(",");
//   Serial.print(trialUCmd);
//   Serial.print(",");
//   Serial.print(T, 4);
//   Serial.print(",");
//   Serial.print(trialC0);
//   Serial.print(",");
//   Serial.print(trialCT);
//   Serial.print(",");
//   Serial.print(deltaC);
//   Serial.print(",");
//   Serial.print(deltaL, 8);
//   Serial.print(",");
//   Serial.print(cableSpeed, 8);
//   Serial.print(",");
//   printFloatOrNan(kmRaw, 10);
//   Serial.print(",");
//   printFloatOrNan(kmDead, 10);
//   Serial.print(",");
//   Serial.println(earlyStop ? 1 : 0);

//   Serial.println("Km trial complete.");
// }

// void updateKmTrial() {
//   if (testState == TEST_IDLE || testState == TEST_DONE) {
//     return;
//   }

//   if (!imuReady()) {
//     emergencyStop("imu_not_ready");
//     return;
//   }

//   unsigned long now = millis();

//   if (testState == TEST_WAITING) {
//     offM2();

//     if (now >= motionStartMs) {
//       beginMotion();
//     }

//     return;
//   }

//   if (testState == TEST_RUNNING) {
//     driveM2(TEST_DIR, trialPwm);

//     unsigned long elapsedMs = now - motionStartMs;

//     if (axisVal() >= SAFETY_ANGLE_DEG) {
//       finishKmTrial(true);
//       return;
//     }

//     if (elapsedMs >= TEST_TIME_MS) {
//       finishKmTrial(false);
//       return;
//     }

//     return;
//   }
// }

// void emergencyStop(const char* reason) {
//   manMode = false;
//   manDir = 0;

//   offAll();

//   testState = TEST_IDLE;

//   Serial.print("EVENT,stop,");
//   Serial.print(trialId);
//   Serial.print(",");
//   Serial.print(reason);
//   Serial.print(",theta,");
//   Serial.print(axisVal(), 3);
//   Serial.print(",m2_counts,");
//   Serial.println(countsM2());
// }

// // ======================================================
// // Serial commands
// // ======================================================

// void serialCheck() {
//   while (Serial.available() > 0) {
//     char c = Serial.read();

//     if (escState == 0) {
//       if (c == 27) {
//         escState = 1;
//       } else {
//         handleChar(c);
//       }
//     } else if (escState == 1) {
//       escState = (c == '[') ? 2 : 0;
//     } else if (escState == 2) {
//       handleArrow(c);
//       escState = 0;
//     }
//   }
// }

// void handleArrow(char c) {
//   if (c == 'D') {
//     handleChar('a');
//   } else if (c == 'C') {
//     handleChar('d');
//   }
// }

// void handleChar(char c) {
//   if (c == '\n' || c == '\r') {
//     return;
//   }

//   if (c == 'n' || c == 'N') {
//     startKmTrial();
//     return;
//   }

//   if (c == 's' || c == 'S' || c == 'p' || c == 'P' || c == 'e' || c == 'E' || c == ' ') {
//     emergencyStop("user_command");
//     return;
//   }

//   if (c == 'a' || c == 'A') {
//     emergencyStop("manual_down");

//     manMode = true;
//     manDir = REV;

//     Serial.println("Manual mode: Motor 2 reverse/down");
//     return;
//   }

//   if (c == 'd' || c == 'D') {
//     emergencyStop("manual_up");

//     manMode = true;
//     manDir = FWD;

//     Serial.println("Manual mode: Motor 2 forward/up");
//     return;
//   }

//   if (c == 'r' || c == 'R') {
//     emergencyStop("reset");
//     enc1.write(0);
//     enc2.write(0);
//     imuZero();
//     enc1.write(0);
//     enc2.write(0);
//     Serial.println("Reset complete. IMU and encoders are zeroed.");
//     return;
//   }

//   if (c == '+') {
//     TEST_PWM += 5;
//     TEST_PWM = constrain(TEST_PWM, 0, 255);

//     Serial.print("TEST_PWM = ");
//     Serial.println(TEST_PWM);

//     Serial.print("EVENT,pwm_set,");
//     Serial.println(TEST_PWM);
//     return;
//   }

//   if (c == '-') {
//     TEST_PWM -= 5;
//     TEST_PWM = constrain(TEST_PWM, 0, 255);

//     Serial.print("TEST_PWM = ");
//     Serial.println(TEST_PWM);

//     Serial.print("EVENT,pwm_set,");
//     Serial.println(TEST_PWM);
//     return;
//   }

//   if (c == 'v' || c == 'V') {
//     pwmIndex++;

//     if (pwmIndex >= PWM_VALUE_COUNT) {
//       pwmIndex = 0;
//     }

//     TEST_PWM = PWM_VALUES[pwmIndex];

//     Serial.print("Selected TEST_PWM = ");
//     Serial.println(TEST_PWM);

//     Serial.print("EVENT,pwm_set,");
//     Serial.println(TEST_PWM);
//     return;
//   }

//   if (c == 'm' || c == 'M') {
//     printMenu();
//     return;
//   }

//   Serial.print("Unknown command: ");
//   Serial.println(c);
// }

// // ======================================================
// // Printing
// // ======================================================

// void printFloatOrNan(float value, int decimals) {
//   if (isnan(value)) {
//     Serial.print("nan");
//   } else {
//     Serial.print(value, decimals);
//   }
// }

// void printQuat(const char* label, Quat q) {
//   Serial.print(label);
//   Serial.print(": ");
//   Serial.print(q.w, 6);
//   Serial.print(", ");
//   Serial.print(q.x, 6);
//   Serial.print(", ");
//   Serial.print(q.y, 6);
//   Serial.print(", ");
//   Serial.println(q.z, 6);
// }

// void printFastData() {
//   unsigned long now = millis();

//   if (now - lastDataMs < DATA_MS) {
//     return;
//   }

//   lastDataMs = now;

//   unsigned long dataTimeMs = now;

//   if (testState == TEST_WAITING || testState == TEST_RUNNING || testState == TEST_DONE) {
//     dataTimeMs = now - trialStartMs;
//   }

//   Serial.print("DATA,");
//   Serial.print(dataTimeMs);
//   Serial.print(",");
//   Serial.print(axisVal(), 4);
//   Serial.print(",");
//   Serial.print(countsM1());
//   Serial.print(",");
//   Serial.print(countsM2());
//   Serial.print(",");
//   Serial.print(TEST_PWM);      // target/set PWM selected for the next Km trial
//   Serial.print(",");
//   Serial.print(lastPwm);       // actual PWM currently being sent to the motor
//   Serial.print(",");
//   Serial.print(lastU, 3);
//   Serial.print(",");
//   Serial.print(stateName());
//   Serial.print(",");
//   Serial.println(trialId);
// }

// void printDebug() {
//   unsigned long now = millis();

//   if (now - lastDebugMs < DEBUG_MS) {
//     return;
//   }

//   lastDebugMs = now;

//   float targetDeg = axisVal();
//   float currentDeg = axisVal();
//   float errorDeg = 0.0;

//   Serial.print("\nTargetDeg: ");
//   Serial.print(targetDeg, 2);

//   Serial.print(" | CurrentDeg: ");
//   Serial.print(currentDeg, 2);

//   Serial.print(" | RawDeg: ");
//   Serial.print(rawAng, 2);

//   Serial.print(" | RejectedSpikes: ");
//   Serial.print(rejSpks);

//   Serial.print(" | ErrorDeg: ");
//   Serial.print(errorDeg, 2);

//   Serial.print(" | TargetPWM: ");
//   Serial.print(TEST_PWM);

//   Serial.print(" | PWMNorm: ");
//   Serial.print(lastUNorm, 3);

//   Serial.print(" | PWM: ");
//   Serial.print(lastPwm);

//   Serial.print(" | UCmd: ");
//   Serial.print(lastU, 3);

//   Serial.print(" | Mode: ");
//   Serial.print(stateName());

//   Serial.print(" | FreqHz: ");
//   Serial.print(0.000, 3);

//   Serial.print(" | M1Counts: ");
//   Serial.print(countsM1());

//   Serial.print(" | M2Counts: ");
//   Serial.println(countsM2());
// }

// void printData() {
//   printFastData();
//   printDebug();

//   unsigned long now = millis();

//   if (now - lastQuatMs >= QUAT_MS) {
//     lastQuatMs = now;

//     printQuat("qUpperZeroed", qUpZeroed);
//     printQuat("qForearmZeroed", qForeZeroed);
//     printQuat("qJointZeroed", qJointZeroed);
//   }
// }

// void printMenu() {
//   Serial.println();
//   Serial.println("========== Km IDENTIFICATION MENU ==========");
//   Serial.println("n    : Start one fixed-time Km trial");
//   Serial.println("v    : Select next worksheet PWM value");
//   Serial.println("+/-  : Increase/decrease TEST_PWM by 5");
//   Serial.println("a    : Manual Motor 2 reverse/down");
//   Serial.println("d    : Manual Motor 2 forward/up");
//   Serial.println("Left : Manual Motor 2 reverse/down");
//   Serial.println("Right: Manual Motor 2 forward/up");
//   Serial.println("s/p/e/space : Emergency stop");
//   Serial.println("r    : Reset IMU and encoders");
//   Serial.println("m    : Print menu");
//   Serial.println();
//   Serial.println("DATA format:");
//   Serial.println("DATA,time_ms,theta_deg,m1_counts,m2_counts,target_pwm,pwm,u_cmd,mode,trial_id");
//   Serial.println("RESULT format:");
//   Serial.println("RESULT,trial_id,pwm,u_cmd,test_time_s,c0,ct,delta_c,delta_l_m,cable_speed_mps,km_raw,km_deadband,early_stop");
//   Serial.println();
//   Serial.print("TEST_PWM = ");
//   Serial.println(TEST_PWM);
//   Serial.print("TEST_TIME_MS = ");
//   Serial.println(TEST_TIME_MS);
//   Serial.print("R_SPOOL_M = ");
//   Serial.println(R_SPOOL_M, 5);
//   Serial.print("COUNTS_PER_REV = ");
//   Serial.println(COUNTS_PER_REV, 0);
//   Serial.print("U_DEAD = ");
//   Serial.println(U_DEAD);
//   Serial.println("============================================");
//   Serial.println();
// }

// // ======================================================
// // Setup and loop
// // ======================================================

// void setup() {
//   Serial.begin(115200);

//   pinMode(M1_IN1, OUTPUT);
//   pinMode(M1_IN2, OUTPUT);
//   pinMode(M2_IN1, OUTPUT);
//   pinMode(M2_IN2, OUTPUT);

//   offAll();

//   delay(1500);

//   Serial.println("Starting Km identification test program...");

//   if (!imuStart()) {
//     Serial.println("IMU startup failed. Check wiring.");
//   }

//   enc1.write(0);
//   enc2.write(0);

//   lastCtrlUs = micros();
//   lastDataMs = 0;
//   lastDebugMs = 0;
//   lastQuatMs = 0;

//   printMenu();
// }

// void loop() {
//   serialCheck();

//   unsigned long nowUs = micros();

//   if (nowUs - lastCtrlUs >= CTRL_US) {
//     lastCtrlUs = nowUs;

//     imuRead();

//     if (manMode) {
//       driveM2(manDir, MAN_PWM);
//     } else {
//       updateKmTrial();
//     }
//   }

//   printData();
// }