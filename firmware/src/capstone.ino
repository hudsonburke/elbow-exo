/*
* ============================================================
*  ExoArm — Arduino Nano ESP32
*  Standalone PWM + PI Control
*  Compatible with Arduino ESP32 Core 2.x
*  No BLE, No IMU, No MUX — ready for serial testing
* ============================================================
*
* COMMAND PROTOCOL (Serial @ 115200 baud, same as BLE format):
*   Send:  "<mode>,<speed>\n"
*   mode:  0=StopAll | 1=ElbowFlexExt | 2=ProSup
*          3=WristFlexExt | 4=WristAbdAdd
*   speed: 0=Stop | 1=Positive direction | 2=Negative direction
*
*   e.g.   "1,1"  → Elbow Flex
*          "1,2"  → Elbow Extend
*          "2,1"  → Pronate  (Motor 2 runs, Motor 3 coasts)
*          "2,2"  → Supinate (Motor 3 runs, Motor 2 coasts)
*          "3,1"  → Wrist Flex   (both actuators forward)
*          "3,2"  → Wrist Extend (both actuators reverse)
*          "4,1"  → Wrist Abduct (ACT1 fwd, ACT2 rev)
*          "4,2"  → Wrist Adduct (ACT1 rev, ACT2 fwd)
*          "0,0"  → Stop Everything
* ============================================================
*/
#include <Arduino.h>

// ============================================================
// [EDIT] — CONTROL MODE
// 0 = open-loop time-based ramp (no sensor needed — use now)
// 1 = PI closed-loop (requires IMU — flip when integrating)
// ============================================================
#define USE_PI_CONTROL 0

// ============================================================
// [EDIT] — PIN ASSIGNMENTS
//
// Motor 1  (Elbow Flex/Ext)      IN1=D4,  IN2=D5
// Motor 2  (Forearm Pronation)   IN1=D6,  IN2=D7
// Motor 3  (Forearm Supination)  IN1=D8,  IN2=D9
// Actuator 1 (Left  Wrist)       IN1=D10, IN2=D11
// Actuator 2 (Right Wrist)       IN1=D2,  IN2=D3
//
// Notes:
//   - D0/D1 avoided (hardware Serial TX/RX)
//   - D18/D19 avoided (I2C SDA/SCL — reserved for IMU/MUX)
//   - All pins above are PWM-capable on ESP32 core 3.x
// ============================================================

// Motor 1 — Elbow Flexion / Extension
const int M1_IN1  = 4;
const int M1_IN2  = 5;

// Motor 2 — Forearm Pronation (cable pull, runs alone)
const int M2_IN1  = 6;
const int M2_IN2  = 7;

// Motor 3 — Forearm Supination (cable pull, runs alone)
const int M3_IN1  = 8;
const int M3_IN2  = 9;

// Linear Actuator 1 — Left Wrist
// (parallel with ACT2 for flex/ext; opposing for abd/add)
const int ACT1_IN1 = 10;
const int ACT1_IN2 = 11;

// Linear Actuator 2 — Right Wrist
const int ACT2_IN1 = 2;
const int ACT2_IN2 = 3;

// ============================================================
// [EDIT] — PWM SETTINGS
// FREQ: 20 kHz is inaudible and safe for DRV8871 (max 200 kHz)
// RESOLUTION: 8-bit (0-255) — simpler and fully supported in 3.x
//   Note: 3.x supports up to 14-bit but 8-bit is the most
//   reliable across all ESP32 variants. Change to 10 and
//   PWM_MAX to 1023 only if you need finer speed resolution.
// ============================================================
const int PWM_FREQ       = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX        = 255;    // 2^8 - 1

// ============================================================
// [EDIT] — DUTY CYCLE LIMITS
// Expressed as 8-bit values (0–255).
// PWM_MIN_DUTY: ~35% — lowest duty that overcomes stiction.
//   Raise if motors stall or make grinding noise at start.
// PWM_MAX_DUTY: ~78% — safe rehab speed ceiling.
//   Raise toward 220 only if cables feel slack under load.
// ============================================================
const int PWM_MIN_DUTY = 90;    // ~35% of 255
const int PWM_MAX_DUTY = 200;   // ~78% of 255

// ============================================================
// [EDIT] — OPEN-LOOP RAMP TIMING
// RAMP_UP_MS:   ms from first command to full speed.
// RAMP_DOWN_MS: ms from command release to zero.
// ============================================================
const unsigned long RAMP_UP_MS   = 1500;   // 1.5 s to full speed
const unsigned long RAMP_DOWN_MS =  500;   // 0.5 s to coast

// ============================================================
// [EDIT] — WATCHDOG TIMEOUT
// If no valid command arrives within this many ms, all motors
// coast to stop. Set high during bench testing so you have
// time to type. Tighten to 300–500 ms for normal operation.
// ============================================================
const unsigned long CMD_TIMEOUT_MS = 10000;   // 10 s for testing

// ============================================================
// [EDIT] — WRIST ACTUATOR HOMING
// Both actuators drive forward for ACT_HOME_MS on boot to
// reach approximate mid-stroke (neutral resting position).
//
// HOW TO CALIBRATE:
//   1. Start with actuators fully retracted.
//   2. Set ACT_HOME_MS = 0 and upload — skip homing for now.
//   3. Use "3,1" command to extend, time how long full stroke
//      takes at ACT_HOME_DUTY.
//   4. Set ACT_HOME_MS to half that time.
//
// Set to 0 to skip homing until actuators are connected.
// ============================================================
const unsigned long ACT_HOME_MS   = 0;     // set to 0 until calibrated
const int           ACT_HOME_DUTY = 100;   // gentle homing speed (~39%)

// ============================================================
// [EDIT] — PI TARGET ANGLES (degrees) — from spec table
// Elbow:  0–120° | Pro/Sup: 0–60° | Wrist: ±45°
// Only used when USE_PI_CONTROL = 1.
// ============================================================
const float TARGET_ELBOW_FLEX  = 120.0f;
const float TARGET_ELBOW_EXT   =   0.0f;
const float TARGET_PRO         =  60.0f;
const float TARGET_SUP         =   0.0f;
const float TARGET_WRIST_FLEX  =  45.0f;
const float TARGET_WRIST_EXT   = -45.0f;
const float TARGET_WRIST_ABD   =  45.0f;
const float TARGET_WRIST_ADD   = -45.0f;

// ============================================================
// [EDIT] — PI GAINS
// Start with Ki = 0. Tune Kp until responsive without
// oscillation, then introduce small Ki to eliminate
// steady-state error.
//
// Scaling: output is duty (0–PWM_MAX_DUTY), input is degrees.
// Rule of thumb starting point:
//   Kp ≈ PWM_MAX_DUTY / (target_angle × 0.5)
// ============================================================
const float KP_ELBOW  = 1.3f;   const float KI_ELBOW  = 0.0f;
const float KP_PROSUP = 2.6f;   const float KI_PROSUP = 0.0f;
const float KP_WRIST  = 3.5f;   const float KI_WRIST  = 0.0f;

// ============================================================
// [KEEP] — MOTOR STRUCT (2.x version — pins + LEDC channels)
// In core 2.x, each PWM output needs a unique LEDC channel.
// ledcSetup(channel, freq, resolution) configures the channel,
// ledcAttachPin(pin, channel) binds pin to channel,
// ledcWrite(channel, duty) sets the duty cycle.
// ============================================================

struct Motor {
    int pin1;
    int pin2;
    int ch1;
    int ch2;
};

Motor mElbow  = { M1_IN1,   M1_IN2,    0,   1  };
Motor mPro    = { M2_IN1,   M2_IN2,    2,   3  };
Motor mSup    = { M3_IN1,   M3_IN2,    4,   5  };
Motor aWrist1 = { ACT1_IN1, ACT1_IN2,  6,   7  };
Motor aWrist2 = { ACT2_IN1, ACT2_IN2,  8,   9  };

// ============================================================
// [KEEP] — DRV8871 H-BRIDGE PRIMITIVES (2.x ledcWrite API)
// Truth table:
//   IN1=PWM, IN2=0   → forward (fast decay)
//   IN1=0,   IN2=PWM → reverse (fast decay)
//   IN1=0,   IN2=0   → coast   (Hi-Z, motor spins freely)
//   IN1=1,   IN2=1   → brake   (slow decay, holds position)
// ============================================================
void motorFwd(Motor& m, int duty) {
   ledcWrite(m.ch1, constrain(duty, 0, PWM_MAX));
   ledcWrite(m.ch2, 0);
}

void motorRev(Motor& m, int duty) {
   ledcWrite(m.ch1, 0);
   ledcWrite(m.ch2, constrain(duty, 0, PWM_MAX));
}

void motorStop(Motor& m) {
   ledcWrite(m.ch1, 0);
   ledcWrite(m.ch2, 0);
}

void motorBrake(Motor& m) {
   // Holds position — use after reaching target angle
   ledcWrite(m.ch1, PWM_MAX);
   ledcWrite(m.ch2, PWM_MAX);
}

void stopAll() {
   motorStop(mElbow);
   motorStop(mPro);
   motorStop(mSup);
   motorStop(aWrist1);
   motorStop(aWrist2);
}

// ============================================================
// [KEEP] — OPEN-LOOP TIME-BASED RAMP
// Ramps duty up linearly from PWM_MIN_DUTY to PWM_MAX_DUTY
// over RAMP_UP_MS, then ramps back down on release.
// State resets whenever mode or direction changes.
// ============================================================
unsigned long rampStartTime = 0;
unsigned long rampStopTime  = 0;
bool          isRamping     = false;

int computeRampDuty(bool moving) {
   unsigned long now = millis();
   if (moving) {
       if (!isRamping) {
           rampStartTime = now;
           isRamping = true;
       }
       float t = constrain(
           (float)(now - rampStartTime) / (float)RAMP_UP_MS,
           0.0f, 1.0f
       );
       return (int)(PWM_MIN_DUTY + t * (PWM_MAX_DUTY - PWM_MIN_DUTY));
   } else {
       if (isRamping) {
           rampStopTime = now;
           isRamping = false;
       }
       float t = constrain(
           (float)(now - rampStopTime) / (float)RAMP_DOWN_MS,
           0.0f, 1.0f
       );
       return (int)((1.0f - t) * PWM_MIN_DUTY);
   }
}

// ============================================================
// [KEEP] — PI CONTROLLER
// One PIState per DOF. resetPI() on mode change prevents
// integral windup from carrying over between moves.
// ============================================================

struct PIState {
    float         integral;
    unsigned long lastTime;
};

PIState piElbow  = {0.0f, 0};
PIState piProSup = {0.0f, 0};
PIState piWrist  = {0.0f, 0};

int computePI(PIState& st, float setpoint, float current,
             float kp, float ki) {
   unsigned long now = millis();
   float dt = (st.lastTime == 0)
              ? 0.02f
              : (float)(now - st.lastTime) / 1000.0f;
   st.lastTime = now;

   float error   = setpoint - current;
   st.integral  += error * dt;

   // Anti-windup clamp
   float maxI = (ki > 0.0f) ? (float)PWM_MAX_DUTY / ki : 1e6f;
   st.integral = constrain(st.integral, -maxI, maxI);

   float output = kp * error + ki * st.integral;
   return constrain((int)output, 0, PWM_MAX_DUTY);
}

void resetPI(PIState& st) {
   st.integral = 0.0f;
   st.lastTime = 0;
}

// ============================================================
// [KEEP / ADD] — MODE APPLICATION
// Called every loop tick. Recalculates duty each call so the
// ramp or PI output progresses smoothly between commands.
//
// [ADD] Replace the three 0.0f STUB values with real IMU reads
// when BNO055 + TCA9548A are integrated. Flip USE_PI_CONTROL
// to 1 at that point.
// ============================================================
int lastMode  = -1;
int lastSpeed = -1;

void applyMode(int mode, int speed) {
   bool moving = (speed != 0);

   // Reset ramp / PI state on any mode or direction change
   if (mode != lastMode || speed != lastSpeed) {
       isRamping = false;
       if (mode != lastMode) {
           resetPI(piElbow);
           resetPI(piProSup);
           resetPI(piWrist);
       }
       lastMode  = mode;
       lastSpeed = speed;
   }

   if (mode == 0 || speed == 0) {
       stopAll();
       return;
   }

   // [ADD] Replace these with real IMU reads when integrated:
   //   tcaselect(BICEP);   elbowAngle  = readEuler(bno);
   //   tcaselect(FOREARM); proSupAngle = readEuler(bno);
   //   tcaselect(WRIST);   wristAngle  = readEuler(bno);
   float elbowAngle  = 0.0f;   // STUB
   float proSupAngle = 0.0f;   // STUB
   float wristAngle  = 0.0f;   // STUB

   int duty = 0;

   switch (mode) {
       // MODE 1 — Elbow Flexion (1) / Extension (2)
       case 1:
           #if USE_PI_CONTROL
               duty = computePI(piElbow,
                   (speed == 1) ? TARGET_ELBOW_FLEX : TARGET_ELBOW_EXT,
                   elbowAngle, KP_ELBOW, KI_ELBOW);
           #else
               duty = computeRampDuty(moving);
           #endif
           (speed == 1) ? motorFwd(mElbow, duty)
                        : motorRev(mElbow, duty);
           break;

       // MODE 2 — Pronation (1) / Supination (2)
       // Only one motor runs at a time — prevents cable conflict.
       case 2:
           #if USE_PI_CONTROL
               duty = computePI(piProSup,
                   (speed == 1) ? TARGET_PRO : TARGET_SUP,
                   proSupAngle, KP_PROSUP, KI_PROSUP);
           #else
               duty = computeRampDuty(moving);
           #endif
           if (speed == 1) {
               motorFwd(mPro, duty);
               motorStop(mSup);
           } else {
               motorStop(mPro);
               motorFwd(mSup, duty);
           }
           break;

       // MODE 3 — Wrist Flex (1) / Extend (2)
       // Both actuators PARALLEL — same direction, same duty.
       case 3:
           #if USE_PI_CONTROL
               duty = computePI(piWrist,
                   (speed == 1) ? TARGET_WRIST_FLEX : TARGET_WRIST_EXT,
                   wristAngle, KP_WRIST, KI_WRIST);
           #else
               duty = computeRampDuty(moving);
           #endif
           if (speed == 1) {
               motorFwd(aWrist1, duty);
               motorFwd(aWrist2, duty);
           } else {
               motorRev(aWrist1, duty);
               motorRev(aWrist2, duty);
           }
           break;

       // MODE 4 — Wrist Abduction (1) / Adduction (2)
       // Actuators OPPOSING — one extends while other retracts.
       // Neutral position = mid-stroke (set by homeActuators()).
       case 4:
           #if USE_PI_CONTROL
               duty = computePI(piWrist,
                   (speed == 1) ? TARGET_WRIST_ABD : TARGET_WRIST_ADD,
                   wristAngle, KP_WRIST, KI_WRIST);
           #else
               duty = computeRampDuty(moving);
           #endif
           if (speed == 1) {
               motorFwd(aWrist1, duty);
               motorRev(aWrist2, duty);
           } else {
               motorRev(aWrist1, duty);
               motorFwd(aWrist2, duty);
           }
           break;

       default:
           stopAll();
           break;
   }
}
// ============================================================
// [ADD] — WRIST ACTUATOR HOMING
// Skip for now (ACT_HOME_MS = 0). Calibrate after actuators
// are physically connected and you've measured full stroke time.
// ============================================================
void homeActuators() {
   if (ACT_HOME_MS == 0) {
       Serial.println("[HOME] Skipped (ACT_HOME_MS = 0).");
       return;
   }
   Serial.println("[HOME] Homing wrist actuators to mid-stroke...");
   motorFwd(aWrist1, ACT_HOME_DUTY);
   motorFwd(aWrist2, ACT_HOME_DUTY);
   delay(ACT_HOME_MS);
   motorStop(aWrist1);
   motorStop(aWrist2);
   Serial.println("[HOME] Done.");
}

// ============================================================
// [KEEP] — SERIAL PARSER
// Same "mode,speed" format as BLE — no Python changes needed.
// ============================================================
bool parseSerial(const String& raw, int& mode, int& speed) {
   int comma = raw.indexOf(',');
   if (comma < 1) return false;
   mode  = raw.substring(0, comma).toInt();
   speed = raw.substring(comma + 1).toInt();
   if (mode  < 0 || mode  > 4) return false;
   if (speed < 0 || speed > 2) return false;
   return true;
}

// ============================================================
// [KEEP] — SETUP
// core 2.x: ledcSetup(channel, freq, resolution) then
//            ledcAttachPin(pin, channel).
// ============================================================
void setup() {
   Serial.begin(115200);
   delay(300);
   Serial.println("=== ExoArm Standalone Boot (core 2.x) ===");

   // Configure all 10 LEDC channels (0–9) with same freq & resolution
   for (int ch = 0; ch < 10; ch++) {
       ledcSetup(ch, PWM_FREQ, PWM_RESOLUTION);
   }

   // Bind each pin to its channel
   ledcAttachPin(M1_IN1,   0);
   ledcAttachPin(M1_IN2,   1);
   ledcAttachPin(M2_IN1,   2);
   ledcAttachPin(M2_IN2,   3);
   ledcAttachPin(M3_IN1,   4);
   ledcAttachPin(M3_IN2,   5);
   ledcAttachPin(ACT1_IN1, 6);
   ledcAttachPin(ACT1_IN2, 7);
   ledcAttachPin(ACT2_IN1, 8);
   ledcAttachPin(ACT2_IN2, 9);

   stopAll();
   Serial.println("[OK] PWM ready — 10 pins @ 20 kHz / 8-bit");

   homeActuators();

   Serial.println("[OK] Ready for commands.");
   Serial.println("     Format : <mode>,<speed>");
   Serial.println("     mode   : 0=Stop 1=Elbow 2=ProSup 3=WristFlex 4=WristAbd");
   Serial.println("     speed  : 0=Stop 1=Fwd/Flex/Pro 2=Rev/Ext/Sup");
}

// ============================================================
// [KEEP / ADD] — MAIN LOOP
// ============================================================
String        serialBuf   = "";
int           activeMode  = 0;
int           activeSpeed = 0;
unsigned long lastCmdTime = 0;


void loop() {
   // [ADD] IMU reads go here when integrating BNO055:
   // tcaselect(BICEP);   elbowAngle  = readEuler(bno);
   // tcaselect(FOREARM); proSupAngle = readEuler(bno);
   // tcaselect(WRIST);   wristAngle  = readEuler(bno);
 
   // Read Serial commands
   while (Serial.available()) {
       char c = (char)Serial.read();
       if (c == '\n' || c == '\r') {
           serialBuf.trim();
           if (serialBuf.length() > 0) {
               int m, s;
               if (parseSerial(serialBuf, m, s)) {
                   activeMode  = m;
                   activeSpeed = s;
                   lastCmdTime = millis();
                   Serial.print("[CMD] mode="); Serial.print(m);
                   Serial.print(" speed=");     Serial.println(s);
               } else {
                   Serial.println("[ERR] Bad command — use: mode,speed  e.g. 1,1");
               }
               serialBuf = "";
           }
       } else {
           if (serialBuf.length() < 16) serialBuf += c;
       }
   }

   // Watchdog — stop everything if commands go quiet
   if (lastCmdTime != 0 &&
       (millis() - lastCmdTime) > CMD_TIMEOUT_MS) {
       if (activeMode != 0 || activeSpeed != 0) {
           Serial.println("[WDT] Timeout — all motors stopped");
       }
       activeMode  = 0;
       activeSpeed = 0;
       stopAll();
       isRamping = false;
   }
   // Continuously reapply — this is what makes the ramp work
   applyMode(activeMode, activeSpeed);
   // 20 ms tick — fast enough for smooth ramp, matches IMU rate
   delay(20);
}

/*
* ============================================================
*  TESTING CHECKLIST
* ============================================================
*
*  BEFORE FIRST UPLOAD:
*   [ ] Fill in all 10 pin numbers at the top
*   [ ] Confirm ACT_HOME_MS = 0 (actuators not connected yet)
*   [ ] Board set to "Arduino Nano ESP32" in Tools menu
*   [ ] Serial Monitor: 115200 baud, line ending = Newline
*
*  FIRST MOTOR TEST:
*   [ ] Type "1,1" → Motor 1 should ramp up (elbow flex)
*   [ ] Type "0,0" → All stop
*   [ ] Type "1,2" → Motor 1 reverses (elbow extend)
*   [ ] Repeat for modes 2, 3, 4
*
*  IF MOTOR DIRECTION IS WRONG:
*   [ ] Physically swap OUT1/OUT2 wires on that DRV8871
*       Do NOT flip IN1/IN2 in software — keep code truthful
*
*  IF MOTOR DOESN'T MOVE:
*   [ ] Raise PWM_MIN_DUTY (try 120 → ~47%)
*   [ ] Check 12V is present on DRV8871 VM terminal
*   [ ] Check shared GND between Arduino and DRV8871
*
*  IF MOTOR IS TOO FAST / JERKY:
*   [ ] Lower PWM_MAX_DUTY (try 150)
*   [ ] Raise RAMP_UP_MS (try 2500)
*
*  ACTUATOR HOMING CALIBRATION (once connected):
*   [ ] Set ACT_HOME_MS = 0, use "3,1" to extend fully
*   [ ] Time how many ms at ACT_HOME_DUTY reaches full stroke
*   [ ] Set ACT_HOME_MS = half that value
*
*  INTEGRATION STUBS (search to find them):
*   "STUB"        — three IMU angle reads in applyMode()
*   "[ADD] IMU"   — read location at top of loop()
*   "[ADD] BLE"   — swap Serial block for rxChar.written()
* ============================================================
*/
