// #include <Arduino.h>
// #include <math.h>

// // ----------------------
// // Motor 1 pins
// // ----------------------
// #define M1_IN1 5
// #define M1_IN2 6
// #define M1_ENCA 2   // Encoder 1 Yellow Wire, Uno interrupt pin
// #define M1_ENCB 4   // Encoder 1 White Wire

// // ----------------------
// // Motor 2 pins
// // ----------------------
// #define M2_IN1 9
// #define M2_IN2 10
// #define M2_ENCA 3   // Encoder 2 Yellow Wire, Uno interrupt pin
// #define M2_ENCB 7   // Encoder 2 White Wire

// // ----------------------
// // Encoder positions
// // ----------------------
// volatile long pos1 = 0;
// volatile long pos2 = 0;

// // ----------------------
// // Direction constants
// // ----------------------
// const int FORWARD = 1;
// const int REVERSE = -1;

// // ----------------------
// // Encoder / gearbox settings
// // ----------------------
// const int MOTOR_COUNTS_PER_REV_FULL = 64;
// const int GEAR_RATIO = 270;
// const int OUTPUT_COUNTS_PER_REV = MOTOR_COUNTS_PER_REV_FULL * GEAR_RATIO; // 1216 counts/rev

// const int ENCODER_SIGN_1 = 1;
// const int ENCODER_SIGN_2 = 1;

// const int MOTOR_SIGN_1 = 1;
// const int MOTOR_SIGN_2 = 1;

// // ----------------------
// // PID constants
// // ----------------------
// float kp = 0.12;
// float kd = 0.012;
// float ki = 0.0;

// // ----------------------
// // Motor limits
// // ----------------------
// const int MIN_PWM = 125;
// const int MAX_PWM = 255;

// // ----------------------
// // Position tolerance
// // ----------------------
// const int TOLERANCE_COUNTS = 10;

// // ----------------------
// // Timing
// // ----------------------
// const unsigned long CONTROL_PERIOD_US = 10000; // 10 ms
// const unsigned long PLOT_INTERVAL_MS = 50;

// unsigned long lastControlTime = 0;
// unsigned long lastPlotTime = 0;

// // ----------------------
// // Motor 1 controller variables
// // ----------------------
// long target1 = 0;
// float eprev1 = 0.0;
// float eintegral1 = 0.0;
// int activeMaxPwm1 = 220;

// // ----------------------
// // Motor 2 controller variables
// // ----------------------
// long target2 = 0;
// float eprev2 = 0.0;
// float eintegral2 = 0.0;
// int activeMaxPwm2 = 220;

// // ----------------------
// // Function declarations
// // ----------------------
// void readEncoder1();
// void readEncoder2();

// void moveMotor1ByDegrees(float degrees, int direction, int maxPwm);
// // void moveMotor2ByDegrees(float degrees, int direction, int maxPwm);
// void holdTargetsFor(unsigned long holdTimeMs);

// void setMotor1RelativeTarget(float degrees, int direction, int maxPwm);
// void setMotor2RelativeTarget(float degrees, int direction, int maxPwm);

// void updateBothPositionPID();
// void updateMotor1PID(float dt);
// void updateMotor2PID(float dt);

// bool motor1TargetReached();
// bool motor2TargetReached();

// long getMotor1Position();
// long getMotor2Position();

// long degreesToCounts(float degrees);
// float countsToDegrees(long counts);

// void setMotor1(int dir, int pwmVal);
// void setMotor2(int dir, int pwmVal);

// void stopMotor1();
// void stopMotor2();
// void brakeMotor1();
// void brakeMotor2();

// void printData();

// void setup() {
//   Serial.begin(115200);
//   delay(2000);

//   // ----------------------
//   // Encoder setup
//   // ----------------------
//   pinMode(M1_ENCA, INPUT_PULLUP);
//   pinMode(M1_ENCB, INPUT_PULLUP);

//   pinMode(M2_ENCA, INPUT_PULLUP);
//   pinMode(M2_ENCB, INPUT_PULLUP);

//   attachInterrupt(digitalPinToInterrupt(M1_ENCA), readEncoder1, RISING);
//   attachInterrupt(digitalPinToInterrupt(M2_ENCA), readEncoder2, RISING);

//   // ----------------------
//   // Motor setup
//   // ----------------------
//   pinMode(M1_IN1, OUTPUT);
//   pinMode(M1_IN2, OUTPUT);
//   pinMode(M2_IN1, OUTPUT);
//   pinMode(M2_IN2, OUTPUT);

//   stopMotor1();
//   stopMotor2();

//   // Reset encoder counts
//   noInterrupts();
//   pos1 = 0;
//   pos2 = 0;
//   interrupts();

//   target1 = getMotor1Position();
//   target2 = getMotor2Position();

//   lastControlTime = micros();

//   Serial.println("target1 pos1 error1 target2 pos2 error2");

//   // ----------------------
//   // Motion sequence
//   // ----------------------

//   // moveMotor2ByDegrees(220.0, REVERSE, 220);
//   // holdTargetsFor(3000);

//   // moveMotor2ByDegrees(220.0, FORWARD, 220);
//   // holdTargetsFor(3000);
  





//   Serial.println("Sequence complete. Holding final targets.");
// }

// void loop() {
//   if (Serial.available() > 0) {
//     char cmd = Serial.read();

//     if (cmd == '1') {
//       setMotor1(REVERSE, 180);
//     }
//     else if (cmd == '2') {
//       setMotor1(FORWARD, 180);
//     }
//     else if (cmd == '0') {
//       stopMotor1();
//     }
//   }
// }

// // ----------------------
// // Encoder interrupt functions
// // ----------------------

// void readEncoder1() {
//   int b = digitalRead(M1_ENCB);

//   if (b == HIGH) {
//     pos1++;
//   } 
//   else {
//     pos1--;
//   }
// }

// void readEncoder2() {
//   int b = digitalRead(M2_ENCB);

//   if (b == HIGH) {
//     pos2++;
//   } 
//   else {
//     pos2--;
//   }
// }

// // ----------------------
// // Motion command functions
// // ----------------------

// void moveMotor1ByDegrees(float degrees, int direction, int maxPwm) {
//   setMotor1RelativeTarget(degrees, direction, maxPwm);

//   while (!motor1TargetReached()) {
//     updateBothPositionPID();
//     delay(1);
//   }

//   brakeMotor1();
// }

// // void moveMotor2ByDegrees(float degrees, int direction, int maxPwm) {
// //   setMotor2RelativeTarget(degrees, direction, maxPwm);

// //   while (!motor2TargetReached()) {
// //     updateBothPositionPID();
// //     delay(1);
// //   }

// //   brakeMotor2();
// // }

// void holdTargetsFor(unsigned long holdTimeMs) {
//   unsigned long startTime = millis();

//   while (millis() - startTime < holdTimeMs) {
//     updateBothPositionPID();
//     delay(1);
//   }
// }

// void setMotor1RelativeTarget(float degrees, int direction, int maxPwm) {
//   long moveCounts = degreesToCounts(degrees);

//   target1 = target1 + direction * moveCounts;
//   activeMaxPwm1 = constrain(maxPwm, MIN_PWM, MAX_PWM);

//   eintegral1 = 0.0;
//   eprev1 = target1 - getMotor1Position();
// }

// void setMotor2RelativeTarget(float degrees, int direction, int maxPwm) {
//   long moveCounts = degreesToCounts(degrees);

//   target2 = target2 + direction * moveCounts;
//   activeMaxPwm2 = constrain(maxPwm, MIN_PWM, MAX_PWM);

//   eintegral2 = 0.0;
//   eprev2 = target2 - getMotor2Position();
// }

// // ----------------------
// // PID update functions
// // ----------------------

// void updateBothPositionPID() {
//   unsigned long now = micros();

//   if (now - lastControlTime < CONTROL_PERIOD_US) {
//     return;
//   }

//   float dt = (now - lastControlTime) / 1000000.0;
//   lastControlTime = now;

//   if (dt <= 0) {
//     dt = 0.001;
//   }

//   updateMotor1PID(dt);
//   updateMotor2PID(dt);

//   printData();
// }

// void updateMotor1PID(float dt) {
//   long pos = getMotor1Position();

//   float error = target1 - pos;
//   float absError = fabs(error);

//   if (absError <= TOLERANCE_COUNTS) {
//     brakeMotor1();
//     eintegral1 = 0.0;
//     eprev1 = error;
//     return;
//   }

//   float dedt = (error - eprev1) / dt;

//   eintegral1 = eintegral1 + error * dt;

//   if (eintegral1 > 300) {
//     eintegral1 = 300;
//   }

//   if (eintegral1 < -300) {
//     eintegral1 = -300;
//   }

//   float u = kp * error + kd * dedt + ki * eintegral1;

//   int pwm = abs((int)u);

//   if (pwm > activeMaxPwm1) {
//     pwm = activeMaxPwm1;
//   }

//   if (pwm > 0 && pwm < MIN_PWM) {
//     pwm = MIN_PWM;
//   }

//   int dir = FORWARD;

//   if (u < 0) {
//     dir = REVERSE;
//   }

//   dir = dir * MOTOR_SIGN_1;

//   setMotor1(dir, pwm);

//   eprev1 = error;
// }

// void updateMotor2PID(float dt) {
//   long pos = getMotor2Position();

//   float error = target2 - pos;
//   float absError = fabs(error);

//   if (absError <= TOLERANCE_COUNTS) {
//     brakeMotor2();
//     eintegral2 = 0.0;
//     eprev2 = error;
//     return;
//   }

//   float dedt = (error - eprev2) / dt;

//   eintegral2 = eintegral2 + error * dt;

//   if (eintegral2 > 300) {
//     eintegral2 = 300;
//   }

//   if (eintegral2 < -300) {
//     eintegral2 = -300;
//   }

//   float u = kp * error + kd * dedt + ki * eintegral2;

//   int pwm = abs((int)u);

//   if (pwm > activeMaxPwm2) {
//     pwm = activeMaxPwm2;
//   }

//   if (pwm > 0 && pwm < MIN_PWM) {
//     pwm = MIN_PWM;
//   }

//   int dir = FORWARD;

//   if (u < 0) {
//     dir = REVERSE;
//   }

//   dir = dir * MOTOR_SIGN_2;

//   setMotor2(dir, pwm);

//   eprev2 = error;
// }

// // ----------------------
// // Target checks
// // ----------------------

// bool motor1TargetReached() {
//   long error = target1 - getMotor1Position();
//   return labs(error) <= TOLERANCE_COUNTS;
// }

// bool motor2TargetReached() {
//   long error = target2 - getMotor2Position();
//   return labs(error) <= TOLERANCE_COUNTS;
// }

// // ----------------------
// // Conversion functions
// // ----------------------

// long degreesToCounts(float degrees) {
//   return (long)((degrees / 360.0) * OUTPUT_COUNTS_PER_REV);
// }

// float countsToDegrees(long counts) {
//   return ((float)counts / OUTPUT_COUNTS_PER_REV) * 360.0;
// }

// // ----------------------
// // Encoder functions
// // ----------------------

// long getMotor1Position() {
//   long currentPos;

//   noInterrupts();
//   currentPos = pos1;
//   interrupts();

//   return currentPos * ENCODER_SIGN_1;
// }

// long getMotor2Position() {
//   long currentPos;

//   noInterrupts();
//   currentPos = pos2;
//   interrupts();

//   return currentPos * ENCODER_SIGN_2;
// }

// // ----------------------
// // Motor control functions
// // ----------------------

// void setMotor1(int dir, int pwmVal) {
//   pwmVal = constrain(pwmVal, 0, 255);

//   if (dir == FORWARD) {
//     analogWrite(M1_IN1, pwmVal);
//     analogWrite(M1_IN2, 0);
//   } 
//   else if (dir == REVERSE) {
//     analogWrite(M1_IN1, 0);
//     analogWrite(M1_IN2, pwmVal);
//   } 
//   else {
//     stopMotor1();
//   }
// }

// void setMotor2(int dir, int pwmVal) {
//   pwmVal = constrain(pwmVal, 0, 255);

//   if (dir == FORWARD) {
//     analogWrite(M2_IN1, pwmVal);
//     analogWrite(M2_IN2, 0);
//   } 
//   else if (dir == REVERSE) {
//     analogWrite(M2_IN1, 0);
//     analogWrite(M2_IN2, pwmVal);
//   } 
//   else {
//     stopMotor2();
//   }
// }

// void stopMotor1() {
//   analogWrite(M1_IN1, 0);
//   analogWrite(M1_IN2, 0);
// }

// void stopMotor2() {
//   analogWrite(M2_IN1, 0);
//   analogWrite(M2_IN2, 0);
// }

// void brakeMotor1() {
//   analogWrite(M1_IN1, 255);
//   analogWrite(M1_IN2, 255);
// }

// void brakeMotor2() {
//   analogWrite(M2_IN1, 255);
//   analogWrite(M2_IN2, 255);
// }

// // ----------------------
// // Serial output
// // ----------------------

// void printData() {
//   if (millis() - lastPlotTime >= PLOT_INTERVAL_MS) {
//     lastPlotTime = millis();

//     long currentPos1 = getMotor1Position();
//     long currentPos2 = getMotor2Position();

//     long error1 = target1 - currentPos1;
//     long error2 = target2 - currentPos2;

//     Serial.print("target1:");
//     Serial.print(target1);
//     Serial.print(" ");

//     Serial.print("pos1:");
//     Serial.print(currentPos1);
//     Serial.print(" ");

//     Serial.print("error1:");
//     Serial.print(error1);
//     Serial.print(" ");

//     Serial.print("target2:");
//     Serial.print(target2);
//     Serial.print(" ");

//     Serial.print("pos2:");
//     Serial.print(currentPos2);
//     Serial.print(" ");

//     Serial.print("error2:");
//     Serial.println(error2);
//   }
// }      



// // #include <Arduino.h>
// // #include <math.h>

// // // ----------------------
// // // Motor 1 pins
// // // ----------------------
// // #define M1_IN1 5
// // #define M1_IN2 6
// // #define M1_ENCA 2
// // #define M1_ENCB 4

// // // ----------------------
// // // Motor 2 pins
// // // ----------------------
// // #define M2_IN1 9
// // #define M2_IN2 10
// // #define M2_ENCA 3
// // #define M2_ENCB 7

// // // ----------------------
// // // Encoder positions
// // // ----------------------
// // volatile long pos1 = 0;
// // volatile long pos2 = 0;

// // // ----------------------
// // // Direction constants
// // // ----------------------
// // const int FORWARD = 1;
// // const int REVERSE = -1;

// // // ----------------------
// // // Encoder / gearbox settings
// // // ----------------------
// // const int MOTOR_COUNTS_PER_REV_FULL = 64;
// // const int GEAR_RATIO = 270;
// // const long OUTPUT_COUNTS_PER_REV = (long)MOTOR_COUNTS_PER_REV_FULL * GEAR_RATIO;

// // const int ENCODER_SIGN_1 = 1;
// // const int ENCODER_SIGN_2 = 1;

// // const int MOTOR_SIGN_1 = 1;
// // const int MOTOR_SIGN_2 = 1;

// // // ----------------------
// // // PID constants
// // // ----------------------
// // float kp = 0.12;
// // float kd = 0.012;
// // float ki = 0.0;

// // // ----------------------
// // // Motor limits
// // // ----------------------
// // const int MIN_PWM = 125;
// // const int MAX_PWM = 255;
// // const int DEFAULT_COMMAND_PWM = 220;

// // // ----------------------
// // // Position tolerance
// // // ----------------------
// // const int TOLERANCE_COUNTS = 10;

// // // ----------------------
// // // Command settings
// // // ----------------------
// // const float STEP_DEGREES_M1 = 15.0;
// // const float STEP_DEGREES_M2 = 15.0;

// // // ----------------------
// // // Timing
// // // ----------------------
// // const unsigned long CONTROL_PERIOD_US = 10000;
// // const unsigned long STATUS_INTERVAL_MS = 100;

// // unsigned long lastControlTime = 0;
// // unsigned long lastStatusTime = 0;

// // // ----------------------
// // // Motor 1 controller variables
// // // ----------------------
// // long target1 = 0;
// // float eprev1 = 0.0;
// // float eintegral1 = 0.0;
// // int activeMaxPwm1 = DEFAULT_COMMAND_PWM;

// // // ----------------------
// // // Motor 2 controller variables
// // // ----------------------
// // long target2 = 0;
// // float eprev2 = 0.0;
// // float eintegral2 = 0.0;
// // int activeMaxPwm2 = DEFAULT_COMMAND_PWM;

// // void readEncoder1();
// // void readEncoder2();

// // void readSerialCommands();
// // void handleCommand(char command);

// // void setMotor1RelativeTarget(float degrees, int direction, int maxPwm);
// // void setMotor2RelativeTarget(float degrees, int direction, int maxPwm);
// // void holdCurrentPosition();
// // void resetEncoderPositions();

// // void updateBothPositionPID();
// // void updateMotor1PID(float dt);
// // void updateMotor2PID(float dt);

// // long getMotor1Position();
// // long getMotor2Position();

// // long degreesToCounts(float degrees);
// // float countsToDegrees(long counts);

// // void setMotor1(int dir, int pwmVal);
// // void setMotor2(int dir, int pwmVal);

// // void stopMotor1();
// // void stopMotor2();
// // void brakeMotor1();
// // void brakeMotor2();

// // void printData(bool force = false);
// // void printControls();

// // void setup() {
// //   Serial.begin(115200);
// //   delay(2000);

// //   pinMode(M1_ENCA, INPUT_PULLUP);
// //   pinMode(M1_ENCB, INPUT_PULLUP);
// //   pinMode(M2_ENCA, INPUT_PULLUP);
// //   pinMode(M2_ENCB, INPUT_PULLUP);

// //   attachInterrupt(digitalPinToInterrupt(M1_ENCA), readEncoder1, RISING);
// //   attachInterrupt(digitalPinToInterrupt(M2_ENCA), readEncoder2, RISING);

// //   pinMode(M1_IN1, OUTPUT);
// //   pinMode(M1_IN2, OUTPUT);
// //   pinMode(M2_IN1, OUTPUT);
// //   pinMode(M2_IN2, OUTPUT);

// //   stopMotor1();
// //   stopMotor2();

// //   resetEncoderPositions();

// //   target1 = getMotor1Position();
// //   target2 = getMotor2Position();
// //   lastControlTime = micros();

// //   printControls();
// // }

// // void loop() {
// //   readSerialCommands();
// //   updateBothPositionPID();
// //   printData();
// // }

// // void readEncoder1() {
// //   int b = digitalRead(M1_ENCB);
// //   if (b == HIGH) {
// //     pos1++;
// //   } else {
// //     pos1--;
// //   }
// // }

// // void readEncoder2() {
// //   int b = digitalRead(M2_ENCB);
// //   if (b == HIGH) {
// //     pos2++;
// //   } else {
// //     pos2--;
// //   }
// // }

// // void readSerialCommands() {
// //   while (Serial.available() > 0) {
// //     char incoming = (char)Serial.read();
// //     handleCommand(incoming);
// //   }
// // }

// // void handleCommand(char command) {
// //   if (command == 'U') {
// //     setMotor1RelativeTarget(STEP_DEGREES_M1, FORWARD, DEFAULT_COMMAND_PWM);
// //     Serial.println("CMD U: Motor 1 forward");
// //   } else if (command == 'D') {
// //     setMotor1RelativeTarget(STEP_DEGREES_M1, REVERSE, DEFAULT_COMMAND_PWM);
// //     Serial.println("CMD D: Motor 1 reverse");
// //   } else if (command == 'R') {
// //     setMotor2RelativeTarget(STEP_DEGREES_M2, FORWARD, DEFAULT_COMMAND_PWM);
// //     Serial.println("CMD R: Motor 2 forward");
// //   } else if (command == 'L') {
// //     setMotor2RelativeTarget(STEP_DEGREES_M2, REVERSE, DEFAULT_COMMAND_PWM);
// //     Serial.println("CMD L: Motor 2 reverse");
// //   } else if (command == 'H') {
// //     holdCurrentPosition();
// //     Serial.println("CMD H: Hold current position");
// //   } else if (command == 'Z') {
// //     resetEncoderPositions();
// //     holdCurrentPosition();
// //     Serial.println("CMD Z: Encoder counts reset to zero");
// //   } else if (command == 'P') {
// //     printData(true);
// //   } else if (command == '?') {
// //     printControls();
// //   }
// // }

// // void setMotor1RelativeTarget(float degrees, int direction, int maxPwm) {
// //   long moveCounts = degreesToCounts(degrees);
// //   target1 = target1 + direction * moveCounts;
// //   activeMaxPwm1 = constrain(maxPwm, MIN_PWM, MAX_PWM);
// //   eintegral1 = 0.0;
// //   eprev1 = target1 - getMotor1Position();
// // }

// // void setMotor2RelativeTarget(float degrees, int direction, int maxPwm) {
// //   long moveCounts = degreesToCounts(degrees);
// //   target2 = target2 + direction * moveCounts;
// //   activeMaxPwm2 = constrain(maxPwm, MIN_PWM, MAX_PWM);
// //   eintegral2 = 0.0;
// //   eprev2 = target2 - getMotor2Position();
// // }

// // void holdCurrentPosition() {
// //   target1 = getMotor1Position();
// //   target2 = getMotor2Position();
// //   eintegral1 = 0.0;
// //   eintegral2 = 0.0;
// //   eprev1 = 0.0;
// //   eprev2 = 0.0;
// //   brakeMotor1();
// //   brakeMotor2();
// // }

// // void resetEncoderPositions() {
// //   noInterrupts();
// //   pos1 = 0;
// //   pos2 = 0;
// //   interrupts();
// // }

// // void updateBothPositionPID() {
// //   unsigned long now = micros();
// //   if (now - lastControlTime < CONTROL_PERIOD_US) {
// //     return;
// //   }

// //   float dt = (now - lastControlTime) / 1000000.0;
// //   lastControlTime = now;

// //   if (dt <= 0) {
// //     dt = 0.001;
// //   }

// //   updateMotor1PID(dt);
// //   updateMotor2PID(dt);
// // }

// // void updateMotor1PID(float dt) {
// //   long pos = getMotor1Position();
// //   float error = target1 - pos;
// //   float absError = fabs(error);

// //   if (absError <= TOLERANCE_COUNTS) {
// //     brakeMotor1();
// //     eintegral1 = 0.0;
// //     eprev1 = error;
// //     return;
// //   }

// //   float dedt = (error - eprev1) / dt;
// //   eintegral1 = eintegral1 + error * dt;

// //   if (eintegral1 > 300) {
// //     eintegral1 = 300;
// //   }
// //   if (eintegral1 < -300) {
// //     eintegral1 = -300;
// //   }

// //   float u = kp * error + kd * dedt + ki * eintegral1;
// //   int pwm = abs((int)u);

// //   if (pwm > activeMaxPwm1) {
// //     pwm = activeMaxPwm1;
// //   }
// //   if (pwm > 0 && pwm < MIN_PWM) {
// //     pwm = MIN_PWM;
// //   }

// //   int dir = FORWARD;
// //   if (u < 0) {
// //     dir = REVERSE;
// //   }

// //   dir = dir * MOTOR_SIGN_1;
// //   setMotor1(dir, pwm);
// //   eprev1 = error;
// // }

// // void updateMotor2PID(float dt) {
// //   long pos = getMotor2Position();
// //   float error = target2 - pos;
// //   float absError = fabs(error);

// //   if (absError <= TOLERANCE_COUNTS) {
// //     brakeMotor2();
// //     eintegral2 = 0.0;
// //     eprev2 = error;
// //     return;
// //   }

// //   float dedt = (error - eprev2) / dt;
// //   eintegral2 = eintegral2 + error * dt;

// //   if (eintegral2 > 300) {
// //     eintegral2 = 300;
// //   }
// //   if (eintegral2 < -300) {
// //     eintegral2 = -300;
// //   }

// //   float u = kp * error + kd * dedt + ki * eintegral2;
// //   int pwm = abs((int)u);

// //   if (pwm > activeMaxPwm2) {
// //     pwm = activeMaxPwm2;
// //   }
// //   if (pwm > 0 && pwm < MIN_PWM) {
// //     pwm = MIN_PWM;
// //   }

// //   int dir = FORWARD;
// //   if (u < 0) {
// //     dir = REVERSE;
// //   }

// //   dir = dir * MOTOR_SIGN_2;
// //   setMotor2(dir, pwm);
// //   eprev2 = error;
// // }

// // long degreesToCounts(float degrees) {
// //   return (long)((degrees / 360.0) * OUTPUT_COUNTS_PER_REV);
// // }

// // float countsToDegrees(long counts) {
// //   return ((float)counts / OUTPUT_COUNTS_PER_REV) * 360.0;
// // }

// // long getMotor1Position() {
// //   long currentPos;
// //   noInterrupts();
// //   currentPos = pos1;
// //   interrupts();
// //   return currentPos * ENCODER_SIGN_1;
// // }

// // long getMotor2Position() {
// //   long currentPos;
// //   noInterrupts();
// //   currentPos = pos2;
// //   interrupts();
// //   return currentPos * ENCODER_SIGN_2;
// // }

// // void setMotor1(int dir, int pwmVal) {
// //   pwmVal = constrain(pwmVal, 0, 255);

// //   if (dir == FORWARD) {
// //     analogWrite(M1_IN1, pwmVal);
// //     analogWrite(M1_IN2, 0);
// //   } else if (dir == REVERSE) {
// //     analogWrite(M1_IN1, 0);
// //     analogWrite(M1_IN2, pwmVal);
// //   } else {
// //     stopMotor1();
// //   }
// // }

// // void setMotor2(int dir, int pwmVal) {
// //   pwmVal = constrain(pwmVal, 0, 255);

// //   if (dir == FORWARD) {
// //     analogWrite(M2_IN1, pwmVal);
// //     analogWrite(M2_IN2, 0);
// //   } else if (dir == REVERSE) {
// //     analogWrite(M2_IN1, 0);
// //     analogWrite(M2_IN2, pwmVal);
// //   } else {
// //     stopMotor2();
// //   }
// // }

// // void stopMotor1() {
// //   analogWrite(M1_IN1, 0);
// //   analogWrite(M1_IN2, 0);
// // }

// // void stopMotor2() {
// //   analogWrite(M2_IN1, 0);
// //   analogWrite(M2_IN2, 0);
// // }

// // void brakeMotor1() {
// //   analogWrite(M1_IN1, 255);
// //   analogWrite(M1_IN2, 255);
// // }

// // void brakeMotor2() {
// //   analogWrite(M2_IN1, 255);
// //   analogWrite(M2_IN2, 255);
// // }

// // void printData(bool force) {
// //   if (!force && millis() - lastStatusTime < STATUS_INTERVAL_MS) {
// //     return;
// //   }

// //   lastStatusTime = millis();

// //   long currentPos1 = getMotor1Position();
// //   long currentPos2 = getMotor2Position();

// //   Serial.print("M1 counts:");
// //   Serial.print(currentPos1);
// //   Serial.print(" deg:");
// //   Serial.print(countsToDegrees(currentPos1), 2);
// //   Serial.print(" targetDeg:");
// //   Serial.print(countsToDegrees(target1), 2);
// //   Serial.print(" | ");

// //   Serial.print("M2 counts:");
// //   Serial.print(currentPos2);
// //   Serial.print(" deg:");
// //   Serial.print(countsToDegrees(currentPos2), 2);
// //   Serial.print(" targetDeg:");
// //   Serial.println(countsToDegrees(target2), 2);
// // }

// // void printControls() {
// //   Serial.println("Serial motor control ready.");
// //   Serial.println("Commands: U D L R H Z P ?");
// //   Serial.println("U/D = Motor 1 forward/reverse");
// //   Serial.println("L/R = Motor 2 reverse/forward");
// //   Serial.println("H = hold current position");
// //   Serial.println("Z = reset encoders to zero");
// //   Serial.println("P = print positions now");
// //   Serial.println("? = print help");
// // }
