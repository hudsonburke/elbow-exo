// #include <Arduino.h>
// #include <math.h>

// #define M1_IN1 5
// #define M1_IN2 6
// #define M1_ENCA 2
// #define M1_ENCB 4

// #define M2_IN1 9
// #define M2_IN2 10
// #define M2_ENCA 3
// #define M2_ENCB 7

// volatile long pos1 = 0;
// volatile long pos2 = 0;

// const int FORWARD = 1;
// const int REVERSE = -1;

// const int MANUAL_PWM = 180;

// void readEncoder1();
// void readEncoder2();

// void setMotor1(int dir, int pwmVal);
// void setMotor2(int dir, int pwmVal);

// void stopMotor1();
// void stopMotor2();

// long getMotor1Position();
// long getMotor2Position();

// void setup() {
//   Serial.begin(115200);
//   delay(2000);

//   pinMode(M1_ENCA, INPUT_PULLUP);
//   pinMode(M1_ENCB, INPUT_PULLUP);
//   pinMode(M2_ENCA, INPUT_PULLUP);
//   pinMode(M2_ENCB, INPUT_PULLUP);

//   attachInterrupt(digitalPinToInterrupt(M1_ENCA), readEncoder1, RISING);
//   attachInterrupt(digitalPinToInterrupt(M2_ENCA), readEncoder2, RISING);

//   pinMode(M1_IN1, OUTPUT);
//   pinMode(M1_IN2, OUTPUT);
//   pinMode(M2_IN1, OUTPUT);
//   pinMode(M2_IN2, OUTPUT);

//   stopMotor1();
//   stopMotor2();

//   Serial.println("Keyboard control ready.");
//   Serial.println("1 = reverse, 2 = forward, 0 = stop");
// }

// void loop() {
//   if (Serial.available() > 0) {
//     char cmd = Serial.read();

//     if (cmd == '1') {
//       setMotor2(REVERSE, MANUAL_PWM);
//     }
//     else if (cmd == '2') {
//       setMotor2(FORWARD, MANUAL_PWM);
//     }
//     else if (cmd == '0') {
//       stopMotor2();
//     }
//   }
// }

// void readEncoder1() {
//   int b = digitalRead(M1_ENCB);

//   if (b == HIGH) {
//     pos1++;
//   } else {
//     pos1--;
//   }
// }

// void readEncoder2() {
//   int b = digitalRead(M2_ENCB);

//   if (b == HIGH) {
//     pos2++;
//   } else {
//     pos2--;
//   }
// }

// long getMotor1Position() {
//   long currentPos;
//   noInterrupts();
//   currentPos = pos1;
//   interrupts();
//   return currentPos;
// }

// long getMotor2Position() {
//   long currentPos;
//   noInterrupts();
//   currentPos = pos2;
//   interrupts();
//   return currentPos;
// }

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