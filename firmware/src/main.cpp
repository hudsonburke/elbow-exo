
#include <Arduino.h>
#include <Encoder.h>


#define DRIVER1_IN1 4
#define DRIVER1_IN2 5
#define ENC1_A 6 // Encoder Yellow Wire 
#define ENC1_B 7 //Endoder White Wire

#define DRIVER2_IN1 11
#define DRIVER2_IN2 10
#define ENC2_A 8 // Encoder Yellow Wire 
#define ENC2_B 9 //Endoder White Wire

const int MOTOR_COUNTS_PER_REV = 64; // Number of encoder counts per revolution
const int GEAR_RATIO = 19; // Gear ratio of the motor 19:1
const int COUNTS_PER_REV = MOTOR_COUNTS_PER_REV * GEAR_RATIO; // Total counts per revolution of the output shaft

void setup() {
  pinMode(DRIVER1_IN1, OUTPUT);
  pinMode(DRIVER1_IN2, OUTPUT);
  pinMode(DRIVER2_IN1, OUTPUT);
  pinMode(DRIVER2_IN2, OUTPUT);
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);
}

void loop() {
  // Move forward
  digitalWrite(DRIVER1_IN1, HIGH);
  digitalWrite(DRIVER1_IN2, LOW);
  digitalWrite(DRIVER2_IN1, HIGH);
  digitalWrite(DRIVER2_IN2, LOW);
  delay(2000);

  // Move backward
  digitalWrite(DRIVER1_IN1, LOW);
  digitalWrite(DRIVER1_IN2, HIGH);
  digitalWrite(DRIVER2_IN1, LOW);
  digitalWrite(DRIVER2_IN2, HIGH);
  delay(2000);

  // Stop
  digitalWrite(DRIVER1_IN1, LOW);
  digitalWrite(DRIVER1_IN2, LOW);
  digitalWrite(DRIVER2_IN1, LOW);
  digitalWrite(DRIVER2_IN2, LOW);
  delay(2000);
}



