// #include <Arduino.h>
// #include <Encoder.h>

// // Encoder pins
// const int M1_ENC_A = 30;
// const int M1_ENC_B = 31;

// const int M2_ENC_A = 28;
// const int M2_ENC_B = 29;

// // Encoder objects
// Encoder enc1(M1_ENC_A, M1_ENC_B);
// Encoder enc2(M2_ENC_A, M2_ENC_B);

// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   Serial.println("Encoder count test started.");
//   Serial.println("Move the motors by hand or run the motor slowly.");
// }

// void loop() {
//   long m1_counts = enc1.read();
//   long m2_counts = enc2.read();

//   Serial.print("M1Counts: ");
//   Serial.print(m1_counts);

//   Serial.print(" | M2Counts: ");
//   Serial.println(m2_counts);

//   delay(100);
// }