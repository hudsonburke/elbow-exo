// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>

// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// // Zero/reference values
// float yawZero = 0.0;
// float pitchZero = 0.0;
// float rollZero = 0.0;

// // Corrected angle values
// float yawAngle = 0.0;
// float pitchAngle = 0.0;
// float rollAngle = 0.0;

// // This handles wrap-around, like 359 degrees to 0 degrees
// float angleDifference(float currentAngle, float zeroAngle) {
//   float diff = currentAngle - zeroAngle;

//   while (diff > 180.0) {
//     diff -= 360.0;
//   }

//   while (diff < -180.0) {
//     diff += 360.0;
//   }

//   return diff;
// }

// void recalibrateIMU() {
//   sensors_event_t event;
//   bno.getEvent(&event);

//   yawZero = event.orientation.x;
//   pitchZero = event.orientation.y;
//   rollZero = event.orientation.z;

//   Serial.println("IMU recalibrated. Current position is now 0.");
// }

// void setup() {
//   Serial.begin(9600);
//   delay(1000);

//   Serial.println("BNO055 IMU Test");

//   Wire.begin();

//   if (!bno.begin()) {
//     Serial.println("BNO055 not detected. Check wiring or I2C address.");
//     while (1);
//   }

//   delay(1000);
//   bno.setExtCrystalUse(true);

//   Serial.println("BNO055 detected!");

//   delay(500);

//   // Make current secured position equal to zero
//   recalibrateIMU();

//   Serial.println("Type r to recalibrate again.");
// }

// void loop() {
//   if (Serial.available() > 0) {
//     char command = Serial.read();

//     if (command == 'r' || command == 'R') {
//       recalibrateIMU();
//     }
//   }

//   sensors_event_t event;
//   bno.getEvent(&event);

//   // Raw differences from starting position
//   yawAngle = angleDifference(event.orientation.x, yawZero);

//   // Flipped sign so your physical +90 becomes +90
//   pitchAngle = -angleDifference(event.orientation.y, pitchZero);

//   rollAngle = angleDifference(event.orientation.z, rollZero);

//   Serial.print("Corrected Pitch Angle: ");
//   Serial.print(pitchAngle);

//   Serial.print(" | Raw Pitch/Y: ");
//   Serial.print(event.orientation.y);

//   Serial.print(" | Raw Yaw/X: ");
//   Serial.print(event.orientation.x);

//   Serial.print(" | Raw Roll/Z: ");
//   Serial.println(event.orientation.z);

//   delay(500);
// }