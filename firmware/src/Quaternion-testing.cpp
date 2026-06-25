// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>

// // =====================================================
// // USER CONFIGURATION
// // =====================================================

// #define TCA_ADDR 0x70
// #define MAX_IMUS 8
// #define NUM_IMUS 3

// // Sensor sampling period (how fast we read from IMUs)
// #define SAMPLE_PERIOD_MS 20

// // Print output period (how fast we send data to serial)
// #define PRINT_PERIOD_MS 500

// static_assert(NUM_IMUS <= MAX_IMUS, "NUM_IMUS cannot be greater than MAX_IMUS.");
// static_assert(MAX_IMUS <= 8, "One TCA9548A supports only 8 channels.");

// // =====================================================
// // BNO055 OBJECTS
// // =====================================================

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

// // =====================================================
// // RUNTIME STATE
// // =====================================================

// bool imu_ok[NUM_IMUS] = {false};
// bool have_event[NUM_IMUS] = {false};
// bool zeroed = false;
// unsigned long lastPrintTime = 0;

// // Store absolute quaternion orientation from each IMU
// imu::Quaternion current_q[NUM_IMUS];

// // Store the zero/reference quaternion for each IMU
// imu::Quaternion zero_q[NUM_IMUS];

// // Store the zeroed quaternion (relative to reference)
// imu::Quaternion zeroed_q[NUM_IMUS];

// // =====================================================
// // MULTIPLEXER CHANNEL SELECTION
// // =====================================================

// void tcaselect(uint8_t channel) {
//   if (channel > 7) return;

//   Wire.beginTransmission(TCA_ADDR);
//   Wire.write(1 << channel);
//   Wire.endTransmission();
// }

// // =====================================================
// // QUATERNION TO EULER CONVERSION
// // =====================================================
// //
// // Convert a quaternion to Euler angles (in degrees).
// // This uses the standard aerospace convention:
// //   - heading (yaw):   rotation around Z axis
// //   - roll:            rotation around X axis
// //   - pitch:           rotation around Y axis

// void quaternionToEuler(const imu::Quaternion &q, float &heading, float &roll, float &pitch) {
//   // Get Euler angles in radians from the quaternion
//   imu::Vector<3> euler = q.toEuler();
  
//   // Convert from radians to degrees
  
//   heading = euler.x() * RAD_TO_DEG;
//   roll = euler.y() * RAD_TO_DEG;
//   pitch = euler.z() * RAD_TO_DEG;
// }

// // =====================================================
// // QUATERNION NORMALIZATION
// // =====================================================
// //
// // Ensure quaternion is normalized (unit length).
// // This prevents drift and numerical errors.

// imu::Quaternion normalizeQuaternion(const imu::Quaternion &q) {
//   imu::Quaternion normalized = q;
//   normalized.normalize();
//   return normalized;
// }

// // =====================================================
// // CLEAN SERIAL PRINT HELPERS
// // =====================================================

// void printQuaternion(const imu::Quaternion &q) {
//   Serial.print("Quat(w,x,y,z): ");
//   Serial.print(q.w(), 4);
//   Serial.print(", ");
//   Serial.print(q.x(), 4);
//   Serial.print(", ");
//   Serial.print(q.y(), 4);
//   Serial.print(", ");
//   Serial.print(q.z(), 4);
// }

// void printEuler(const imu::Quaternion &q) {
//   float heading, roll, pitch;
//   quaternionToEuler(q, heading, roll, pitch);

//   Serial.print("Euler(H,R,P):  ");
//   Serial.print(heading, 2);
//   Serial.print(", ");
//   Serial.print(roll, 2);
//   Serial.print(", ");
//   Serial.print(pitch, 2);
// }

// void printQuatAndEuler(const imu::Quaternion &q) {
//   Serial.print("    ");
//   printQuaternion(q);
//   Serial.println();
//   Serial.print("    ");
//   printEuler(q);
//   Serial.println();
// }

// // =====================================================
// // READ IMU QUATERNION
// // =====================================================
// //
// // Select the multiplexer channel and read quaternion from the BNO055.
// // Returns the raw orientation quaternion.

// imu::Quaternion readQuaternionFromIMU(int imu_index) {
//   // Select this IMU's channel on the multiplexer
//   tcaselect(imu_index);
  
  
//   // Read and return the quaternion from the sensor
//   return bnos[imu_index].getQuat();
// }

// // =====================================================
// // APPLY ZERO REFERENCE
// // =====================================================
// //
// // Given a current quaternion and a zero reference,
// // compute the relative quaternion (how much it has rotated from zero).
// //
// // Formula: q_relative = q_zero_inverse * q_current
// // This tells us the rotation FROM the zero position TO the current position.

// imu::Quaternion applyZeroReference(const imu::Quaternion &current, const imu::Quaternion &zero_ref) {
//   // Conjugate of a unit quaternion is its inverse
//   imu::Quaternion zero_inverse = zero_ref.conjugate();
  
//   // Apply: relative = zero_inverse * current
//   imu::Quaternion relative = zero_inverse * current;
  
//   return normalizeQuaternion(relative);
// }

// // =====================================================
// // COMPUTE RELATIVE ORIENTATION
// // =====================================================
// //
// // Given two quaternions representing absolute orientations,
// // compute the relative orientation between them.
// //
// // This represents: "how much has IMU 2 rotated relative to IMU 1?"
// //
// // Formula: q_relative = q_imu1_inverse * q_imu2
// // This expresses the rotation FROM imu1's frame TO imu2's frame.

// imu::Quaternion computeRelativeOrientation(const imu::Quaternion &q_imu1, const imu::Quaternion &q_imu2) {
//   // Conjugate of a unit quaternion is its inverse
//   imu::Quaternion q_imu1_inverse = q_imu1.conjugate();
  
//   // Apply: relative = imu1_inverse * imu2
//   imu::Quaternion relative = q_imu2 * q_imu1_inverse ;
  
//   return normalizeQuaternion(relative);
// }

// // =====================================================
// // SERIAL COMMAND PROCESSING
// // =====================================================
// //
// // Handle user commands from the Serial Monitor.
// //
// // Commands:
// //   "zero"   - Captures current orientation as reference. Subsequent readings are relative to this.
// //   "revert" - Clears the reference. Readings revert to absolute orientations.

// void processSerialCommands() {
//   if (!Serial.available()) return;

//   String line = Serial.readStringUntil('\n');
//   line.trim();
//   line.toLowerCase();

//   if (line == "z") {
//     bool any_zeroed = false;

//     // Save current orientation as reference for each active IMU
//     for (int i = 0; i < NUM_IMUS; i++) {
//       if (imu_ok[i]) {
//         zero_q[i] = readQuaternionFromIMU(i);
//         any_zeroed = true;
//       }
//     }

//     if (any_zeroed) {
//       zeroed = true;
//       Serial.println(">>> Zero reference captured. All readings are now relative to this position.");
//     } else {
//       Serial.println(">>> ERROR: Cannot zero - no IMU available.");
//     }
//   }
//   else if (line == "revert") {
//     zeroed = false;
//     Serial.println(">>> Reverted to absolute orientations.");
//   }
// }

// // =====================================================
// // SETUP
// // =====================================================

// void setup() {
//   Serial.begin(115200);
//   delay(2000);

//   Wire.begin();
//   Wire.setClock(400000);

//   Serial.println("\n=== Quaternion-Based IMU Orientation Test ===\n");

//   // Try to initialize each IMU
//   for (int i = 0; i < NUM_IMUS; i++) {
//     tcaselect(i);
//     delay(100);

//     imu_ok[i] = bnos[i].begin();

//     if (imu_ok[i]) {
//       Serial.print("✓ IMU ");
//       Serial.print(i);
//       Serial.println(" initialized");
      
//       bnos[i].setExtCrystalUse(true);
//     } else {
//       Serial.print("✗ IMU ");
//       Serial.print(i);
//       Serial.println(" NOT FOUND");
//     }
//   }

//   Serial.println("\nConfiguration:");
//   Serial.print("  Sample rate: ");
//   Serial.print(SAMPLE_PERIOD_MS);
//   Serial.println(" ms");
//   Serial.print("  Print rate: ");
//   Serial.print(PRINT_PERIOD_MS);
//   Serial.println(" ms");
//   Serial.println("\nCommands:");
//   Serial.println("  Type 'z' to set reference orientation\n");
// }

// // =====================================================
// // MAIN LOOP
// // =====================================================
// //
// // Sampling Phase:
// //   1. Read raw quaternions from all IMUs at SAMPLE_PERIOD_MS rate
// //   2. Apply zero reference if enabled
// //
// // Print Phase (independent timing):
// //   1. Only when PRINT_PERIOD_MS has elapsed
// //   2. Convert quaternions to Euler angles
// //   3. Calculate and print relative angles between IMUs
// //   4. Print calibration status

// void loop() {
//   // =====================================================
//   // PHASE 1: CHECK FOR SERIAL COMMANDS
//   // =====================================================
  
//   processSerialCommands();

//   // =====================================================
//   // PHASE 2: READ ALL IMU QUATERNIONS (SAMPLING)
//   // =====================================================
//   //
//   // This happens every SAMPLE_PERIOD_MS regardless of print timing.
//   // This ensures we sample at a consistent, fast rate.

//   for (int i = 0; i < NUM_IMUS; i++) {
//     have_event[i] = false;

//     if (imu_ok[i]) {
//       // Read raw quaternion from this IMU
//       current_q[i] = readQuaternionFromIMU(i);
//       have_event[i] = true;

//       // Apply zero reference if it's been set
//       if (zeroed) {
//         zeroed_q[i] = applyZeroReference(current_q[i], zero_q[i]);
//       } else {
//         zeroed_q[i] = current_q[i];
//       }
//     }
//   }

//   // =====================================================
//   // PHASE 3: PRINT OUTPUT (INDEPENDENT TIMING)
//   // =====================================================
//   //
//   // Only print if PRINT_PERIOD_MS time has elapsed.
//   // This decouples printing from sampling.

//   unsigned long currentTime = millis();
//   if (currentTime - lastPrintTime >= PRINT_PERIOD_MS) {
//     lastPrintTime = currentTime;

//     Serial.println("\n====================");
//     Serial.print("Mode: ");
//     Serial.println(zeroed ? "ZEROED" : "ABSOLUTE");

//     // Print individual IMU orientations
//     Serial.println("\nORIENTATION");
//     for (int i = 0; i < NUM_IMUS; i++) {
//       if (imu_ok[i] && have_event[i]) {
//         // Get calibration status
//         uint8_t sys, gyro, accel, mag;
//         tcaselect(i);
//         bnos[i].getCalibration(&sys, &gyro, &accel, &mag);

//         Serial.print("  IMU ");
//         Serial.print(i);
//         Serial.print("    Cal(S,G,A,M): ");
//         Serial.print(sys);
//         Serial.print("/");
//         Serial.print(gyro);
//         Serial.print("/");
//         Serial.print(accel);
//         Serial.print("/");
//         Serial.println(mag);

//         printQuatAndEuler(zeroed_q[i]);
//       } else if (imu_ok[i]) {
//         Serial.print("  IMU ");
//         Serial.print(i);
//         Serial.println("    no data this cycle");
//       } else {
//         Serial.print("  IMU ");
//         Serial.print(i);
//         Serial.println("    not found");
//       }
//     }

//     // =====================================================
//     // CALCULATE AND PRINT RELATIVE ANGLES
//     // =====================================================
//     //
//     // For consecutive IMU pairs, calculate the relative rotation
//     // between them. This tells us the joint angle change.
//     //
//     // For 3 IMUs:
//     //   - IMU1 relative to IMU0 = upper joint
//     //   - IMU2 relative to IMU1 = lower joint

//     Serial.println("\nDIFFERENCE / RELATIVE ROTATION");
//     for (int i = 1; i < NUM_IMUS; i++) {
//       if (have_event[i] && have_event[i - 1]) {
//         // Difference from IMU i-1 to IMU i.
//         imu::Quaternion rel_q = computeRelativeOrientation(zeroed_q[i - 1], zeroed_q[i]);

//         Serial.print("  Joint ");
//         Serial.print(i);
//         Serial.print("-");
//         Serial.println(i - 1);

//         printQuatAndEuler(rel_q);
//       } else {
//         Serial.print("  Joint ");
//         Serial.print(i);
//         Serial.print("-");
//         Serial.print(i - 1);
//         Serial.println("    unavailable");
//       }
//     }
//   }

//   // =====================================================
//   // PHASE 4: WAIT FOR NEXT SAMPLE
//   // =====================================================
//   //
//   // Sleep for SAMPLE_PERIOD_MS before reading sensors again.
//   // This maintains consistent sampling rate.

//   delay(SAMPLE_PERIOD_MS);
// }