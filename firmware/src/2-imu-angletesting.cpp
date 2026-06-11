#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// =====================================================
// USER CONFIGURATION
// =====================================================

// I2C address for the TCA9548A multiplexer
#define TCA_ADDR 0x70

// MAX_IMUS is the maximum number of IMUs this program is prepared to handle.
// The TCA9548A has 8 channels
#define MAX_IMUS 8

// NUM_IMUS is how many IMUs you are actually using right now.
#define NUM_IMUS 3

// Time between printed readings in milliseconds.
// 500 ms = 2 readings per second.
// For actual motion tracking later, you may want 20-50 ms instead.
#define SAMPLE_PERIOD_MS 500

// Safety check at compile time.
// If you accidentally set NUM_IMUS above 8, the code will stop compiling
// with a useful error instead of failing in a confusing way.
static_assert(NUM_IMUS <= MAX_IMUS, "NUM_IMUS cannot be greater than MAX_IMUS.");
static_assert(MAX_IMUS <= 8, "One TCA9548A multiplexer only supports 8 channels.");


// =====================================================
// IMU OBJECTS
// =====================================================
//
// This creates 8 possible BNO055 sensor objects.


Adafruit_BNO055 bnos[MAX_IMUS] = {
  Adafruit_BNO055(0, 0x28),
  Adafruit_BNO055(1, 0x28),
  Adafruit_BNO055(2, 0x28),
  Adafruit_BNO055(3, 0x28),
  Adafruit_BNO055(4, 0x28),
  Adafruit_BNO055(5, 0x28),
  Adafruit_BNO055(6, 0x28),
  Adafruit_BNO055(7, 0x28)
};


// =====================================================
// RUNTIME STATE VARIABLES
// =====================================================
//
// These arrays store information for each active IMU.
//
// Index i corresponds to IMU i on multiplexer channel i.


// imu_ok[i] tells us whether IMU i was detected successfully during setup.
// true  = sensor was found and can be read
// false = sensor was not found
bool imu_ok[NUM_IMUS] = {false};

// have_event[i] tells us whether we successfully read orientation data
// from IMU i during the current loop cycle.
bool have_event[NUM_IMUS] = {false};

// zeroed becomes true after the user types "zero" into Serial Monitor.
// Before zeroing, the code prints absolute Euler angles.
// After zeroing, the code prints angle changes relative to the zero position.
bool zeroed = false;

// These store the zero/reference orientation for each IMU.
// When the user types "zero", the current X/Y/Z angles are saved here.
float zero_x[NUM_IMUS] = {0};
float zero_y[NUM_IMUS] = {0};
float zero_z[NUM_IMUS] = {0};

// These store the most recent angle readings for each IMU.
// If zeroed == false, these are absolute Euler angles from the BNO055.
// If zeroed == true, these are relative to the saved zero position.
float x_angle[NUM_IMUS] = {0};
float y_angle[NUM_IMUS] = {0};
float z_angle[NUM_IMUS] = {0};


// =====================================================
// SELECT MULTIPLEXER CHANNEL
// =====================================================
//
// The Arduino is connected to the TCA9548A multiplexer.
// Each IMU is connected to one channel of the multiplexer.
//
// To talk to a specific IMU, we first tell the multiplexer which
// channel to connect.
//
// Example:
// tcaselect(0) connects the Arduino to SD0/SC0.
// tcaselect(1) connects the Arduino to SD1/SC1.
// tcaselect(2) connects the Arduino to SD2/SC2.
//
// After selecting a channel, normal BNO055 commands go only to the
// sensor on that channel.

void tcaselect(uint8_t channel) {
  // The TCA9548A only has channels 0 through 7.
  // If an invalid channel is requested, do nothing.
  if (channel > 7) return;

  // Start communicating with the multiplexer itself at address 0x70.
  Wire.beginTransmission(TCA_ADDR);

  // This sends a bitmask telling the multiplexer which channel to enable.
  //
  // 1 << 0 = 00000001, selects channel 0
  // 1 << 1 = 00000010, selects channel 1
  // 1 << 2 = 00000100, selects channel 2
  //
  // Only one channel is selected at a time in this code.
  Wire.write(1 << channel);

  // Finish sending the command.
  Wire.endTransmission();
}


// =====================================================
// ANGLE WRAPPING
// =====================================================
//
// Euler angles wrap around.
// For example, heading/yaw can jump from 359 degrees to 0 degrees.
//
// Without correction:
// 1 - 359 = -358 degrees
//
// But physically, 1 degree and 359 degrees are only 2 degrees apart.
//
// This function returns the shortest difference between two angles.
// The output is always between -180 and +180 degrees.

float wrappedAngleDifference(float a, float b) {
  float diff = a - b;

  while (diff > 180.0f) {
    diff -= 360.0f;
  }

  while (diff <= -180.0f) {
    diff += 360.0f;
  }

  return diff;
}

// Used when zeroing.
// It compares the current angle to the stored zero/reference angle.
float applyZeroedAngle(float current, float zeroOffset) {
  return wrappedAngleDifference(current, zeroOffset);
}


// =====================================================
// SERIAL COMMANDS
// =====================================================
//
// This function checks whether the user typed anything into Serial Monitor.
//
// Current command:
//
// zero
//
// If the user types "zero", the current orientation of every detected IMU
// becomes the reference position.


void processSerialCommands() {
  // If nothing has been typed into Serial Monitor, leave immediately.
  if (!Serial.available()) return;

  // Read the user's command up to the newline character.
  String line = Serial.readStringUntil('\n');

  // Remove spaces/newlines from the beginning and end.
  line.trim();

  // Convert to lowercase so "ZERO", "Zero", and "zero" all work.
  line.toLowerCase();

  // If the user typed "zero", store current IMU angles as references.
  if (line == "zero") {
    bool any = false;

    // Loop through every IMU currently configured by NUM_IMUS.
    for (int i = 0; i < NUM_IMUS; i++) {
      // Only try to zero sensors that were found during setup.
      if (imu_ok[i]) {
        // Select the multiplexer channel for this IMU.
        tcaselect(i);
        delay(10);

        // Create a temporary event variable to hold sensor data.
        sensors_event_t event;

        // Read the current orientation from this BNO055.
        bnos[i].getEvent(&event);

        // Save current Euler angles as the zero/reference position.
        zero_x[i] = event.orientation.x;
        zero_y[i] = event.orientation.y;
        zero_z[i] = event.orientation.z;

        any = true;
      }
    }

    // If at least one IMU was successfully zeroed, enable zeroed mode.
    if (any) {
      zeroed = true;
      Serial.println("Zero reference captured.");
    } else {
      Serial.println("Cannot zero: no IMU available.");
    }
  }
}


// =====================================================
// SETUP
// =====================================================
//
// setup() runs once when the board starts or resets.
//
// Main tasks:
// 1. Start Serial Monitor output.
// 2. Start I2C communication.
// 3. Try to find each IMU on its multiplexer channel.
// 4. Store which IMUs are available.

void setup() {
  // Start serial communication at 115200 baud.
  // Make sure PlatformIO Serial Monitor is also set to 115200.
  Serial.begin(115200);

  // Give the USB serial connection time to open.
  delay(2000);

  // Start the Arduino's I2C bus.
  // On the Nano ESP32, this uses the default SDA/SCL pins.
  Wire.begin();

  // Set I2C speed to 400 kHz.
  // Standard I2C is often 100 kHz.
  // 400 kHz is faster and usually works well for short sensor wiring.
  // If readings become unstable, comment this line out.
  Wire.setClock(400000);

  Serial.println("Starting variable-number BNO055 test...");

  // Try to initialize every IMU from channel 0 up to NUM_IMUS - 1.
  for (int i = 0; i < NUM_IMUS; i++) {
    // Select the multiplexer channel for IMU i.
    tcaselect(i);

    // Small delay gives the mux/sensor time after switching channels.
    delay(100);

    // Try to start communication with the BNO055 on this channel.
    // If begin() returns true, the sensor was found.
    // If begin() returns false, the sensor was not detected.
    imu_ok[i] = bnos[i].begin();

    if (imu_ok[i]) {
      Serial.print("IMU ");
      Serial.print(i);
      Serial.print(" found on channel ");
      Serial.println(i);

      // Tell the BNO055 to use its external crystal oscillator.
      // This usually improves timing and orientation stability.
      bnos[i].setExtCrystalUse(true);
    } else {
      Serial.print("IMU ");
      Serial.print(i);
      Serial.print(" NOT found on channel ");
      Serial.println(i);
    }
  }

  Serial.println("Type 'zero' in Serial Monitor to zero all detected IMUs.");
}


// =====================================================
// MAIN LOOP
// =====================================================
//
// loop() runs repeatedly forever.
//
// Each loop does this:
// 1. Check whether the user typed "zero".
// 2. Read Euler angles from every detected IMU.
// 3. Print each IMU's angles and calibration status.
// 4. Print sequential relative angle differences.
// 5. Wait SAMPLE_PERIOD_MS before repeating.

void loop() {
  // Check whether the user typed a serial command.
  processSerialCommands();

  Serial.println("----------");

  // =====================================================
  // READ ALL ACTIVE IMUS
  // =====================================================
  //
  // This loop reads each IMU one at a time.
  //
  // Because all BNO055s have the same I2C address, we must:
  // 1. Select channel i on the multiplexer.
  // 2. Read the BNO055 on that channel.
  // 3. Move to the next channel.
  //
  // The readings are sequential, not perfectly simultaneous,
  // but the time delay is usually small enough for basic limb tracking.

  for (int i = 0; i < NUM_IMUS; i++) {
    // Reset this flag at the start of each loop cycle.
    // It will become true only if this IMU is successfully read.
    have_event[i] = false;

    // Only read sensors that were detected during setup.
    if (imu_ok[i]) {
      // Connect Arduino SDA/SCL to this IMU's mux channel.
      tcaselect(i);
      delay(10);

      // sensors_event_t is an Adafruit data container.
      // For this library call, it stores orientation data.
      sensors_event_t event;

      // Read fused orientation from the BNO055.
      // The BNO055 internally combines accelerometer, gyro, and magnetometer
      // data to estimate orientation.
      bnos[i].getEvent(&event);

      // Mark that this IMU produced data during this loop.
      have_event[i] = true;

      // Store Euler angles.
      //
      // For the Adafruit BNO055 library:
      // orientation.x = heading/yaw, usually 0 to 360 degrees
      // orientation.y = roll, usually -180 to +180 degrees
      // orientation.z = pitch, often around -90 to +90 degrees
      //
      // These are easier to understand than quaternions, but less robust
      // for advanced joint-angle calculations.
      x_angle[i] = event.orientation.x;
      y_angle[i] = event.orientation.y;
      z_angle[i] = event.orientation.z;

      // If the user has typed "zero", convert absolute angles into
      // angles relative to the saved zero/reference orientation.
      if (zeroed) {
        x_angle[i] = applyZeroedAngle(x_angle[i], zero_x[i]);
        y_angle[i] = applyZeroedAngle(y_angle[i], zero_y[i]);
        z_angle[i] = applyZeroedAngle(z_angle[i], zero_z[i]);
      }

      // Read calibration status.
      //
      // Each value ranges from 0 to 3:
      // 0 = not calibrated
      // 1 = low calibration
      // 2 = partially calibrated
      // 3 = fully calibrated
      //
      // sys   = overall system calibration
      // gyro  = gyroscope calibration
      // accel = accelerometer calibration
      // mag   = magnetometer calibration
      uint8_t sys, gyro, accel, mag;
      bnos[i].getCalibration(&sys, &gyro, &accel, &mag);

      // Print this IMU's Euler angles.
      Serial.print("IMU ");
      Serial.print(i);
      Serial.print(" | X: ");
      Serial.print(x_angle[i]);
      Serial.print(" Y: ");
      Serial.print(y_angle[i]);
      Serial.print(" Z: ");
      Serial.print(z_angle[i]);

      // Print this IMU's calibration status.
      Serial.print(" | Cal: ");
      Serial.print(sys);
      Serial.print(" ");
      Serial.print(gyro);
      Serial.print(" ");
      Serial.print(accel);
      Serial.print(" ");
      Serial.println(mag);
    } else {
      // If setup did not find this IMU, report it instead of trying to read it.
      Serial.print("IMU ");
      Serial.print(i);
      Serial.println(" unavailable");
    }
  }

  // =====================================================
  // SEQUENTIAL RELATIVE ANGLE DIFFERENCES
  // =====================================================
  //
  // This section compares neighboring IMUs.
  //
  // For 2 IMUs:
  //   prints difference 1 - 0
  //
  // For 3 IMUs:
  //   prints difference 1 - 0
  //   prints difference 2 - 1
  // For research-quality 3D joint angles, relative quaternions are usually better.

  for (int i = 1; i < NUM_IMUS; i++) {
    // Only compare IMUs if both produced valid readings this loop.
    if (have_event[i] && have_event[i - 1]) {
      // Compare IMU i against the previous IMU.
      //
      // Example:
      // i = 1 compares IMU 1 - IMU 0.
      // i = 2 compares IMU 2 - IMU 1.
      float yawDiff = wrappedAngleDifference(x_angle[i], x_angle[i - 1]);
      float rollDiff = wrappedAngleDifference(y_angle[i], y_angle[i - 1]);
      float pitchDiff = wrappedAngleDifference(z_angle[i], z_angle[i - 1]);

      // Print relative angle differences.
      Serial.print("Relative Angle Change ");
      Serial.print(i);
      Serial.print("-");
      Serial.print(i - 1);
      Serial.print(": ");

      Serial.print("yaw=");
      Serial.print(yawDiff);
      Serial.print(" deg, ");

      Serial.print("roll=");
      Serial.print(rollDiff);
      Serial.print(" deg, ");

      Serial.print("pitch=");
      Serial.print(pitchDiff);
      Serial.println(" deg");
    }
  }

  // Wait before the next full read/print cycle.
  delay(SAMPLE_PERIOD_MS);
}