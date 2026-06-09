#include <Arduino.h>
#include <Wire.h>        
 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define TCA_ADDR 0x70

Adafruit_BNO055 bno0 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno1 = Adafruit_BNO055(56, 0x28);

bool imu0_ok = false;
bool imu1_ok = false;

void tcaselect(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin();

  Serial.println("Starting dual BNO055 test...");

  tcaselect(0);
  delay(100);
  imu0_ok = bno0.begin();
  if (imu0_ok) {
    Serial.println("IMU 0 found on channel 0");
    bno0.setExtCrystalUse(true);
  } else {
    Serial.println("IMU 0 NOT found on channel 0");
  }

  tcaselect(1);
  delay(100);
  imu1_ok = bno1.begin();
  if (imu1_ok) {
    Serial.println("IMU 1 found on channel 1");
    bno1.setExtCrystalUse(true);
  } else {
    Serial.println("IMU 1 NOT found on channel 1");
  }
}

void loop() {
  Serial.println("----------");

  if (imu0_ok) {
    tcaselect(0);
    delay(10);

    sensors_event_t event0;
    bno0.getEvent(&event0);

    uint8_t sys, gyro, accel, mag;
    bno0.getCalibration(&sys, &gyro, &accel, &mag);

    Serial.print("IMU 0 | X: ");
    Serial.print(event0.orientation.x);
    Serial.print(" Y: ");
    Serial.print(event0.orientation.y);
    Serial.print(" Z: ");
    Serial.print(event0.orientation.z);

    Serial.print(" | Cal: ");
    Serial.print(sys); Serial.print(" ");
    Serial.print(gyro); Serial.print(" ");
    Serial.print(accel); Serial.print(" ");
    Serial.println(mag);
  } else {
    Serial.println("IMU 0 unavailable");
  }

  if (imu1_ok) {
    tcaselect(1);
    delay(10);

    sensors_event_t event1;
    bno1.getEvent(&event1);

    uint8_t sys, gyro, accel, mag;
    bno1.getCalibration(&sys, &gyro, &accel, &mag);

    Serial.print("IMU 1 | X: ");
    Serial.print(event1.orientation.x);
    Serial.print(" Y: ");
    Serial.print(event1.orientation.y);
    Serial.print(" Z: ");
    Serial.print(event1.orientation.z);

    Serial.print(" | Cal: ");
    Serial.print(sys); Serial.print(" ");
    Serial.print(gyro); Serial.print(" ");
    Serial.print(accel); Serial.print(" ");
    Serial.println(mag);
  } else {
    Serial.println("IMU 1 unavailable");
  }

  delay(500);
}