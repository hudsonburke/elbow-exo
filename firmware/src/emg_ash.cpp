#include <Arduino.h>
/*
  Myoware Muscle Sensor Test
  myoware-test-plotter.ino
  Demo of Myoware Muscle Sensor
  Output to Serial Monitor
  Use battery-powered computer or USB isolator for safety!
  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/

// Connection to Myoware sensor
#define SENSOR_PIN 0

// Integer for sensor value
int sensorValue;


void setup() {
  
  // Set up serial port
  Serial.begin(9600);
}

void loop() {
 
  // Read sensor value
  sensorValue = analogRead(SENSOR_PIN);
  
  // Add "fake" plots to stabilize Y axis
  Serial.print(0);
  Serial.print(" ");
  Serial.print(1000); // To freeze the upper limit
  Serial.print(" ");
  
  // Print value to Serial Monitor
  Serial.println(sensorValue);
}
