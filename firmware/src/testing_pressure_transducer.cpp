#include <Arduino.h>

const int pressurePin = A0;

const float sensorMaxPSI = 80.0;
const float adcZeroPSI = 129.0;
const float adcMaxPSI = 921.6;    // 4.5V on 5V Arduino

void setup() {
  Serial.begin(9600);
  Serial.println("0-80 PSI Pressure Sensor Test");
}

void loop() {
  int adc = analogRead(pressurePin);

  float pressurePSI = ((adc - adcZeroPSI) * sensorMaxPSI) /
                      (adcMaxPSI - adcZeroPSI);

  if (pressurePSI < 0) {
    pressurePSI = 0;
  }

  float pressureKPa = pressurePSI * 6.89476;

  Serial.print("ADC: ");
  Serial.print(adc);

  Serial.print(" | Pressure: ");
  Serial.print(pressurePSI, 2);
  Serial.print(" PSI");

  Serial.print(" | ");
  Serial.print(pressureKPa, 2);
  Serial.println(" kPa");

  delay(250);
}