void setup() {
  Serial.begin(9600); // Match this in your Python script
}

void loop() {
  int emg = analogRead(A0);   // Read EMG value from A0
  Serial.println(emg);        // Send to Python over USB
  delay(10);                  // 100 Hz sampling rate
}
