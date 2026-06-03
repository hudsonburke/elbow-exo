// Motor Driver Connections
int PWMPin = 9;
int DirPin1 = 4;
int DirPin2 = 2;

// Encoder A and B Connections
const byte interruptPinA = 5;
const byte interruptPinB = 6;
volatile long EncoderCount = 0;

float kp = 0.02;
float ki = 0.00015;
float kd = 0;
float RPM_d, RPM, RPM_max = 230;
unsigned long t, t_prev = 0;
float Theta, Theta_prev = 0;
int dt;
float Vmax = 6, Vmin = -6, V = 0.1;
float e, e_prev = 0, inte, inte_prev = 0;
#define pi 3.1416

void ISR_EncoderA() {
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);
  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount++;
    } else {
      EncoderCount--;
    }
  } else {
    if (PinA == HIGH) {
      EncoderCount--;
    } else {
      EncoderCount++;
    }
  }
}

void ISR_EncoderB() {
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);
  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCount--;
    } else {
      EncoderCount++;
    }
  } else {
    if (PinB == HIGH) {
      EncoderCount++;
    } else {
      EncoderCount--;
    }
  }
}

// Implementing the sign function
float sign(float x) {
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0;
  }
}

void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  if (V > 0) {
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
  } else if (V < 0) {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
  } else {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
  }
  analogWrite(PWMPin, PWMval);
}

// Timer interrupt function (for ESP32)
void IRAM_ATTR TimerISR() {
  EncoderCount++; // Increment count for each interrupt
  Serial.print(EncoderCount);
  Serial.print(" \t");
}

void setup() {
  Serial.begin(115200);
  
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);

  // Set up Timer (ESP32 Specific Timer Setup)
  timerBegin(0, 80, true); // Timer 0, 80 prescaler
  timerAttachInterrupt(0, TimerISR, true); // Attach ISR to timer 0
  timerAlarmWrite(0, 1000000, true); // Set timer interval to 1 second
  timerAlarmEnable(0); // Enable the timer interrupt
}

void loop() {
  if (t > t_prev) {
    t = millis();
    Theta = EncoderCount / 900.0;
    dt = (t - t_prev);
    RPM_d = RPM_max * (sin(2 * pi * 0.005 * t / 1000.0)) * sign(sin(2 * pi * 0.05 * t / 1000.0));
    if (t / 1000.0 > 100) {
      RPM_d = 0;
    }
    RPM = (Theta - Theta_prev) / (dt / 1000.0) * 60;
    e = RPM_d - RPM;
    inte = inte_prev + (dt * (e + e_prev) / 2);
    V = kp * e + ki * inte + (kd * (e - e_prev) / dt);
    
    if (V > Vmax) {
      V = Vmax;
      inte = inte_prev;
    }
    if (V < Vmin) {
      V = Vmin;
      inte = inte_prev;
    }

    WriteDriverVoltage(V, Vmax);

    Serial.print(RPM_d); Serial.print(" \t");
    Serial.print(RPM); Serial.print(" \t ");
    Serial.print(V); Serial.print("\t  ");
    Serial.print(e); Serial.println("  ");

    Theta_prev = Theta;
    t_prev = t;
    inte_prev = inte;
    e_prev = e;
  }
}
