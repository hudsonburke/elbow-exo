#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ----------------------
// IMU
// ----------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float targetPitch90 = 0.0;
float targetPitch180 = 0.0;

bool calibrated90 = false;
bool calibrated180 = false;

int trialNumber = 0;

const float TARGET_TOLERANCE_DEG = 3.0;

// ----------------------
// Motor 2 pins
// ----------------------
#define M2_IN1 9
#define M2_IN2 10
#define M2_ENCA 3
#define M2_ENCB 7

volatile long pos2 = 0;

const int FORWARD = 1;
const int REVERSE = -1;

const int MOTOR_PWM = 180;

const int PULL_DIRECTION = FORWARD;
const int RETURN_DIRECTION = REVERSE;

// ----------------------
// Encoder conversion
// ----------------------
const float ENCODER_COUNTS_PER_MOTOR_REV = 64.0;
const float GEAR_RATIO = 270.0;
const float COUNTS_PER_OUTPUT_REV = ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO; // 17280
const float COUNTS_PER_DEGREE = COUNTS_PER_OUTPUT_REV / 360.0;                 // 48
const float EXPECTED_COUNTS_90_DEG = COUNTS_PER_DEGREE * 90.0;                 // 4320

// ----------------------
// Trial settings
// ----------------------
const unsigned long HOLD_TIME_MS = 5000;

// ----------------------
// Trial state machine
// ----------------------
enum TrialState {
  IDLE,
  PULL_TO_90,
  HOLD_AT_90,
  RETURN_TO_180
};

TrialState trialState = IDLE;

long startCount = 0;
long countAt90 = 0;
long finalCount = 0;

unsigned long trialStartTime = 0;
unsigned long reached90Time = 0;
unsigned long holdStartTime = 0;
unsigned long returned180Time = 0;

// ----------------------
// Function declarations
// ----------------------
void readEncoder2();
long getMotor2Position();
void resetMotor2Encoder();
float getOutputDegrees();

void setMotor2(int dir, int pwmVal);
void stopMotor2();

float getPitch();
float errorFromTarget(float targetPitch);

void startTrial();
void reached90();
void returned180();
void emergencyStop();

void printStatus(float pitch);

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring.");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  pinMode(M2_ENCA, INPUT_PULLUP);
  pinMode(M2_ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M2_ENCA), readEncoder2, RISING);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  stopMotor2();
  resetMotor2Encoder();

  Serial.println("System ready.");
  Serial.println("Commands:");
  Serial.println("r = save current IMU position as 90 degrees");
  Serial.println("s = save current IMU position as 180/start");
  Serial.println("z = reset motor encoder");
  Serial.println("g = start trial");
  Serial.println("0 = emergency stop");

  Serial.print("Counts per output revolution: ");
  Serial.println(COUNTS_PER_OUTPUT_REV);

  Serial.print("Counts per degree: ");
  Serial.println(COUNTS_PER_DEGREE);

  Serial.print("Expected counts for 90 deg: ");
  Serial.println(EXPECTED_COUNTS_90_DEG);
}

void loop() {
  float pitch = getPitch();

  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'r') {
      targetPitch90 = pitch;
      calibrated90 = true;
      Serial.println("Saved 90 degree target.");
      Serial.print("targetPitch90:");
      Serial.println(targetPitch90);
    }

    else if (cmd == 's') {
      targetPitch180 = pitch;
      calibrated180 = true;
      Serial.println("Saved 180/start target.");
      Serial.print("targetPitch180:");
      Serial.println(targetPitch180);
    }

    else if (cmd == 'z') {
      resetMotor2Encoder();
      Serial.println("Encoder reset to 0.");
    }

    else if (cmd == 'g') {
      startTrial();
    }

    else if (cmd == '0') {
      emergencyStop();
    }
  }

  if (trialState == PULL_TO_90) {
    float error90 = errorFromTarget(targetPitch90);

    if (abs(error90) <= TARGET_TOLERANCE_DEG) {
      reached90();
    } else {
      setMotor2(PULL_DIRECTION, MOTOR_PWM);
    }
  }

  else if (trialState == HOLD_AT_90) {
    stopMotor2();

    if (millis() - holdStartTime >= HOLD_TIME_MS) {
      Serial.println("Hold complete. Returning to 180/start.");
      trialState = RETURN_TO_180;
    }
  }

  else if (trialState == RETURN_TO_180) {
    float error180 = errorFromTarget(targetPitch180);

    if (abs(error180) <= TARGET_TOLERANCE_DEG) {
      returned180();
    } else {
      setMotor2(RETURN_DIRECTION, MOTOR_PWM);
    }
  }

  else {
    stopMotor2();
  }

  printStatus(pitch);
  delay(50);
}

// ----------------------
// Encoder
// ----------------------
void readEncoder2() {
  int b = digitalRead(M2_ENCB);

  if (b == HIGH) {
    pos2++;
  } else {
    pos2--;
  }
}

long getMotor2Position() {
  long currentPos;

  noInterrupts();
  currentPos = pos2;
  interrupts();

  return currentPos;
}

void resetMotor2Encoder() {
  noInterrupts();
  pos2 = 0;
  interrupts();
}

float getOutputDegrees() {
  return getMotor2Position() / COUNTS_PER_DEGREE;
}

// ----------------------
// Motor
// ----------------------
void setMotor2(int dir, int pwmVal) {
  pwmVal = constrain(pwmVal, 0, 255);

  if (dir == FORWARD) {
    analogWrite(M2_IN1, pwmVal);
    analogWrite(M2_IN2, 0);
  }
  else if (dir == REVERSE) {
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, pwmVal);
  }
  else {
    stopMotor2();
  }
}

void stopMotor2() {
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);
}

// ----------------------
// IMU
// ----------------------
float getPitch() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.y();
}

float errorFromTarget(float targetPitch) {
  float pitch = getPitch();
  return pitch - targetPitch;
}

// ----------------------
// Trial functions
// ----------------------
void startTrial() {
  if (!calibrated90 || !calibrated180) {
    Serial.println("ERROR: Save both targets first.");
    Serial.println("Use r at 90 degrees and s at 180/start.");
    return;
  }

  resetMotor2Encoder();

  trialNumber++;

  startCount = getMotor2Position();
  trialStartTime = millis();

  trialState = PULL_TO_90;

  Serial.println("---------------------------");
  Serial.print("TRIAL_START:");
  Serial.println(trialNumber);
  Serial.print("startCount:");
  Serial.println(startCount);
}

void reached90() {
  stopMotor2();

  countAt90 = getMotor2Position();
  reached90Time = millis();
  holdStartTime = millis();

  trialState = HOLD_AT_90;

  long countsTo90 = countAt90 - startCount;
  float outputDegTo90 = countsTo90 / COUNTS_PER_DEGREE;

  Serial.print("TRIAL:");
  Serial.println(trialNumber);
  Serial.println("REACHED_90");

  Serial.print("countAt90:");
  Serial.println(countAt90);

  Serial.print("countsTo90:");
  Serial.println(countsTo90);

  Serial.print("estimatedOutputDegreesTo90:");
  Serial.println(outputDegTo90);

  Serial.print("expectedCountsFor90Deg:");
  Serial.println(EXPECTED_COUNTS_90_DEG);

  Serial.print("timeTo90_s:");
  Serial.println((reached90Time - trialStartTime) / 1000.0);

  Serial.println("Holding for 5 seconds.");
}

void returned180() {
  stopMotor2();

  finalCount = getMotor2Position();
  returned180Time = millis();

  long countsTo90 = countAt90 - startCount;
  long returnCounts = finalCount - countAt90;
  long totalCounts = finalCount - startCount;

  float outputDegreesTo90 = countsTo90 / COUNTS_PER_DEGREE;
  float returnDegrees = returnCounts / COUNTS_PER_DEGREE;
  float totalDegrees = totalCounts / COUNTS_PER_DEGREE;

  float timeTo90 = (reached90Time - trialStartTime) / 1000.0;
  float returnTime = (returned180Time - reached90Time) / 1000.0;
  float totalTime = (returned180Time - trialStartTime) / 1000.0;

  Serial.println("---------------------------");
  Serial.print("TRIAL_COMPLETE:");
  Serial.println(trialNumber);

  Serial.print("trialNumber:");
  Serial.println(trialNumber);

  Serial.print("startCount:");
  Serial.println(startCount);

  Serial.print("countAt90:");
  Serial.println(countAt90);

  Serial.print("finalCount:");
  Serial.println(finalCount);

  Serial.print("countsTo90:");
  Serial.println(countsTo90);

  Serial.print("returnCounts:");
  Serial.println(returnCounts);

  Serial.print("totalCounts:");
  Serial.println(totalCounts);

  Serial.print("outputDegreesTo90:");
  Serial.println(outputDegreesTo90);

  Serial.print("returnDegrees:");
  Serial.println(returnDegrees);

  Serial.print("totalDegrees:");
  Serial.println(totalDegrees);

  Serial.print("expectedCountsFor90Deg:");
  Serial.println(EXPECTED_COUNTS_90_DEG);

  Serial.print("timeTo90_s:");
  Serial.println(timeTo90);

  Serial.print("returnTime_s:");
  Serial.println(returnTime);

  Serial.print("totalTrialTime_s:");
  Serial.println(totalTime);

  Serial.println("CSV_ROW:");
  Serial.print(trialNumber);
  Serial.print(",");
  Serial.print(startCount);
  Serial.print(",");
  Serial.print(countAt90);
  Serial.print(",");
  Serial.print(finalCount);
  Serial.print(",");
  Serial.print(countsTo90);
  Serial.print(",");
  Serial.print(returnCounts);
  Serial.print(",");
  Serial.print(totalCounts);
  Serial.print(",");
  Serial.print(outputDegreesTo90);
  Serial.print(",");
  Serial.print(returnDegrees);
  Serial.print(",");
  Serial.print(totalDegrees);
  Serial.print(",");
  Serial.print(timeTo90);
  Serial.print(",");
  Serial.print(returnTime);
  Serial.print(",");
  Serial.println(totalTime);

  Serial.println("---------------------------");
  Serial.println("Send g for next trial.");

  trialState = IDLE;
}

void emergencyStop() {
  stopMotor2();
  trialState = IDLE;
  Serial.println("EMERGENCY_STOP");
}

// ----------------------
// Serial status
// ----------------------
void printStatus(float pitch) {
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint < 200) {
    return;
  }

  lastPrint = millis();

  Serial.print("pitch:");
  Serial.print(pitch);

  Serial.print(" err90:");
  if (calibrated90) {
    Serial.print(pitch - targetPitch90);
  } else {
    Serial.print("NA");
  }

  Serial.print(" err180:");
  if (calibrated180) {
    Serial.print(pitch - targetPitch180);
  } else {
    Serial.print("NA");
  }

  Serial.print(" counts:");
  Serial.print(getMotor2Position());

  Serial.print(" outputDeg:");
  Serial.print(getOutputDegrees());

  Serial.print(" state:");
  Serial.println(trialState);
}