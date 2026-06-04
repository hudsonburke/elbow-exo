/*
Setup():
    start serial communication
    start I2C communication
    select default IMU channel on multiplexer
    initialize the BNO055 IMU
    print instructions for user
*/

/* 
code start:
IMU 1 = wrist SC/D 2
IMU 2 = forearm SC/D 3
IMU 3 = bicep SC/D 4 
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

// EMG muscle variables
float wristEMG;
float forearmEMG;
float bicepEMG;

// IMU selection channels (multiplexer)
int WRIST = 2;
int FOREARM = 3;
int BICEP = 4;

// stores which IMU is currently selected
int selectedIMU = WRIST;   // default 0 

// time tracking
unsigned long time;
unsigned long timeReset;

// function to select multiplexer channel
void tcaselect(uint8_t channel)
{
    Wire.beginTransmission(0x70); // MUX I2C address
    Wire.write(1 << channel);     // activate only that channel
    Wire.endTransmission();
}

// IMU object
Adafruit_BNO055 bno = Adafruit_BNO055(55); // BNO055 default address

// prints sensor info for debugging
void displaySensorDetails()
{
    sensor_t sensor;
    bno.getSensor(&sensor);

    Serial.println("------------------------------------");
    Serial.print("Sensor: "); Serial.println(sensor.name);
    Serial.print("Driver Ver: "); Serial.println(sensor.version);
    Serial.print("Unique ID: "); Serial.println(sensor.sensor_id);
    Serial.println("------------------------------------");
    Serial.println("");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Orientation Sensor Test");

    Wire.begin();

    // connect to default IMU channel
    tcaselect(selectedIMU);

    if (!bno.begin())
    {
        Serial.println("No BNO055 detected. Check wiring!");
        while (1);
    }

    delay(1000);

    timeReset = millis();

    Serial.println("Enter IMU number to select:");
    Serial.println("0 = Wrist");
    Serial.println("1 = Forearm");
    Serial.println("2 = Bicep");
}

void loop()
{
// Check for user input to switch IMU
if (Serial.available())
{
    int userInput = Serial.parseInt();

    if (userInput >= 0 && userInput <= 2) // validate input
    {
        // turn user input to correct MUX channel
        if (userInput == 0) selectedIMU = WRIST;
        else if (userInput == 1) selectedIMU = FOREARM;
        else if (userInput == 2) selectedIMU = BICEP;

        tcaselect(selectedIMU); // switch MUX to correct channel

        Serial.print("Switched to IMU: ");
        if (userInput == 0) Serial.println("Wrist");
        else if (userInput == 1) Serial.println("Forearm");
        else if (userInput == 2) Serial.println("Bicep");
    }
    else
    {
        Serial.println("Invalid IMU number! Enter 0, 1, or 2.");
    }
}

    // calculate time
    time = millis() - timeReset;
    Serial.print(time);
    Serial.print(",");

    // read selected IMU
    sensors_event_t event;
    bno.getEvent(&event);

    imu::Quaternion q = bno.getQuat();

    // adjust quaternion axes
    float temp = q.x();
    q.x() = q.y();
    q.y() = temp;
    q.z() = -q.z();
    q.normalize();

    imu::Vector<3> euler;
    euler.x() = 180 / M_PI * atan2(q.w() * q.z() + q.x() * q.y(), 0.5 - q.y() * q.y() - q.z() * q.z()); // yaw
    euler.y() = 180 / M_PI * atan2(q.w() * q.x() + q.y() * q.z(), 0.5 - q.x() * q.x() - q.y() * q.y()); // roll
    euler.z() = 180 / M_PI * asin(2 * (q.w() * q.y() - q.x() * q.z()));                                  // pitch

    // print IMU orientation
    Serial.print(euler.x()); Serial.print(",");
    Serial.print(euler.y()); Serial.print(",");
    Serial.print(euler.z()); Serial.print(",");

    // read EMG sensors
    forearmEMG = analogRead(A0);
    bicepEMG   = analogRead(A1);
    wristEMG   = analogRead(A2);

    // print EMG
    Serial.print(forearmEMG); Serial.print(",");
    Serial.print(bicepEMG);   Serial.print(",");
    Serial.print(wristEMG);

    Serial.println();

    delay(100); // small delay for readability
}







