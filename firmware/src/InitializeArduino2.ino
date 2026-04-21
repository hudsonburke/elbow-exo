#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <ArduinoJson.h>

#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID             "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

BLECharacteristic *pRxCharacteristic;

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();

        if (rxValue.length() > 0) {
            Serial.print("Raw Received: ");
            Serial.println(rxValue.c_str());

            // Create JSON document
            StaticJsonDocument<200> doc;

            // Parse JSON
            DeserializationError error = deserializeJson(doc, rxValue);

            if (error) {
                Serial.print("JSON Parse Failed: ");
                Serial.println(error.c_str());
                return;
            }

            // Extract values
            int mode = doc["mode"];
            int direction = doc["direction"];

            // PI goes here?
        }
    }
};

void setup() {
    Serial.begin(115200);

    BLEDevice::init("Nano_ESP32");  // THIS must match your Python code
    BLEServer *pServer = BLEDevice::createServer();

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pRxCharacteristic = pService->createCharacteristic(
        RX_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("BLE advertising as Nano_ESP32...");
}

void loop() {
    delay(10);
}