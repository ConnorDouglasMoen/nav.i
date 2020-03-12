#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

int dist = 0;
bool deviceConnected = false;

#define SERVICE_UUID        "76850745-6cd8-40c0-baa7-58465de27e5b"
#define CHARACTERISTIC_UUID "9b5a0e0f-5812-4ece-bef7-47be894bfafe"

class ServerCallbacks: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic) {
      pCharacteristic -> setValue(dist); 
    }

    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms
  // This period should be at least as long as the timing budget
  sensor.startContinuous(50);

//  Bluetooth setup
  BLEDevice::init("Navi");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE);
                      
  pCharacteristic->setCallbacks(new ServerCallbacks());
  pCharacteristic->setValue("Nav.i Time of Flight");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
//  pAdvertising->start();
  BLEDevice::startAdvertising();
  Serial.println("Waiting a Nav.i Client to connect");
}

void loop() {
    // notify changed value
   if (deviceConnected) {
        Serial.println("Device connected");
   }
  dist = sensor.read();

  Serial.print(sensor.read());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
  delay(80);
}
