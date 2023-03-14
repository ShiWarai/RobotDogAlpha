#include "remote_debug.hpp"

#define SERVICE_UUID_OTA              "c8659210-af91-4ad3-a995-a58d6fd26145"
#define CHARACTERISTIC_UUID_MOTOR0      "c8659212-af91-4ad3-a995-a58d6fd26145"

void RemoteDebug::loop()
{
    BLE bt;

    bt.begin();

    while(1)
        vTaskDelay(1);
}

bool BLE::begin() {

    const char* localName = "RDB-1";
    // Create the BLE Device
    BLEDevice::init(localName);

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new BLECustomServerCallbacks());

    // Create the BLE Service
    pService = pServer->createService(SERVICE_UUID_OTA);

    pMotorCharacteristic = pService->createCharacteristic(
                                CHARACTERISTIC_UUID_MOTOR0,
                                BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
                            );

    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();

    uint8_t motor0[5] = {0, 0, 0, 0, 0};
    pMotorCharacteristic->setValue((uint8_t*)motor0, 5);

    return true;
}

void RemoteDebug::updateModel(SemaphoreHandle_t model_changed) {
    xSemaphoreGive(model_changed);
    vTaskDelay(100);
    xSemaphoreTake(model_changed, portMAX_DELAY);
}