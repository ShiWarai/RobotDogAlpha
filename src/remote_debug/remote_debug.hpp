#pragma once

#include <vector>
#include <Arduino.h>
#include "freertos/semphr.h"
#include "../model/command.hpp"
#include "../model/model.hpp"
#include "BluetoothSerial.h"

#define PIN_CODE "2284"

class RemoteDebug
{
public:
    RemoteDebug() {};
    void loop();
private:
    // Misc
    void updateModel(SemaphoreHandle_t model_changed);

    // Bluetooth
    void trust_loop(BluetoothSerial* serial_bt);
};

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_ota_ops.h"

class BLE; // forward declaration

class BLECustomServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      // deviceConnected = true;
      // // code here for when device connects
    };

    void onDisconnect(BLEServer* pServer) {
      // deviceConnected = false;
      // // code here for when device disconnects
    }
};

class otaCallback: public BLECharacteristicCallbacks {
  public:
    otaCallback(BLE* ble) {
      _p_ble = ble;
    }
    BLE* _p_ble;

    void onWrite(BLECharacteristic *pCharacteristic);
};

class BLE
{
  public:

    bool begin();
  
  private:
    BLEServer *pServer = NULL;

    BLEService *pESPOTAService = NULL;
    BLECharacteristic * pESPOTAIdCharacteristic = NULL;

    BLEService *pService = NULL;
    BLECharacteristic * pMotorCharacteristic = NULL;
    BLECharacteristic * pOtaCharacteristic = NULL;
};

