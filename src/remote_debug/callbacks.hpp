#pragma once

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_ota_ops.h"
#include "misc.hpp"

class BLECustomServerCallbacks : public BLEServerCallbacks
{
public:
    BLECustomServerCallbacks(BLEAdvertising *advertising)
    {
        this->advertising = advertising;
    }

private:
    BLEAdvertising *advertising;

    void onConnect(BLEServer *pServer)
    {
        // Serial.println("Connect");
        return;
    };

    void onDisconnect(BLEServer *pServer)
    {
        this->advertising->start();
        // Serial.println("Disconnect");
        return;
    }
};

class BLEMotorOnCharacteristicCallbacks : public BLECharacteristicCallbacks
{
    void onRead(BLECharacteristic *pCharacteristic)
    {
        uploadMotorsOn(pCharacteristic);
    }

    void onWrite(BLECharacteristic *pCharacteristic)
    {
        extern SemaphoreHandle_t model_changed;

        loadMotorsOn(pCharacteristic);

        xSemaphoreGive(model_changed);
        vTaskDelay(32);
        xSemaphoreTake(model_changed, portMAX_DELAY);
    }
};

class BLEReadMotorsCharacteristicCallbacks : public BLECharacteristicCallbacks
{
    void onRead(BLECharacteristic *pCharacteristic)
    {
        extern SemaphoreHandle_t model_changed;

        // Test
        Model::push_command(Command{CHECK, 1, 0});

        xSemaphoreGive(model_changed);
        vTaskDelay(32);
        xSemaphoreTake(model_changed, portMAX_DELAY);

        uploadMotorsModel(pCharacteristic);
    }
};

class BLEWriteMotorsCharacteristicCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        extern SemaphoreHandle_t model_changed;

        loadMotorsModel(pCharacteristic);

        xSemaphoreGive(model_changed);
        vTaskDelay(32);
        xSemaphoreTake(model_changed, portMAX_DELAY);

        uploadMotorsModel(pCharacteristic);
    }
};