#pragma once

#include <vector>
#include <Arduino.h>
#include "freertos/semphr.h"
#include "../model/command.hpp"
#include "../model/model.hpp"
#include "misc.hpp"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_ota_ops.h"

class RemoteDebug
{
public:
	RemoteDebug(){};
	void loop();
private:
	BLEServer *pServer = NULL;
	BLEService *pService = NULL;
	BLECharacteristic* pMotorsCurrentCharacteristic = NULL;
	BLECharacteristic* pMotorsTargetCharacteristic = NULL;

	bool begin();
};


class BLECustomServerCallbacks : public BLEServerCallbacks
{
public:

	BLECustomServerCallbacks(BLEAdvertising* advertising) {
		this->advertising = advertising;
	}

private:
	BLEAdvertising* advertising;

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


class BLECustomCharacteristicCallbacks: public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		extern SemaphoreHandle_t model_changed;

		loadModel(pCharacteristic);

		xSemaphoreGive(model_changed);
		taskYIELD();
		xSemaphoreTake(model_changed, portMAX_DELAY);
	}
};