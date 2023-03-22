#pragma once

#include <vector>
#include <Arduino.h>
#include "freertos/semphr.h"
#include "../model/command.hpp"
#include "../model/model.hpp"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_ota_ops.h"
#include "callbacks.hpp"

class RemoteDebug
{
public:
	RemoteDebug(){};
	void loop();
private:
	BLEServer* pServer = NULL;
	BLEService* pService = NULL;
	BLECharacteristic* pMotorsOnCharacterestic = NULL;
	BLECharacteristic* pMotorsCurrentCharacteristic = NULL;
	BLECharacteristic* pMotorsTargetCharacteristic = NULL;

	bool begin();
};