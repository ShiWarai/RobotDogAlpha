#pragma once

#include "../model/command.hpp"
#include "../model/model.hpp"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Misc
void uploadModel(BLECharacteristic* pMotorsCharacteristic);
void loadModel(BLECharacteristic* pMotorsCharacteristic);