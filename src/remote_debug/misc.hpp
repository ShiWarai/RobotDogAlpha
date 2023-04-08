#pragma once

#include <cstdint>
#include "../model/command.hpp"
#include "../model/model.hpp"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Misc
void uploadMotorsModel(BLECharacteristic* motorsCharacteristic);
void loadMotorsModel(BLECharacteristic* motorsCharacteristic);
void uploadMotorsOn(BLECharacteristic* characteristic);
void loadMotorsOn(BLECharacteristic* characteristic);