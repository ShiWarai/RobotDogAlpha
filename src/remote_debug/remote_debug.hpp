#pragma once

#include <vector>
#include <Arduino.h>
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
