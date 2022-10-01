#pragma once

#include <vector>
#include <Arduino.h>

#include "../model/command.hpp"
#include "../model/model.hpp"
#include "freertos/semphr.h"
#include "freertos/task.h"

class InputController
{
public:
    InputController() {};
    void loop();
};
