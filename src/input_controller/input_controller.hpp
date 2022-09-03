#pragma once

#include <vector>
#include <Arduino.h>

#include "command.hpp"
#include "../motor_controller/motor.hpp"
#include "freertos/semphr.h"

class InputController
{
public:
  InputController() = delete;
  InputController(std::vector<Command> *commands);

  void loop();
private:
  std::vector<Command> *_commands;
};
