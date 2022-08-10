#pragma once

#include <vector>
#include <Arduino.h>
#include "command.hpp"
#include "../motor_controller/motor.hpp"

class InputController
{
public:
  InputController() = delete;
  InputController(std::vector<Command> *commands);

  void loop();
private:
  std::vector<Command> *_commands;
};
