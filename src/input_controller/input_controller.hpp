#pragma once

#include <vector>
#include <Arduino.h>
#include "command.hpp"

class InputController
{
public:
  InputController() = delete;
  InputController(std::vector<Command> *commands);

  bool set_zero = false; // DEBUG
  bool go_zero = false;  // DEBUG
  void loop();

private:
  std::vector<Command> *_commands;
};
