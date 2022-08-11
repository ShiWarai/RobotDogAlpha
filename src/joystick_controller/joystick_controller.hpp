#pragma once

#include <vector>
#include <Arduino.h>
#include "../input_controller/command.hpp"
#include "../motor_controller/motor.hpp"
#include <PS4Controller.h>

#define MAC_PS4_JOYSTICK "c0:e4:34:4f:b0:4a"

class ButtonWithState {
private:
	bool __state;
	bool __buttonPressed = false;
public:

    ButtonWithState(bool state = false) {
        this->__state = state;
    }

	bool state() {
		return __state;
	}

	bool turn(bool side) {
        if (side) {
            if (!__buttonPressed)
            {
                __state = !__state;
            }
            __buttonPressed = true;
        }
        else {
            __buttonPressed = false;
        }

        return __state;
	}
};

class JoystickController
{
public:
  JoystickController() = delete;
  JoystickController(std::vector<Command> *commands);

  void loop();
private:
  std::vector<Command> *_commands;
};
