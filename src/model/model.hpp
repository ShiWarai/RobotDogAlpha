#pragma once

#include <queue>

#include "motor.hpp"
#include "command.hpp"

const unsigned char MOTORS_COUNT = 12;

class Model {
public:
    inline static bool need_update[MOTORS_COUNT + 1];
    inline static Motor motors[MOTORS_COUNT + 1];
    inline static std::queue<Command> commands;

    static bool init();
    static void push_command(Command command);

    Model() = delete;
};