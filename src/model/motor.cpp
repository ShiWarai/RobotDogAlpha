#include "motor.hpp"
#include "model.hpp"

#include <Arduino.h>

void Motor::set_position_by_procent(float proc) {
    t_pos = min_pos + proc * abs(max_pos - min_pos);
    Model::need_update[id] = true;
}