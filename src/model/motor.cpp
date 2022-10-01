#include "motor.hpp"
#include "model.hpp"

#include <Arduino.h>

float Motor::set_position_by_procent(float proc) {
    Serial.println("Input...");
    t_pos = min_pos + proc * abs(max_pos - min_pos);
    Model::need_update[id] = true;
    Serial.println(t_pos);
    Serial.println(Model::need_update[id]);
}