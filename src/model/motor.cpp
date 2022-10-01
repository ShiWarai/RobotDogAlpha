#include "motor.hpp"
#include "model.hpp"

#include <Arduino.h>

float Motor::set_position_by_procent(float proc) {
    printf("Input...");
    t_pos = min_pos + proc * abs(max_pos - min_pos);
    Model::need_update[id] = true;
    printf("%f\r\n", t_pos);
    printf("%d\r\n", Model::need_update[id]);
}