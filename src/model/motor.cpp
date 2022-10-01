#include "motor.hpp"
#include "model.hpp"

void Motor::set_position_by_procent(float proc) {
    t_pos = min_pos + proc * abs(max_pos - min_pos);
    Model::need_update[id] = true;
}

String Motor::get_stats() {
    return String("Motor ") + String(id) + String(" has pos:") + String(c_pos) + String(";");
}