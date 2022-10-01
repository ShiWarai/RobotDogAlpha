#include "motor.hpp"

String Motor::get_stats() {
    return String("Motor ") + String(id) + String(" has pos:") + String(c_pos) + String(";");
}