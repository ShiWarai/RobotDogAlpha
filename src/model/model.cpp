#include "model.hpp"

bool Model::init() {
    //Serial.println("Start init...");

    Model::motors[1] = Motor(0);
    Model::motors[2] = Motor(0);
    Model::motors[3] = Motor(0);
    Model::motors[4] = Motor(0);
    Model::motors[5] = Motor(0);
    Model::motors[6] = Motor(0);
    Model::motors[7] = Motor(0);
    Model::motors[8] = Motor(0);
    Model::motors[9] = Motor(0);
    Model::motors[10] = Motor(0);
    Model::motors[11] = Motor(0);
    Model::motors[12] = Motor(0);

    // Front left leg
    Model::motors[1].min_pos = -0.5;
    Model::motors[1].max_pos = 2;
    Model::motors[1].kp = 6;
    Model::motors[2].min_pos = -2;
    Model::motors[2].max_pos = 2;
    Model::motors[2].kp = 5;
    Model::motors[3].min_pos = -2;
    Model::motors[3].max_pos = 0;
    Model::motors[3].kp = 2;

    // Front right leg
    Model::motors[4].min_pos = -2;
    Model::motors[4].max_pos = 0.5;
    Model::motors[4].kp = 6;
    Model::motors[5].min_pos = -2;
    Model::motors[5].max_pos = 2;
    Model::motors[5].kp = 5;
    Model::motors[6].min_pos = 0;
    Model::motors[6].max_pos = 2;
    Model::motors[6].kp = 2;

    // Back left leg
    Model::motors[7].min_pos = -2;
    Model::motors[7].max_pos = 0.5;
    Model::motors[7].kp = 6;
    Model::motors[8].min_pos = -2;
    Model::motors[8].max_pos = 2;
    Model::motors[8].kp = 5;
    Model::motors[9].min_pos = -2;
    Model::motors[9].max_pos = 0;
    Model::motors[9].kp = 2;

    // Back right leg
    Model::motors[10].min_pos = -0.5;
    Model::motors[10].max_pos = 2;
    Model::motors[10].kp = 6;
    Model::motors[11].min_pos = -2;
    Model::motors[11].max_pos = 2;
    Model::motors[11].kp = 5;
    Model::motors[12].min_pos = 0;
    Model::motors[12].max_pos = 2;
    Model::motors[12].kp = 2;

    return true;
}

void Model::push_command(Command command) {
    Model::commands.push(command);
    Model::need_update[0] = true;
}