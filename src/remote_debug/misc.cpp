#include "misc.hpp"

void convert(float* input, uint8_t* output, size_t size) {
    // Предполагаем, что размер output равен 4 * size
    for (size_t i = 0; i < size; i++) {
        // Преобразуем указатель на float в указатель на uint8_t
        uint8_t* bytes = reinterpret_cast<uint8_t*>(&input[i]);
        // Копируем 4 байта из bytes в output
        memcpy(&output[4 * i], bytes, 4);
    }
}

void convert_inv(uint8_t* input, float* output, size_t size) {
    // Предполагаем, что размер input равен 4 * size
    for (size_t i = 0; i < size; i++) {
        // Преобразуем указатель на uint8_t в указатель на float
        float* value = reinterpret_cast<float*>(&input[4 * i]);
        // Копируем значение из value в output
        output[i] = *value;
    }
}

void uploadModel(BLECharacteristic* pMotorsCharacteristic) {
    const size_t vars_count = MOTORS_COUNT * 5;

    float motors_data[vars_count];
    for(uint8_t m = 1; m <= MOTORS_COUNT; m++) {
        for(uint8_t i = 0; i < 5; i++) {
            switch (i)
            {
            case 0:
                motors_data[(m-1)*5 + i] = Model::motors[m].c_pos;
                break;
            case 1:
                motors_data[(m-1)*5 + i] = Model::motors[m].kp;
                break;
            case 2:
                motors_data[(m-1)*5 + i] = Model::motors[m].c_vel;
                break;
            case 3:
                motors_data[(m-1)*5 + i] = Model::motors[m].kd;
                break;
            case 4:
                motors_data[(m-1)*5 + i] = Model::motors[m].c_trq;
                break;
            default:
                break;
            }
        }
    }

    uint8_t buffer[vars_count*4];
    convert(motors_data, buffer, vars_count);
    
    pMotorsCharacteristic->setValue(buffer, sizeof(buffer));

    return;
}

void loadModel(BLECharacteristic* pMotorsCharacteristic)
{
    const size_t vars_count = MOTORS_COUNT * 5;
    float motors_data[vars_count];

    convert_inv(pMotorsCharacteristic->getData(), motors_data, vars_count);

    for(uint8_t m = 1; m <= MOTORS_COUNT; m++) {
        for(uint8_t i = 0; i < 5; i++) {
            switch (i)
            {
            case 0:
                Model::motors[m].t_pos = motors_data[(m-1)*5 + i];
                break;
            case 1:
                Model::motors[m].kp = motors_data[(m-1)*5 + i];
                break;
            case 2:
                Model::motors[m].t_vel = motors_data[(m-1)*5 + i];
                break;
            case 3:
                Model::motors[m].kd = motors_data[(m-1)*5 + i];
                break;
            case 4:
                Model::motors[m].t_trq = motors_data[(m-1)*5 + i];
                break;
            default:
                break;
            }
        }

        Model::need_update[m] = true; // need optimize
    }

    return;
}
