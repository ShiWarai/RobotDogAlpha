#include "misc.hpp"
const size_t motorsModelVars = MOTORS_COUNT * 5;
const size_t motorsOnBufferSize = 2;

void convertFloatArrayToUint8Array(float* input, uint8_t* output, size_t size) 
{
    // Предполагаем, что размер output равен 4 * size
    for (size_t i = 0; i < size; i++) {
        // Преобразуем указатель на float в указатель на uint8_t
        uint8_t* bytes = reinterpret_cast<uint8_t*>(&input[i]);
        // Копируем 4 байта из bytes в output
        memcpy(&output[4 * i], bytes, 4);
    }
}

void convertUint8ArrayToFloatArray(uint8_t* input, float* output, size_t size) 
{
    // Предполагаем, что размер input равен 4 * size
    for (size_t i = 0; i < size; i++) {
        // Преобразуем указатель на uint8_t в указатель на float
        float* value = reinterpret_cast<float*>(&input[4 * i]);
        // Копируем значение из value в output
        output[i] = *value;
    }
}

void convertBoolArrayToUint8Array(bool* boolArray, size_t boolArraySize, uint8_t* uint8_tArray, size_t uint8_tArraySize) {
    uint8_t uint8_tValue = 0;
    int bitPosition = 0;
    for (size_t i = 0; i < boolArraySize; i++) {
        uint8_tValue |= (boolArray[i] << bitPosition);
        bitPosition++;
        if (bitPosition == 8) {
            if (i / 8 < uint8_tArraySize) {
                uint8_tArray[i / 8] = uint8_tValue;
            }
            uint8_tValue = 0;
            bitPosition = 0;
        }
    }

    if (bitPosition > 0 && boolArraySize / 8 < uint8_tArraySize) {
        uint8_tArray[boolArraySize / 8] = uint8_tValue;
    }
}

void convertUint8ArrayToBoolArray(uint8_t *uint8_tArray, size_t uint8_tArraySize, bool *boolArray, size_t boolArraySize)
{
    uint8_t uint8_tValue;
    int bitPosition = 0;
    for (size_t i = 0; i < boolArraySize; i++)
    {
        if (i / 8 < uint8_tArraySize)
        {
            uint8_tValue = uint8_tArray[i / 8];
            boolArray[i] = (uint8_tValue >> bitPosition) & 1;
        }
        else
        {
            boolArray[i] = false;
        }
        bitPosition++;

        if (bitPosition == 8)
        {
            bitPosition = 0;
        }
    }
}

void uploadMotorsModel(BLECharacteristic* characteristic) 
{
    float motors_data[motorsModelVars];
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

    uint8_t buffer[motorsModelVars*4];
    convertFloatArrayToUint8Array(motors_data, buffer, motorsModelVars);
    
    characteristic->setValue(buffer, sizeof(buffer));

    return;
}

void loadMotorsModel(BLECharacteristic* characteristic)
{
    float motors_data[motorsModelVars];

    convertUint8ArrayToFloatArray(characteristic->getData(), motors_data, motorsModelVars);

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
                // Model::motors[m].t_vel = motors_data[(m-1)*5 + i];
                break;
            case 3:
                Model::motors[m].kd = motors_data[(m-1)*5 + i];
                break;
            case 4:
                // Model::motors[m].t_trq = motors_data[(m-1)*5 + i];
                break;
            default:
                break;
            }
        }

        Model::need_update[m] = true; // need optimize
    }

    return;
}

void uploadMotorsOn(BLECharacteristic *characteristic)
{
    bool motorsOnStates[MOTORS_COUNT];

    for(uint8_t m = 1; m <= MOTORS_COUNT; m++) {
        motorsOnStates[m-1] = Model::motors[m].turn_on;
    }

    uint8_t buffer[motorsOnBufferSize];
    convertBoolArrayToUint8Array(motorsOnStates, MOTORS_COUNT, buffer, motorsOnBufferSize);

    characteristic->setValue(buffer, motorsOnBufferSize);

    return;
}

void loadMotorsOn(BLECharacteristic *characteristic)
{
    bool motorsOnStates[MOTORS_COUNT];

    convertUint8ArrayToBoolArray(characteristic->getData(), 2, motorsOnStates, MOTORS_COUNT);
    
    for(uint8_t m = 1; m <= MOTORS_COUNT; m++) {
        if(motorsOnStates[m-1] && !Model::motors[m].turn_on) {
            Model::push_command(Command{ MOTOR_ON, m, 0 });
        } else if (!motorsOnStates[m-1] && Model::motors[m].turn_on) {
            Model::push_command(Command{ MOTOR_OFF, m, 0 });
        }
    }
    
    return;
}