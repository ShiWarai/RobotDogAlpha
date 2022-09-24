#include "motor_controller.hpp"

MotorController::MotorController(std::vector<Command>* commands) {
    this->_commands = commands;
}

void MotorController::loop()
{
    Command last_command;
    Motor MOTORS[MOTORS_COUNT + 1]{ NULL, Motor(0), Motor(0), Motor(0), Motor(0), Motor(0), Motor(0), Motor(), Motor(), Motor(), Motor(), Motor(), Motor() };
    unsigned long m_id;
    float pos, vel, trq;
    extern SemaphoreHandle_t commands_ready;

    // Front left leg
    MOTORS[1].min_pos = -0.73 - 0.5;
    MOTORS[1].max_pos = 0.0 - 0.3;
    MOTORS[1].stiffness = 6;
    MOTORS[2].min_pos = -0.7;
    MOTORS[2].max_pos = 0.7;
    MOTORS[2].stiffness = 8;
    MOTORS[3].min_pos = 0.20;
    MOTORS[3].max_pos = 1.5;
    MOTORS[3].stiffness = 2;

    // Front right leg
    MOTORS[4].min_pos = 0.0 + 0.3;
    MOTORS[4].max_pos = 0.73 + 0.5;
    MOTORS[4].stiffness = 6;
    MOTORS[5].min_pos = -0.7;
    MOTORS[5].max_pos = 0.7;
    MOTORS[5].stiffness = 8;
    MOTORS[6].min_pos = -1.5;
    MOTORS[6].max_pos = 0.2;
    MOTORS[6].stiffness = 2;

    // Back left leg
    MOTORS[7].min_pos = 0.0 + 0.3;
    MOTORS[7].max_pos = 0.73 + 0.2;
    MOTORS[7].stiffness = 6;
    MOTORS[8].min_pos = -0.7;
    MOTORS[8].max_pos = 0.7;
    MOTORS[8].stiffness = 12;
    MOTORS[9].min_pos = -1.5;
    MOTORS[9].max_pos = 0.2;
    MOTORS[9].stiffness = 2;

    // Back right leg
    MOTORS[10].min_pos = 0.0 + 0.3;
    MOTORS[10].max_pos = 0.73 + 0.2;
    MOTORS[10].stiffness = 6;
    MOTORS[11].min_pos = -0.7;
    MOTORS[11].max_pos = 0.7;
    MOTORS[11].stiffness = 12;
    MOTORS[12].min_pos = -1.5;
    MOTORS[12].max_pos = 0.2;
    MOTORS[12].stiffness = 2;

    for (int i = 0; i < CAN_COUNT; i++)
    {
        while (_can_buses[i].begin(CAN_1000KBPS, MCP_8MHz) != CAN_OK)
            vTaskDelay(100);
    }

    Serial.println("🔁 Motor controller begin");
    while (1)
    {
        if (xSemaphoreTake(commands_ready, portMAX_DELAY)) {
            while (_commands->size() > 0) {
                last_command = _commands->back();
                _commands->pop_back();

                if (MOTORS[last_command.id]._can_id == -1)
                {
                    vTaskDelay(1);
                    continue;
                }

                switch (last_command.type)
                {
                case CommandType::CONTROL:

                    pos = min(max(last_command.value, float(0.0)), float(1.0));

                    // or...

                    /*
                    if (last_command.value < 0)
                        pos = 0;
                    else if (last_command.value > 1)
                        pos = 1;
                    else
                        pos = last_command.value;
                    */

                    pos = MOTORS[last_command.id].min_pos + pos * abs(MOTORS[last_command.id].max_pos - MOTORS[last_command.id].min_pos);

                    /*
                    Serial.print("Move to ");
                    Serial.print(pos);
                    Serial.print(" with stiffness ");
                    Serial.println(MOTORS[last_command.id].stiffness);
                    */

                    _control_motor(&_can_buses[MOTORS[last_command.id]._can_id], last_command.id, pos, MOTORS[last_command.id].stiffness, 0, &m_id, &pos, &vel, &trq);
                    break;

                case CommandType::MOTOR_NONE:
                    Serial.println("Set none");
                    _control_motor(&_can_buses[MOTORS[last_command.id]._can_id], last_command.id, 0, 0, 0, &m_id, &pos, &vel, &trq);
                    break;

                case CommandType::MOTOR_ON:
                    Serial.println("Motor start");
                    _start_motor(&_can_buses[MOTORS[last_command.id]._can_id], last_command.id, &m_id, &pos, &vel, &trq);
                    break;

                case CommandType::MOTOR_OFF:
                    Serial.println("Motor stop");
                    _control_motor(&_can_buses[MOTORS[last_command.id]._can_id], last_command.id, 0, 0, 0, &m_id, &pos, &vel, &trq);
                    _stop_motor(&_can_buses[MOTORS[last_command.id]._can_id], last_command.id, &m_id, &pos, &vel, &trq);
                    break;

                case CommandType::SET_ORIGIN:
                    Serial.println("Motor zero");
                    _zero_motor(&_can_buses[MOTORS[last_command.id]._can_id], last_command.id, &m_id, &pos, &vel, &trq);
                    break;

                case CommandType::CHECK:
                    Serial.print("Check motor ");
                    Serial.println(last_command.id);

                    _check_motor(&_can_buses[MOTORS[last_command.id]._can_id], last_command.id, &m_id, &pos, &vel, &trq);

                    Serial.print("motor id: ");
                    Serial.print(m_id);
                    Serial.print(" pos: ");
                    Serial.print(pos);
                    Serial.print(" vel: ");
                    Serial.print(vel);
                    Serial.print(" trq : ");
                    Serial.print(trq);
                    Serial.print(" min : ");
                    Serial.print(MOTORS[last_command.id].min_pos);
                    Serial.print(" max : ");
                    Serial.print(MOTORS[last_command.id].max_pos);
                    Serial.println();
                    Serial.println();
                    break;

                case CommandType::SET_MIN:
                    Serial.println("Set low!");

                    _check_motor(&_can_buses[MOTORS[last_command.id]._can_id], last_command.id, &m_id, &pos, &vel, &trq);

                    MOTORS[last_command.id].min_pos = pos;
                    Serial.print("New min: ");
                    Serial.println(MOTORS[last_command.id].min_pos);
                    break;

                case CommandType::SET_MAX:
                    Serial.println("Set max!");

                    _check_motor(&_can_buses[MOTORS[last_command.id]._can_id], last_command.id, &m_id, &pos, &vel, &trq);

                    MOTORS[last_command.id].max_pos = pos;
                    Serial.print("New max: ");
                    Serial.println(MOTORS[last_command.id].max_pos);
                    break;

                case CommandType::MOVE_MIN:
                    Serial.print("Move to min: ");
                    Serial.println(MOTORS[last_command.id].min_pos);

                    _control_motor(&_can_buses[MOTORS[last_command.id]._can_id], last_command.id, MOTORS[last_command.id].min_pos, MOTORS[last_command.id].stiffness, 0, &m_id, &pos, &vel, &trq);
                    break;

                case CommandType::MOVE_MAX:
                    Serial.print("Move to high: ");
                    Serial.println(MOTORS[last_command.id].max_pos);

                    _control_motor(&_can_buses[MOTORS[last_command.id]._can_id], last_command.id, MOTORS[last_command.id].max_pos, MOTORS[last_command.id].stiffness, 0, &m_id, &pos, &vel, &trq);
                    break;

                default:
                    break;
                }

                MOTORS[last_command.id].pos = pos;
                MOTORS[last_command.id].vel = vel;
                MOTORS[last_command.id].trq = trq;

                Serial.print(last_command.id);
                Serial.print(": ");
                Serial.println(pos);
                Serial.println(vel);
                Serial.println(trq);
                Serial.println();
            }

            xSemaphoreGive(commands_ready);
            vTaskDelay(1);
        }

        vTaskDelay(1);
    }
}

void MotorController::_start_motor(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
                                   unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq) // Motor parameters
{
  can->sendMsgBuf(id, 0, 8, START_MOTOR);
  vTaskDelay(_delay);
  _can_unpack(can, id, m_id, m_pos, m_vel, m_trq);
  vTaskDelay(_delay);
}

void MotorController::_stop_motor(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
                                  unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq) // Motor parameters
{
  can->sendMsgBuf(id, 0, 8, STOP_MOTOR);
  vTaskDelay(_delay);
  _can_unpack(can, id, m_id, m_pos, m_vel, m_trq);
  vTaskDelay(_delay);
}

void MotorController::_zero_motor(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
                                  unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq) // Motor parameters
{
  can->sendMsgBuf(id, 0, 8, SET_ZERO);
  vTaskDelay(_delay);
  _can_unpack(can, id, m_id, m_pos, m_vel, m_trq);
  vTaskDelay(_delay);
}

void MotorController::_check_motor(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
                                   unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq) // Motor parameters
{
  _can_pack(can, id, 0, 0, 0);
  vTaskDelay(_delay);
  _can_unpack(can, id, m_id, m_pos, m_vel, m_trq);
  vTaskDelay(_delay);
}

void MotorController::_control_motor(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
                                     float new_position, float new_stiffness, float new_damper,     // New parameters
                                     unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq) // Motor parameters
{
  _can_pack(can, id, new_position, new_stiffness, new_damper);
  vTaskDelay(_delay);
  _can_unpack(can, id, m_id, m_pos, m_vel, m_trq);
  vTaskDelay(_delay);
}

unsigned int MotorController::_float_to_uint(float x, float x_min, float x_max, float bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;

  if (bits == 12)
    pgg = (unsigned int)((x - offset) * 4095.0 / span);

  if (bits == 16)
    pgg = (unsigned int)((x - offset) * 65535.0 / span);

  return pgg;
}

float MotorController::_uint_to_float(unsigned int x_int, float x_min, float x_max, float bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0.0f;

  if (bits == 12)
    pgg = ((float)x_int) * span / 4095.0 + offset;

  if (bits == 16)
    pgg = ((float)x_int) * span / 65535.0 + offset;

  return pgg;
}

byte MotorController::_can_pack(mcp2515_can *can, unsigned long id,                        // CAN bus and CAN ID
                                float new_position, float new_stiffness, float new_damper) // New parameters
{
  // Limit data TODO Use struct functions
  // t_f torque and v_f velocity are 0 for now
  float p_f = constrain(new_position, P_MIN, P_MAX);
  float v_f = constrain(0, V_MIN, V_MAX);
  float kp_f = constrain(new_stiffness, KP_MIN, KP_MAX);
  float kd_f = constrain(new_damper, KD_MIN, KD_MAX);
  float t_f = constrain(0, T_MIN, T_MAX);

  // Convert data TODO Use struct functions
  unsigned int p_int = _float_to_uint(p_f, P_MIN, P_MAX, 16);
  unsigned int v_int = _float_to_uint(v_f, V_MIN, V_MAX, 12);
  unsigned int kp_int = _float_to_uint(kp_f, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = _float_to_uint(kd_f, KD_MIN, KD_MAX, 12);
  unsigned int t_int = _float_to_uint(t_f, T_MIN, T_MAX, 12);

  // Pack uints into the buffer
  byte buf[8] = {
      p_int >> 8,
      p_int & 0xFF,
      v_int >> 4,
      ((v_int & 0xF) << 4) | (kp_int >> 8),
      kp_int & 0xFF,
      kd_int >> 4,
      ((kd_int & 0xF) << 4) | (t_int >> 8),
      t_int & 0xFF};

  return can->sendMsgBuf(id, 0, 8, buf);
}

// Receive a message
byte MotorController::_can_unpack(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
                                  unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq) // Motor parameters
{
  byte len = 0;
  byte buf[8]{0};
  byte result = can->readMsgBuf(&len, buf);

  // Unpack uints from the buffer
  unsigned long buf_id = buf[0];
  unsigned int p_int = (buf[1] << 8) | buf[2];
  unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
  unsigned int t_int = ((buf[4] & 0xF) << 8) | buf[5];

  // Convert uints to floats
  *m_id = buf_id;
  *m_pos = _uint_to_float(p_int, P_MIN, P_MAX, 16);
  *m_vel = _uint_to_float(v_int, V_MIN, V_MAX, 12);
  *m_trq = _uint_to_float(t_int, -T_MAX, T_MAX, 12);

  return result;
}
