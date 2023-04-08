#include "motor_controller.hpp"


void MotorController::loop()
{
    extern SemaphoreHandle_t model_changed;

    Command last_command;
    unsigned long t_id;
    float t_pos, t_vel, t_trq;

    unsigned long c_id;
    float c_pos, c_vel, c_trq;

    for (int i = 0; i < CAN_COUNT; i++)
    {
        while (can_buses[i].begin(CAN_1000KBPS, MCP_8MHz) != CAN_OK)
            vTaskDelay(100);
    }

    Serial.println("🔁 Motor controller begin");
    while (1)
    {
        if (xSemaphoreTake(model_changed, portMAX_DELAY)) {
            for(uint8_t id = 0; id <= MOTORS_COUNT; id++)
            {
                if(!Model::need_update[id]) // Break if model is not changed
                    continue;

                if(id != 0) {
					if (Model::motors[id].can_id != -1)
                            control_motor(&can_buses[Model::motors[id].can_id], id, &Model::motors[id]);
                    continue;
                }
                else {
                    while (!Model::commands.empty()) {
                        last_command = Model::commands.front();
                        Model::commands.pop();

                        if (Model::motors[last_command.id].can_id == -1)
                            continue;

                        t_id = last_command.id;
                        switch (last_command.type)
                        {
                            case CommandType::CONTROL:

                                t_pos = constrain(last_command.value, 0.0, 1.0);
                                t_pos = Model::motors[t_id].min_pos + t_pos * abs(Model::motors[t_id].max_pos - Model::motors[t_id].min_pos);

                                Serial.print("Move to ");
                                Serial.print(t_pos);
                                Serial.print(" with stiffness ");
                                Serial.println(Model::motors[t_id].kp);

                                _control_motor(&can_buses[Model::motors[t_id].can_id], t_id, t_pos, Model::motors[t_id].kp,
                                                &Model::motors[t_id].c_pos, &Model::motors[t_id].c_vel, &Model::motors[t_id].c_trq);
                                break;

                            case CommandType::MOTOR_NONE:
                                Serial.println("Set none");
                                _control_motor(&can_buses[Model::motors[t_id].can_id], t_id, 0, 0,
                                                &Model::motors[t_id].c_pos, &Model::motors[t_id].c_vel, &Model::motors[t_id].c_trq);
                                break;

                            case CommandType::MOTOR_ON:
                                Serial.println("Motor start");
                                _start_motor(&can_buses[Model::motors[t_id].can_id], t_id,
                                                &Model::motors[t_id].c_pos, &Model::motors[t_id].c_vel, &Model::motors[t_id].c_trq);
								Model::motors[t_id].turn_on = true;
                                break;

                            case CommandType::MOTOR_OFF:
                                Serial.println("Motor stop");
                                _control_motor(&can_buses[Model::motors[t_id].can_id], t_id, 0, 0,
                                                &Model::motors[t_id].c_pos, &Model::motors[t_id].c_vel, &Model::motors[t_id].c_trq);
								Model::motors[t_id].t_pos = 0;
                                _stop_motor(&can_buses[Model::motors[t_id].can_id], t_id,
                                            &Model::motors[t_id].c_pos, &Model::motors[t_id].c_vel, &Model::motors[t_id].c_trq);
								Model::motors[t_id].turn_on = false;
                                break;

                            case CommandType::SET_ORIGIN:
                                Serial.println("Motor zero");
                                _zero_motor(&can_buses[Model::motors[t_id].can_id], t_id, 
                                            &Model::motors[t_id].c_pos, &Model::motors[t_id].c_vel, &Model::motors[t_id].c_trq);

                                Model::motors[t_id].set_origin = true;
                                break;

                            case CommandType::CHECK:
                                Serial.print("Check motor ");
                                Serial.println(t_id);

                                if(Model::motors[t_id].turn_on)
                                  control_motor(&can_buses[Model::motors[t_id].can_id], t_id, &Model::motors[t_id]);
                                  //_check_motor(&can_buses[Model::motors[t_id].can_id], t_id, &Model::motors[t_id].c_pos, &Model::motors[t_id].c_vel, &Model::motors[t_id].c_trq);

                                Serial.print("Pos: ");
                                Serial.print(Model::motors[t_id].c_pos);
                                Serial.print(" vel: ");
                                Serial.print(Model::motors[t_id].c_vel);
                                Serial.print(" trq : ");
                                Serial.print(Model::motors[t_id].c_trq);
                                Serial.print(" min : ");
                                Serial.print(Model::motors[t_id].min_pos);
                                Serial.print(" max : ");
                                Serial.println(Model::motors[t_id].max_pos);
                                Serial.println();
                                break;

                            case CommandType::SET_MIN:
                                Serial.println("Set low!");

                                _check_motor(&can_buses[Model::motors[t_id].can_id], t_id,
                                                &Model::motors[t_id].c_pos, &Model::motors[t_id].c_vel, &Model::motors[t_id].c_trq);

                                Model::motors[t_id].min_pos = Model::motors[t_id].c_pos;
                                Serial.print("New min: ");
                                Serial.println(Model::motors[t_id].min_pos);
                                break;

                            case CommandType::SET_MAX:
                                Serial.println("Set max!");

                                _check_motor(&can_buses[Model::motors[t_id].can_id], t_id,
                                                &Model::motors[t_id].c_pos, &Model::motors[t_id].c_vel, &Model::motors[t_id].c_trq);

                                Model::motors[t_id].max_pos = Model::motors[t_id].c_pos;
                                Serial.print("New max: ");
                                Serial.println(Model::motors[t_id].max_pos);
                                break;

                            case CommandType::MOVE_MIN:
                                Serial.print("Move to min: ");
                                Serial.println(Model::motors[t_id].min_pos);

                                _control_motor(&can_buses[Model::motors[t_id].can_id], t_id, Model::motors[t_id].min_pos, Model::motors[t_id].kp,
                                                &Model::motors[t_id].c_pos, &Model::motors[t_id].c_vel, &Model::motors[t_id].c_trq);
                                break;

                            case CommandType::MOVE_MAX:
                                Serial.print("Move to high: ");
                                Serial.println(Model::motors[t_id].max_pos);

                                _control_motor(&can_buses[Model::motors[t_id].can_id], t_id, Model::motors[t_id].max_pos, Model::motors[t_id].kp,
                                                &Model::motors[t_id].c_pos, &Model::motors[t_id].c_vel, &Model::motors[t_id].c_trq);
                                break;

                            default:
                                break;
                        }

                        /*
                        Serial.print(t_id);
                        Serial.print(": ");
                        Serial.println(pos);
                        Serial.println(vel);
                        Serial.println(trq);
                        Serial.println();
                        */
                    }
                }

				// Now motor is up to date
				Model::need_update[id] = false;
            }

            xSemaphoreGive(model_changed);
            taskYIELD();
        }

        vTaskDelay(1);
    }
}

void MotorController::_start_motor(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
                                   float *c_pos, float *c_vel, float *c_trq) // Motor parameters
{
  can->sendMsgBuf(id, 0, 8, START_MOTOR);
  vTaskDelay(DELAY);
  can_unpack(can, id, c_pos, c_vel, c_trq);
}

void MotorController::_stop_motor(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
                                  float *c_pos, float *c_vel, float *c_trq) // Motor parameters
{
  can->sendMsgBuf(id, 0, 8, STOP_MOTOR);
  vTaskDelay(DELAY);
  can_unpack(can, id, c_pos, c_vel, c_trq);
}

void MotorController::_zero_motor(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
                                  float *c_pos, float *c_vel, float *c_trq) // Motor parameters
{
  can->sendMsgBuf(id, 0, 8, SET_ZERO);
  vTaskDelay(SET_ORIGIN_WAITING);
  can_unpack(can, id, c_pos, c_vel, c_trq);
}

void MotorController::_check_motor(mcp2515_can *can, unsigned long id,
                                   float *c_pos, float *c_vel, float *c_trq)
{
  can->sendMsgBuf(id, 0, 8, START_MOTOR);
  vTaskDelay(CHECK_WAITING);
  can_unpack(can, id, c_pos, c_vel, c_trq);
}

void MotorController::control_motor(mcp2515_can *can, unsigned long id, Motor *motor)
{
  if(motor->t_pos >= motor->min_pos && motor->t_pos <= motor->max_pos)
  {
    if(motor->t_vel == 0 && motor->t_trq == 0) 
    {
      can_pack(can, id, motor->t_pos, motor->kp, motor->t_vel, motor->kd, motor->t_trq); // Undone
	    vTaskDelay(DELAY);
	    can_unpack(can, id, &motor->c_pos, &motor->c_vel, &motor->c_trq);
    }
  } else {
    Serial.printf("Wrong position for motor: %d\r\n", motor->id);
  }
}

void MotorController::_control_motor(mcp2515_can *can, unsigned long id,
                                     float t_pos, float t_kp,
                                     float *c_pos, float *c_vel, float *c_trq)
{
  can_pack(can, id, t_pos, t_kp); // Undone
  vTaskDelay(DELAY);
  can_unpack(can, id, c_pos, c_vel, c_trq);
}

unsigned int MotorController::float_to_uint(float x, float x_min, float x_max, float bits)
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

float MotorController::uint_to_float(unsigned int x_int, float x_min, float x_max, float bits)
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

// Send a message
byte MotorController::can_pack(mcp2515_can *can, unsigned long id,
                                 float t_pos, float t_kp, float t_vel, float t_kd, float t_trq)
{
    // Limit data
    float p_f = constrain(t_pos, P_MIN, P_MAX);
    float v_f = constrain(t_vel, V_MIN, V_MAX);
    float t_f = constrain(t_trq, T_MIN, T_MAX);
    float kp_f = constrain(t_kp, KP_MIN, KP_MAX);
    float kd_f = constrain(t_kd, KD_MIN, KD_MAX);

    // Convert data
    unsigned int p_int = float_to_uint(p_f, P_MIN, P_MAX, 16);
    unsigned int v_int = float_to_uint(v_f, V_MIN, V_MAX, 12);
    unsigned int kp_int = float_to_uint(kp_f, KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(kd_f, KD_MIN, KD_MAX, 12);
    unsigned int t_int = float_to_uint(t_f, T_MIN, T_MAX, 12);

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
byte MotorController::can_unpack(mcp2515_can *can, unsigned long id,
                                    float *c_pos, float *c_vel, float *c_trq, unsigned long *m_id)
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
  // *m_id = buf_id; // undone
  *c_pos = uint_to_float(p_int, P_MIN, P_MAX, 16);
  *c_vel = uint_to_float(v_int, V_MIN, V_MAX, 12);
  *c_trq = uint_to_float(t_int, -T_MAX, T_MAX, 12);

  return result;
}
