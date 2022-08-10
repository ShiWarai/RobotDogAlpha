#include "motor_controller.hpp"

MotorController::MotorController(mcp2515_can *can1, std::vector<Command> *commands) : _can1(can1), _commands(commands) {}

void MotorController::loop()
{
  Command last_command;

  while (1)
  {
    // unsigned long id1 = 1; // TEST
    // unsigned long id2 = 2; // TEST
    // unsigned long id3 = 3; // TEST

    // if (_serial_input->set_zero)
    // {
    //   // Set zero on all motors
    //   Serial.println("Motors are zeroed!");

    //   _zero_motor(_can1, id1, &m_id, &pos, &vel, &trq);
    //   _zero_motor(_can1, id2, &m_id, &pos, &vel, &trq);
    //   _zero_motor(_can1, id3, &m_id, &pos, &vel, &trq);

    //   _serial_input->set_zero = false;
    //   vTaskDelay(1000);
    // }

    // if (_serial_input->go_zero)
    // {
    //   // Set zero on all motors
    //   Serial.println("Motors go zero!");

    //   Serial.print("🍒 start ");
    //   _start_motor(_can1, id1, &m_id, &pos, &vel, &trq);
    //   _start_motor(_can1, id2, &m_id, &pos, &vel, &trq);
    //   _start_motor(_can1, id3, &m_id, &pos, &vel, &trq);

    //   vTaskDelay(100);

    //   Serial.print("🍏 go ");
    //   _control_motor(_can1, id1, 0, 12, 1, &m_id, &pos, &vel, &trq);
    //   _control_motor(_can1, id2, 0, 8, 1, &m_id, &pos, &vel, &trq);
    //   _control_motor(_can1, id3, 0, 4, 1, &m_id, &pos, &vel, &trq);

    //   vTaskDelay(5000);

    //   Serial.print("🫐 stop");
    //   Serial.println("\n");
    //   _stop_motor(_can1, id1, &m_id, &pos, &vel, &trq);
    //   _stop_motor(_can1, id2, &m_id, &pos, &vel, &trq);
    //   _stop_motor(_can1, id3, &m_id, &pos, &vel, &trq);

    //   _serial_input->go_zero = false;
    //   vTaskDelay(1000);
    // }

    // // Motor 1
    // _check_motor(_can1, id1, &m_id, &pos, &vel, &trq);

    // Serial.print("🔴 Check motor ");
    // Serial.print(id1);
    // Serial.print(" motor id: ");
    // Serial.print(m_id);
    // Serial.print(" pos: ");
    // Serial.print(pos);
    // Serial.print(" vel: ");
    // Serial.print(vel);
    // Serial.print(" trq : ");
    // Serial.print(trq);
    // Serial.println();

    // vTaskDelay(10);

    // // Motor 2
    // _check_motor(_can1, id2, &m_id, &pos, &vel, &trq);

    // Serial.print("🟢 Check motor ");
    // Serial.print(id2);
    // Serial.print(" motor id: ");
    // Serial.print(m_id);
    // Serial.print(" pos: ");
    // Serial.print(pos);
    // Serial.print(" vel: ");
    // Serial.print(vel);
    // Serial.print(" trq : ");
    // Serial.print(trq);
    // Serial.println();

    // vTaskDelay(10);

    // // Motor 3
    // _check_motor(_can1, id3, &m_id, &pos, &vel, &trq);

    // Serial.print("🔵 Check motor ");
    // Serial.print(id3);
    // Serial.print(" motor id: ");
    // Serial.print(m_id);
    // Serial.print(" pos: ");
    // Serial.print(pos);
    // Serial.print(" vel: ");
    // Serial.print(vel);
    // Serial.print(" trq : ");
    // Serial.print(trq);
    // Serial.println();
    // Serial.println();

    unsigned long m_id;
    float pos, vel, trq;

    if (_commands->size() > 0)
    {
      last_command = _commands->back();
      _commands->pop_back();
    }

    switch (last_command.type)
    {
    case CommandType::MOTOR_ON:
      Serial.println("Motor start");
      _start_motor(_can1, last_command.id, &m_id, &pos, &vel, &trq);
      break;

    case CommandType::MOTOR_OFF:
      Serial.println("Motor stop");
      _stop_motor(_can1, last_command.id, &m_id, &pos, &vel, &trq);
      break;

    case CommandType::MOTOR_ZERO:
      Serial.println("Motor zero");
      _zero_motor(_can1, last_command.id, &m_id, &pos, &vel, &trq);
      break;

    case CommandType::CHECK:
      Serial.print("Check motor ");
      Serial.println(last_command.id);
      _check_motor(_can1, last_command.id, &m_id, &pos, &vel, &trq);

      Serial.print("motor id: ");
      Serial.print(m_id);
      Serial.print(" pos: ");
      Serial.print(pos);
      Serial.print(" vel: ");
      Serial.print(vel);
      Serial.print(" trq : ");
      Serial.print(trq);
      Serial.println();
      Serial.println();
      break;

    default:
      break;
    }

    last_command = Command{};
    vTaskDelay(100);
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
  do // DANGER!! Risk of an infinite loop present!
  {
    _can_unpack(can, id, m_id, m_pos, m_vel, m_trq);
    vTaskDelay(_delay);
  } while (id != *m_id);
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
