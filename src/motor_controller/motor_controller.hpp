#pragma once

#include <vector>
#include <Arduino.h>
#include <mcp2515_can.h>

#include "../model/command.hpp"
#include "freertos/semphr.h"

#include "commands.hpp"
#include "limits.hpp"
#include "../model/model.hpp"

#define CAN_COUNT 1

class MotorController
{
public:
  MotorController() {};
  void loop();

private:
  mcp2515_can can_buses[CAN_COUNT] = { mcp2515_can(4) };

  const int DELAY = 8;
  const int SET_ORIGIN_WAITING = 1500;
  const int CHECK_WAITING = 64;

  void _start_motor(mcp2515_can *can, unsigned long id,
                    float *m_pos, float *m_vel, float *m_trq);

  void _stop_motor(mcp2515_can *can, unsigned long id,
                   float *m_pos, float *m_vel, float *m_trq);

  void _zero_motor(mcp2515_can *can, unsigned long id,
                    float *m_pos, float *m_vel, float *m_trq);

  void _check_motor(mcp2515_can *can, unsigned long id,
                    float *m_pos, float *m_vel, float *m_trq);

  void control_motor(mcp2515_can *can, unsigned long id, Motor *motor);
  void _control_motor(mcp2515_can *can, unsigned long id,
                                     float t_pos, float t_kp,
                                     float *c_pos, float *c_vel, float *c_trq);

  unsigned int float_to_uint(float x, float x_min, float x_max, float bits);
  float uint_to_float(unsigned int x_int, float x_min, float x_max, float bits);

  byte can_pack(mcp2515_can *can, unsigned long id, float t_pos, float t_kp, float t_vel = 0, float t_kd = 0, float t_trq = 0);
  byte can_unpack(mcp2515_can *can, unsigned long id, float *c_pos, float *c_vel, float *c_trq, unsigned long *m_id = nullptr);
};
