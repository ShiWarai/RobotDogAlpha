#pragma once

enum CommandType
{
  MOTOR_NONE,
  MOTOR_OFF,
  MOTOR_ON,
  MOTOR_ZERO,
  CHECK,
  CONTROL,
};

struct Command
{
  CommandType type;
  unsigned long id;
  float value;
};
