#pragma once

enum CommandType
{
  MOTOR_NONE,
  MOTOR_OFF,
  MOTOR_ON,
  SET_ORIGIN,
  SET_LOW,
  SET_HIGH,
  MOVE_LOW,
  MOVE_HIGH,
  CHECK,
  CONTROL,
};

struct Command
{
  CommandType type;
  unsigned long id;
  float value;
};
