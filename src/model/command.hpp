#pragma once

enum CommandType
{
	MOTOR_NONE,
	MOTOR_OFF,
	MOTOR_ON,
	SET_ORIGIN,
	SET_MIN,
	SET_MAX,
	MOVE_MIN,
	MOVE_MAX,
	CHECK,
	CONTROL,
};

struct Command
{
	CommandType type;
	short id;
	float value; // 0-100% (0.0 - 1.0)
};
