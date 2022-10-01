#pragma once

#include <Arduino.h>

static short motor_counter = 0;

class Motor
{
public:
	short id;
	short can_id = -1;
	bool axis_inversion;

	bool turn_on = false;

	float t_pos = 0;
	float t_vel = 0;
	float t_trq = 0;
	
	float kp = 0;
	float kd = 0;

	float c_pos;
	float c_vel;
	float c_trq;

	float min_pos = 0;
	float max_pos = 0;

	String get_stats();

	Motor(short _id, short _can_id) : id(_id), can_id(_can_id) {}
	Motor(short _can_id) : can_id(_can_id) { id = ++motor_counter; }
	Motor() {}
};