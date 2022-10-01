#pragma once

#include <cmath>

class Motor
{
public:
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

	float set_position_by_procent(float proc);

	Motor(short _can_id) : can_id(_can_id) {}
	Motor() {}
};