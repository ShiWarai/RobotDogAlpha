#pragma once

const unsigned char MOTORS_COUNT = 3;

struct Motor
{
	float min_pos = 0;
	float max_pos = 0;

	float pos;
	float stiffness = 0;
	float vel = 0;
	float trq = 0;
	float tr = 0;
};