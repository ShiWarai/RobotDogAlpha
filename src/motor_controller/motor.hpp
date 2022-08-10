#pragma once

const unsigned char MOTORS_COUNT = 3;

struct Motor
{
	float min_pos = 0;
	float max_pos = 0;

	float pos;
	float vel;
	float trq;
	float tr;
};