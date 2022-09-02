#pragma once

const unsigned char MOTORS_COUNT = 12;

class Motor
{
public:
	uint8_t _can_id = -1;

	float min_pos = 0;
	float max_pos = 0;

	float pos;
	float stiffness = 0;
	float vel = 0;
	float trq = 0;
	float tr = 0;

	Motor(uint8_t can_id) : _can_id(can_id) {}
};