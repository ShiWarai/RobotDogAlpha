#include <vector>
#include <Arduino.h>
#include <mcp2515_can.h>
#include "freertos/semphr.h"
#include "motor_controller/limits.hpp"
#include "motor_controller/commands.hpp"
#include "motor_controller/motor.hpp"
#include <mcp2515_can.h>

#define CAN_COUNT 1
#define DELAY 8

mcp2515_can _can_buses[CAN_COUNT] = { mcp2515_can(5) };

Motor MOTORS[MOTORS_COUNT + 1]{ NULL, Motor(0), Motor(0), Motor(0), Motor(0), Motor(0), Motor(0)};

void _start_motor(mcp2515_can *can, unsigned long id,                             // CAN bus and CAN ID
				  unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq); // Motor parameters

void _stop_motor(mcp2515_can *can, unsigned long id,                             // CAN bus and CAN ID
				  unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq); // Motor parameters

void _zero_motor(mcp2515_can *can, unsigned long id,                             // CAN bus and CAN ID
				  unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq); // Motor parameters

void _check_motor(mcp2515_can *can, unsigned long id,                             // CAN bus and CAN ID
				  unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq); // Motor parameters

void _control_motor(mcp2515_can *can, unsigned long id,                             // CAN bus and CAN ID
					float new_position, float new_stiffness, float new_damper,      // New parameters
					unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq); // Motor parameters

unsigned int _float_to_uint(float x, float x_min, float x_max, float bits);
float _uint_to_float(unsigned int x_int, float x_min, float x_max, float bits);

byte _can_pack(mcp2515_can *can, unsigned long id, float new_position, float new_stiffness, float new_damper); // Control
byte _can_unpack(mcp2515_can *can, unsigned long id, unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq);

void setup()
{
	Serial.begin(115200);

  	for (int i = 0; i < CAN_COUNT; i++)
	{
		while (_can_buses[i].begin(CAN_1000KBPS, MCP_8MHz) != CAN_OK)
			vTaskDelay(100);
	}

	MOTORS[1].min_pos = -0.73;
	MOTORS[1].max_pos = -0.3;
	MOTORS[1].stiffness = 2;
	MOTORS[2].min_pos = -0.7;
	MOTORS[2].max_pos = 0.7;
	MOTORS[2].stiffness = 2;
	MOTORS[3].min_pos = 0.2;
	MOTORS[3].max_pos = 0.7;
	MOTORS[3].stiffness = 1;

	// Front right leg
	MOTORS[4].min_pos = 0.3;
	MOTORS[4].max_pos = 0.73;
	MOTORS[4].stiffness = 2;
	MOTORS[5].min_pos = -0.3;
	MOTORS[5].max_pos = 0.3;
	MOTORS[5].stiffness = 2;
	MOTORS[6].min_pos = -0.5;
	MOTORS[6].max_pos = 0.2;
	MOTORS[6].stiffness = 1;
}

void loop()
{
	unsigned long m_id;
	float pos, vel, trq;

	for(unsigned long id = 1; id <= MOTORS_COUNT; id++) {
		if (MOTORS[id]._can_id == -1)
			continue;

		Serial.println("Motor zero");
		_zero_motor(&_can_buses[MOTORS[id]._can_id], id, &m_id, &pos, &vel, &trq);
		vTaskDelay(3000);
	}

	for (unsigned long id = 1; id <= MOTORS_COUNT; id++) {
		vTaskDelay(DELAY);
		if (MOTORS[id]._can_id == -1)
			continue;

		Serial.println("Motor start");
		_start_motor(&_can_buses[MOTORS[id]._can_id], id, &m_id, &pos, &vel, &trq);
	}

	while (1) {
		Serial.println("🔁 Cycle is started");
		vTaskDelay(1500);

		for (unsigned long id = 1; id <= MOTORS_COUNT; id++) {
			vTaskDelay(DELAY);
			if (MOTORS[id]._can_id == -1)
				continue;

			pos = (float)constrain(0.5, 0.0, 1.0);

			pos = MOTORS[id].min_pos + pos * abs(MOTORS[id].max_pos - MOTORS[id].min_pos);

			Serial.print("Move to ");
			Serial.print(pos);
			Serial.print(" with stiffness ");
			Serial.println(MOTORS[id].stiffness);

			_control_motor(&_can_buses[MOTORS[id]._can_id], id, pos, MOTORS[id].stiffness, 0, &m_id, &pos, &vel, &trq);

			MOTORS[id].pos = pos;
			MOTORS[id].vel = vel;
			MOTORS[id].trq = trq;

			Serial.print(id);
			Serial.print(": ");
			Serial.println(pos);
			Serial.println(vel);
			Serial.println(trq);
			Serial.println();
		}

		vTaskDelay(3000);

		for (unsigned long id = 1; id <= MOTORS_COUNT; id++) {
			vTaskDelay(DELAY);
			if (MOTORS[id]._can_id == -1)
				continue;

			pos = (float)constrain(0.0, 0.0, 1.0);

			pos = MOTORS[id].min_pos + pos * abs(MOTORS[id].max_pos - MOTORS[id].min_pos);

			Serial.print("Move to ");
			Serial.print(pos);
			Serial.print(" with stiffness ");
			Serial.println(MOTORS[id].stiffness);

			_control_motor(&_can_buses[MOTORS[id]._can_id], id, pos, MOTORS[id].stiffness, 0, &m_id, &pos, &vel, &trq);
		}

		Serial.println("🔁 Cycle is stoped");
		vTaskDelay(1500);
	}

	for (unsigned long id = 1; id <= MOTORS_COUNT; id++) {
		vTaskDelay(DELAY);
		if (MOTORS[id]._can_id == -1)
			continue;

		Serial.println("Motor stop");
		//_control_motor(&_can_buses[MOTORS[id]._can_id], id, 0, 0, 0, &m_id, &pos, &vel, &trq);
		//vTaskDelay(DELAY);
		_stop_motor(&_can_buses[MOTORS[id]._can_id], id, &m_id, &pos, &vel, &trq);
	}

	while(1) {
		vTaskDelay(1);
	}
}





void _start_motor(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
								   unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq) // Motor parameters
{
  can->sendMsgBuf(id, 0, 8, START_MOTOR);
  vTaskDelay(DELAY);
  _can_unpack(can, id, m_id, m_pos, m_vel, m_trq);
  vTaskDelay(DELAY);
}

void _stop_motor(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
								  unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq) // Motor parameters
{
  can->sendMsgBuf(id, 0, 8, STOP_MOTOR);
  vTaskDelay(DELAY);
  _can_unpack(can, id, m_id, m_pos, m_vel, m_trq);
  vTaskDelay(DELAY);
}

void _zero_motor(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
								  unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq) // Motor parameters
{
  can->sendMsgBuf(id, 0, 8, SET_ZERO);
  vTaskDelay(DELAY);
  _can_unpack(can, id, m_id, m_pos, m_vel, m_trq);
  vTaskDelay(DELAY);
}

void _control_motor(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
									 float new_position, float new_stiffness, float new_damper,     // New parameters
									 unsigned long *m_id, float *m_pos, float *m_vel, float *m_trq) // Motor parameters
{
  _can_pack(can, id, new_position, new_stiffness, new_damper);
  vTaskDelay(DELAY);
  _can_unpack(can, id, m_id, m_pos, m_vel, m_trq);
  vTaskDelay(DELAY);
}

unsigned int _float_to_uint(float x, float x_min, float x_max, float bits)
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

float _uint_to_float(unsigned int x_int, float x_min, float x_max, float bits)
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

byte _can_pack(mcp2515_can *can, unsigned long id,                        // CAN bus and CAN ID
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
byte _can_unpack(mcp2515_can *can, unsigned long id,                            // CAN bus and CAN ID
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
