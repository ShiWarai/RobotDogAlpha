#include <Arduino.h>
#include <mcp2515_can.h>
#include "freertos/semphr.h"

#include "input_controller/input_controller.hpp"
#include "joystick_controller/joystick_controller.hpp"
#include "motor_controller/motor_controller.hpp"
#include "remote_debug/remote_debug.hpp"

SemaphoreHandle_t model_changed = xSemaphoreCreateBinary();

InputController input_controller;
JoystickController joystick_controller;
MotorController motor_controller;
RemoteDebug remote_debug;


void task_input_controller(void *p)
{
	input_controller.loop();
}

void task_joystick_controller(void* p)
{
	joystick_controller.loop(); 
}

void task_motor_controller(void *p)
{
	motor_controller.loop();
}

void task_remote_debug(void *p)
{
	remote_debug.loop();
}


void setup()
{
	Serial.begin(115200);

	Model::init();

	xTaskCreate(task_input_controller, "Input controller", 1024, NULL, 1, NULL);
	delay(5);
	//xTaskCreate(task_joystick_controller, "Joystick controller", 10240, NULL, 1, NULL);
	//delay(5);
	xTaskCreate(task_remote_debug, "Remote debug", 4096, NULL, 1, NULL);
	delay(5);
	xTaskCreate(task_motor_controller, "Motor controller", 4096, NULL, 1, NULL);
	delay(5);
}

void loop()
{
	vTaskDelete(NULL);
}
