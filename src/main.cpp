#include <Arduino.h>
#include <mcp2515_can.h>
#include "freertos/semphr.h"

#include "input_controller/input_controller.hpp"
#include "test_controller/test_controller.hpp"
#include "motor_controller/motor_controller.hpp"

SemaphoreHandle_t model_changed = xSemaphoreCreateBinary();

InputController input_controller;
TestController test_controller;
MotorController motor_controller;


void task_input_controller(void *p)
{
	input_controller.loop();
}

void task_test_controller(void* p)
{
	test_controller.loop(); 
}

void task_motor_controller(void *p)
{
	motor_controller.loop();
}

void setup()
{
	Serial.begin(115200);

	Model::init();

	//xTaskCreate(task_input_controller, "Input controller", 1024, NULL, 1, NULL);
	//delay(5);
	xTaskCreate(task_test_controller, "Test controller", 1024, NULL, 1, NULL);
	delay(5);
	xTaskCreate(task_motor_controller, "Motor controller", 4096, NULL, 1, NULL);
	delay(5);
}

void loop()
{
	vTaskDelete(NULL);
}
