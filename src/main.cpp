#include <vector>
#include <Arduino.h>
#include <mcp2515_can.h>

#include "input_controller/input_controller.hpp"
#include "joystick_controller/joystick_controller.hpp"
#include "motor_controller/motor_controller.hpp"

Motor MOTORS[MOTORS_COUNT + 1] { NULL, Motor(0), Motor(0), Motor(0), Motor(1), Motor(1), Motor(1), Motor(2), Motor(2), Motor(2), Motor(3), Motor(3), Motor(3) };

std::vector<Command> commands;

InputController input_controller(&commands);
JoystickController joystick_controller(&commands);
MotorController motor_controller(&commands);


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

void setup()
{
  Serial.begin(115200);

  // Set max and min - DEBUG

  // Front left leg
  MOTORS[1].min_pos = -0.73 - 0.5;
  MOTORS[1].max_pos = 0.0 - 0.3;
  MOTORS[1].stiffness = 15;
  MOTORS[2].min_pos = -0.5;
  MOTORS[2].max_pos = 0.5;
  MOTORS[2].stiffness = 8;
  MOTORS[3].min_pos = 0.20;
  MOTORS[3].max_pos = 1.5;
  MOTORS[3].stiffness = 1;

  // Front right leg
  MOTORS[4].min_pos = 0.0 + 0.3;
  MOTORS[4].max_pos = 0.73 + 0.5;
  MOTORS[4].stiffness = 15;
  MOTORS[5].min_pos = -0.5;
  MOTORS[5].max_pos = 0.5;
  MOTORS[5].stiffness = 8;
  MOTORS[6].min_pos = -1.5;
  MOTORS[6].max_pos = 0.2;
  MOTORS[6].stiffness = 1;

  xTaskCreate(task_input_controller, "Input controller", 1024, NULL, 1, NULL);
  delay(5);
  xTaskCreate(task_joystick_controller, "Joystick controller", 10240, NULL, 1, NULL);
  delay(5);
  xTaskCreate(task_motor_controller, "Motor controller", 4096, NULL, 1, NULL);
  delay(5);
}

void loop()
{
  vTaskDelete(NULL);
}
