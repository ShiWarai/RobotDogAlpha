#include <vector>
#include <Arduino.h>
#include <mcp2515_can.h>
#include "motor_controller/motor.hpp"
#include "input_controller/command.hpp"
#include "input_controller/input_controller.hpp"
#include "motor_controller/motor.hpp"

Motor MOTORS[MOTORS_COUNT + 1]{ Motor{}, Motor{}, Motor{}, Motor{} };

std::vector<Command> commands;

InputController input_controller(&commands);
void task_input_controller(void *p)
{
  Serial.println("🔁 Serial input begin");
  input_controller.loop();
}

#include "motor_controller/motor_controller.hpp"
mcp2515_can can_bus(5);
MotorController motor_controller(&can_bus, &commands);
void task_motor_controller(void *p)
{
  Serial.println("🔁 Motor controller begin");
  motor_controller.loop();
}

void setup()
{
  delay(500);
  Serial.begin(115200);

  while (can_bus.begin(CAN_1000KBPS, MCP_8MHz) != CAN_OK)
  {
    Serial.println("CAN 1 - error on begin!");
    delay(1000);
  }

  // Set max and min - DEBUG
  MOTORS[1].min_pos = -0.73;
  MOTORS[1].max_pos = 0.32;
  MOTORS[2].min_pos = -0.39;
  MOTORS[2].max_pos = 0.65;
  MOTORS[3].min_pos = 0.00;
  MOTORS[3].max_pos = 1.74;

  xTaskCreate(task_input_controller, "Input controller", 1024, NULL, 1, NULL);
  delay(5);
  xTaskCreate(task_motor_controller, "Motor controller", 1024, NULL, 1, NULL);
  delay(5);
}

void loop()
{
  vTaskDelete(NULL);
}
