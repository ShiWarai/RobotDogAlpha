#include <vector>
#include <Arduino.h>
#include <mcp2515_can.h>

#include "input_controller/command.hpp"
#include "input_controller/input_controller.hpp"
std::vector<Command> commands;
InputController input_controller(&commands);
void task_input_controller(void *p)
{
  Serial.println("🔁 Serial input begin");
  input_controller.loop();
}

#include "motor_controller/motor_controller.hpp"
mcp2515_can can1(5);
MotorController motor_controller(&can1, &commands);
void task_motor_controller(void *p)
{
  Serial.println("🔁 Motor controller begin");
  motor_controller.loop();
}

void setup()
{
  delay(500);
  Serial.begin(115200);

  while (can1.begin(CAN_1000KBPS, MCP_8MHz) != CAN_OK)
  {
    Serial.println("CAN 1 - error on begin!");
    delay(1000);
  }

  xTaskCreate(task_input_controller, "Input controller", 1024, NULL, 1, NULL);
  delay(5);
  xTaskCreate(task_motor_controller, "Motor controller", 1024, NULL, 1, NULL);
  delay(5);
}

void loop()
{
  vTaskDelete(NULL);
}
