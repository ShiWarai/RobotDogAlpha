#include "input_controller.hpp"

InputController::InputController(std::vector<Command> *commands) : _commands(commands) {}

void InputController::loop()
{
  const int msg_len = 32; // Max message length
  char c;                 // Read a char from the Serial
  char buf[msg_len]{0};   // Collect chars into message
  uint8_t i = 0;          // Current message chars position
  unsigned long id;
  unsigned short pos;
  extern SemaphoreHandle_t commands_ready;

  Serial.println("🔁 Serial input begin");
  while (1)
  {
    while (Serial.available())
    {
      c = Serial.read();

      if ((c != '\n') && (i < msg_len - 1))
      {
        buf[i] = c;
        i++;
        continue;
      }

      switch (buf[0])
      {
      case 'a': // Start all motors
        for (unsigned long i = 1; i <= MOTORS_COUNT; i++)
            _commands->push_back(Command{MOTOR_ON, i, 0});
        break;

      case 'b': // Stop all motors
        for (unsigned long i = 1; i <= MOTORS_COUNT; i++)
            _commands->push_back(Command{MOTOR_OFF, i, 0});
        break;

      case 'c': // Check a motor
      {
          id = (buf[1] - 48) * 10 + (buf[2] - 48);

          if (id <= 0 || id >= 100)
          {
              Serial.println("Wrong id!");
              break;
          }

        _commands->push_back(Command{CHECK, id, 0});
        break;
      }

      case 'f': // Zero all motors
        for (unsigned long i = 1; i <= MOTORS_COUNT; i++)
            _commands->push_back(Command{SET_ORIGIN, i, 0});
        break;

      case 'l':
          id = (buf[1] - 48) * 10 + (buf[2] - 48);

          if (id <= 0 || id >= 100)
          {
              Serial.println("Wrong id!");
              break;
          }

          _commands->push_back(Command{ SET_MIN, id, 0 });
          break;

      case 'h':
          id = (buf[1] - 48) * 10 + (buf[2] - 48);

          if (id <= 0 || id >= 100)
          {
              Serial.println("Wrong id!");
              break;
          }

          _commands->push_back(Command{ SET_MAX, id, 0 });
          break;

      case 'w':
          id = (buf[1] - 48) * 10 + (buf[2] - 48);

          if (id <= 0 || id >= 100)
          {
              Serial.println("Wrong id!");
              break;
          }

          _commands->push_back(Command{ MOVE_MAX, id, 0 }); // Replace to _commands->push_back(Command{ CONTROL, id, 1 });
          break;

      case 's':
          id = (buf[1] - 48) * 10 + (buf[2] - 48);

          if (id <= 0 || id >= 100)
          {
              Serial.println("Wrong id!");
              break;
          }

          _commands->push_back(Command{ MOVE_MIN, id, 0 }); // Replace to _commands->push_back(Command{ CONTROL, id, 0 });
          break;

      case 'm':
          id = (buf[1] - 48) * 10 + (buf[2] - 48);
          pos = (buf[4] - 48) * 100 + (buf[5] - 48) * 10 + (buf[6] - 48);

          if (id <= 0 || id >= 100)
          {
              Serial.println("Wrong id!");
              break;
          }

          if (!(pos >= 0 && pos <= 100)) {
              Serial.println("Wrong position!");
              break;
          }

          _commands->push_back(Command{ CONTROL, id, float(pos) / 100 });
          break;
      }

      // Reset the message
      memset(buf, 0, sizeof(buf));
      i = 0;

      xSemaphoreGive(commands_ready);
      vTaskDelay(100);
      xSemaphoreTake(commands_ready, portMAX_DELAY);
    }

    vTaskDelay(1);
  }
}
