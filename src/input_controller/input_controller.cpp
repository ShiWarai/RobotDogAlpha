#include "input_controller.hpp"

InputController::InputController(std::vector<Command> *commands) : _commands(commands) {}

void InputController::loop()
{
  const int msg_len = 32; // Max message length
  char c;                 // Read a char from the Serial
  char buf[msg_len]{0};   // Collect chars into message
  uint8_t i = 0;          // Current message chars position
  unsigned long id;

  while (1)
  {
    while (Serial.available())
    {
      c = Serial.read();

      // Read until a line return appears or last char
      // When last character is '\n' it adds another input?!
      if ((c != '\n') && (i < msg_len - 1))
      {
        buf[i] = c;
        i++;
        continue;
      }

      // TODO TESTING!!!
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
        char a = buf[1];
        char b = buf[2];

        int x = a - 48;
        int y = b - 48;

        unsigned int id = x * 10 + y;

        if (id > 99)
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

          if (id > 99)
          {
              Serial.println("Wrong id!");
              break;
          }

          _commands->push_back(Command{ SET_LOW, id, 0 });
          break;
      case 'h':
          id = (buf[1] - 48) * 10 + (buf[2] - 48);

          if (id > 99)
          {
              Serial.println("Wrong id!");
              break;
          }

          _commands->push_back(Command{ SET_HIGH, id, 0 });
          break;
      case 'w':
          id = (buf[1] - 48) * 10 + (buf[2] - 48);

          if (id > 99)
          {
              Serial.println("Wrong id!");
              break;
          }

          _commands->push_back(Command{ MOVE_HIGH, id, 0 });
          break;
      case 's':
          id = (buf[1] - 48) * 10 + (buf[2] - 48);

          if (id > 99)
          {
              Serial.println("Wrong id!");
              break;
          }

          _commands->push_back(Command{ MOVE_LOW, id, 0 });
          break;
      }

      // Reset the message
      memset(buf, 0, sizeof(buf));
      i = 0;
    }

    vTaskDelay(1);
  }
}
