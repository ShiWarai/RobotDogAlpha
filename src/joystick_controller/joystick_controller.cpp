#include "joystick_controller.hpp"

JoystickController::JoystickController(std::vector<Command> *commands) : _commands(commands) {}


void JoystickController::loop()
{
    float pos1, pos2, pos3;
    float p_pos1, p_pos2, p_pos3;
    float n_pos1, n_pos2, n_pos3;

    ButtonWithState motorSwitch;
    bool motorOnLast = false;
    ButtonWithState sharePosesButton;
    ClickableButton setOriginButton;
    ClickableButton moveToOriginButton;

    PS4.begin(MAC_PS4_JOYSTICK);
    PS4.setLed(255, 0, 0);

    Serial.println("🔁 Joystick begin");
    while (1) {
        if (PS4.isConnected()) {
            pos1 = PS4.LStickX();
            pos2 = PS4.LStickY();
            pos3 = PS4.RStickY();

            // Sending the commands
            if (motorSwitch.turn(PS4.PSButton())) {
                if (motorSwitch.state() != motorOnLast)
                {
                    PS4.setLed(0, 128, 0);
                    for (unsigned long i = 1; i <= MOTORS_COUNT; i++)
                        _commands->push_back(Command{ MOTOR_ON, i, 0 });

                    motorOnLast = true;
                }
            }
            else {
                if (motorSwitch.state() != motorOnLast)
                {
                    PS4.setLed(255, 0, 0);
                    for (unsigned long i = 1; i <= MOTORS_COUNT; i++)
                        _commands->push_back(Command{ MOTOR_OFF, i, 0 });

                    motorOnLast = false;
                }
            }

            if (moveToOriginButton.turn(PS4.Circle())) {
                for (unsigned long i = 1; i <= MOTORS_COUNT; i++)
                    _commands->push_back(Command{ MOTOR_NONE, i, 0 });
            }

            if (setOriginButton.turn(PS4.Options())) {
                for (unsigned long i = 1; i <= MOTORS_COUNT; i++)
                    _commands->push_back(Command{ SET_ORIGIN, i, 0 });
            } 

            if (sharePosesButton.turn(PS4.Share())) {
                PS4.setRumble(10, 0);

                p_pos1 = float(128 + pos1) / 256;
                p_pos2 = float(128 + pos2) / 256;
                p_pos3 = float(128 + pos3) / 256;

                n_pos1 = float(128 + -pos1) / 256;
                n_pos2 = float(128 + -pos2) / 256;
                n_pos3 = float(128 + -pos3) / 256;

                _commands->push_back(Command{ CONTROL, 1, p_pos1 });
                _commands->push_back(Command{ CONTROL, 2, p_pos2 });
                _commands->push_back(Command{ CONTROL, 3, p_pos3 });

                _commands->push_back(Command{ CONTROL, 4, n_pos1 });
                _commands->push_back(Command{ CONTROL, 5, n_pos2 });
                _commands->push_back(Command{ CONTROL, 6, n_pos3 });

                _commands->push_back(Command{ CONTROL, 7, n_pos1 });
                _commands->push_back(Command{ CONTROL, 8, p_pos2 });
                _commands->push_back(Command{ CONTROL, 9, n_pos3 });

                _commands->push_back(Command{ CONTROL, 10, n_pos1 });
                _commands->push_back(Command{ CONTROL, 11, n_pos2 });
                _commands->push_back(Command{ CONTROL, 12, n_pos3 });

                vTaskDelay(1000);
            }
            else {
                PS4.setRumble(0, 0);
            }

            PS4.sendToController();
        }

        vTaskDelay(100);
    }
}
