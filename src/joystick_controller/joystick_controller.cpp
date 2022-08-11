#include "joystick_controller.hpp"

JoystickController::JoystickController(std::vector<Command> *commands) : _commands(commands) {}


void JoystickController::loop()
{
    float pos1;
    float pos2;
    float pos3;

    PS4.begin(MAC_PS4_JOYSTICK);

    ButtonWithState sharePoses;

    while (1) {
        if (PS4.isConnected()) {
            pos1 = PS4.LStickX();
            pos2 = PS4.LStickY();
            pos3 = PS4.RStickY();

            sharePoses.turn(PS4.Share());

            // Sending the commands
            if (sharePoses.state()) {
                pos1 = float(128 + pos1) / 256;
                pos2 = float(128 + pos2) / 256;
                pos3 = float(128 + pos3) / 256;

                _commands->push_back(Command{ CONTROL, 1, pos1 });
                _commands->push_back(Command{ CONTROL, 2, pos2 });
                _commands->push_back(Command{ CONTROL, 3, pos3 });
                vTaskDelay(100);
            }
        }

        vTaskDelay(100);
    }
}
