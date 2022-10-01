#include "joystick_controller.hpp"

void set_motor_pos_by_proc(short motor_id, float proc) {
    Model::motors[motor_id].t_pos = Model::motors[motor_id].min_pos + proc * abs(Model::motors[motor_id].max_pos - Model::motors[motor_id].min_pos);
    Model::need_update[motor_id] = true;
}

void JoystickController::loop()
{
    extern SemaphoreHandle_t model_changed;

    float pos1, pos2, pos3;
    float p_pos1, p_pos2, p_pos3;
    float n_pos1, n_pos2, n_pos3;

    ButtonWithState motorSwitch;
    bool motorOnLast = false;
    ButtonWithState sharePosesButton;
    ClickableButton setOriginButton;
    ClickableButton moveToOriginButton;

    PS4.begin(MAC_PS4_JOYSTICK);
    PS4.setLed(255, 255, 0);

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
                    for (short i = 1; i <= MOTORS_COUNT; i++)
                        Model::push_command(Command{ MOTOR_ON, i, 0 });

                    xSemaphoreGive(model_changed);
                    vTaskDelay(100);
                    xSemaphoreTake(model_changed, portMAX_DELAY);

                    motorOnLast = true;
                }
            }
            else {
                if (motorSwitch.state() != motorOnLast)
                {
                    PS4.setLed(255, 0, 0);
                    for (short i = 1; i <= MOTORS_COUNT; i++)
                        Model::push_command(Command{ MOTOR_OFF, i, 0 });

                    xSemaphoreGive(model_changed);
                    vTaskDelay(100);
                    xSemaphoreTake(model_changed, portMAX_DELAY);

                    motorOnLast = false;
                }
            }

            if (moveToOriginButton.turn(PS4.Circle())) {
                for (short i = 1; i <= MOTORS_COUNT; i++)
                    Model::push_command(Command{ MOTOR_NONE, i, 0 });

                xSemaphoreGive(model_changed);
                vTaskDelay(100);
                xSemaphoreTake(model_changed, portMAX_DELAY);
            }

            if (setOriginButton.turn(PS4.Options())) {
                for (short i = 1; i <= MOTORS_COUNT; i++)
                    Model::push_command(Command{ SET_ORIGIN, i, 0 });

                xSemaphoreGive(model_changed);
                vTaskDelay(100);
                xSemaphoreTake(model_changed, portMAX_DELAY);
                PS4.setLed(255, 0, 0);
            } 

            if (sharePosesButton.turn(PS4.Share())) {
                PS4.setRumble(20, 0);

                p_pos1 = float(128 + pos1) / 256;
                p_pos2 = float(128 + pos2) / 256;
                p_pos3 = float(128 + pos3) / 256;

                n_pos1 = float(128 + -pos1) / 256;
                n_pos2 = float(128 + -pos2) / 256;
                n_pos3 = float(128 + -pos3) / 256;

				set_motor_pos_by_proc(1, p_pos1);
				set_motor_pos_by_proc(2, p_pos2);
				set_motor_pos_by_proc(3, p_pos3);

                set_motor_pos_by_proc(4, n_pos1);
                set_motor_pos_by_proc(5, n_pos2);
                set_motor_pos_by_proc(6, n_pos3);

				/*
                Model::push_command(Command{ CONTROL, 1, p_pos1 });
                Model::push_command(Command{ CONTROL, 2, p_pos2 });
                Model::push_command(Command{ CONTROL, 3, p_pos3 });

                Model::push_command(Command{ CONTROL, 4, n_pos1 });
                Model::push_command(Command{ CONTROL, 5, n_pos2 });
                Model::push_command(Command{ CONTROL, 6, n_pos3 });

                Model::push_command(Command{ CONTROL, 7, n_pos1 });
                Model::push_command(Command{ CONTROL, 8, p_pos2 });
                Model::push_command(Command{ CONTROL, 9, n_pos3 });

                Model::push_command(Command{ CONTROL, 10, n_pos1 });
                Model::push_command(Command{ CONTROL, 11, n_pos2 });
                Model::push_command(Command{ CONTROL, 12, n_pos3 });
				*/

                xSemaphoreGive(model_changed);
                vTaskDelay(100);
                xSemaphoreTake(model_changed, portMAX_DELAY);
            }
            else {
                PS4.setRumble(0, 0);
            }

            PS4.sendToController();
        }

        vTaskDelay(100);
    }
}
