#include "joystick_controller.hpp"
#define REMOVE_BONDED_DEVICES 1   // <- Set to 0 to view all bonded devices addresses, set to 1 to remove

#define PAIR_MAX_DEVICES 20
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

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

				Model::motors[1].set_position_by_procent(p_pos1);
				Model::motors[2].set_position_by_procent(p_pos2);
				Model::motors[3].set_position_by_procent(p_pos3);

                Model::motors[4].set_position_by_procent(n_pos1);
                Model::motors[5].set_position_by_procent(n_pos2);
                Model::motors[6].set_position_by_procent(n_pos3);

                Model::motors[7].set_position_by_procent(n_pos1);
				Model::motors[8].set_position_by_procent(p_pos2);
				Model::motors[9].set_position_by_procent(n_pos3);

                Model::motors[10].set_position_by_procent(n_pos1);
                Model::motors[11].set_position_by_procent(n_pos2);
                Model::motors[12].set_position_by_procent(n_pos3);

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
                taskYIELD();
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

void JoystickController::cleanPairedDevices() {
    initBluetooth();
    // Get the numbers of bonded/paired devices in the BT module
    int count = esp_bt_gap_get_bond_device_num();

    if(count) {
        if(PAIR_MAX_DEVICES < count)
            count = PAIR_MAX_DEVICES;

        esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
        if(ESP_OK == tError) {
            for(int i = 0; i < count; i++) {
                    Serial.print("Found bonded device # "); Serial.print(i); Serial.print(" -> ");
                    Serial.println(bda2str(pairedDeviceBtAddr[i], bda_str, 18));     
                    if(REMOVE_BONDED_DEVICES) {
                        esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
                        if(ESP_OK == tError) {
                            Serial.print("Removed bonded device # "); 
                        } else {
                            Serial.print("Failed to remove bonded device # ");
                        }
                        Serial.println(i);
                    }
            }
        }        
    }
}

bool JoystickController::initBluetooth()
{
  if(!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
 
  if(esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
 
  if(esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}

char* JoystickController::bda2str(const uint8_t* bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}