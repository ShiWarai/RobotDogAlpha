#include "joystick_controller.hpp"

void JoystickController::loop()
{
    extern SemaphoreHandle_t model_changed;

    float pos1, pos2, pos3;
    float p_pos1, p_pos2, p_pos3;
    float n_pos1, n_pos2, n_pos3;

    int movement_tick = 0;
    const int MAX_MOVENENT_TICKS = 4;
    const float MOVEMENT_STATES_POS[MAX_MOVENENT_TICKS][3] = {{0.7, 0.7, 0.8}, {0.7, 0.85, 0.6}, {0.7, 1.0, 0.4}, {0.7, 0.85, 0.6}};

    ButtonWithState motorSwitch;
    bool motorOnLast = false;
    ButtonWithState sharePosesButton;
    ButtonWithState legsMovingButton;
    bool legsMovingLast = false;
    ClickableButton setOriginButton;
    ClickableButton moveToOriginButton;

    cleanPairedDevices();

    PS4.begin(MAC_PS4_JOYSTICK);
    PS4.setLed(255, 255, 0);

    Serial.println("🔁 Joystick begin");
    while (1) {
        if (PS4.isConnected()) {
            pos1 = PS4.LStickX();
            pos2 = PS4.LStickY();
            pos3 = PS4.RStickY();


            if (motorSwitch.turn(PS4.PSButton())) {
                if (motorSwitch.state() != motorOnLast)
                {
                    PS4.setLed(0, 128, 0);
                    for (short i = 1; i <= MOTORS_COUNT; i++)
                        Model::push_command(Command{ MOTOR_ON, i, 0 });

                    this->updateModel(model_changed);

                    motorOnLast = true;
                }
            }
            else {
                if (motorSwitch.state() != motorOnLast)
                {
                    PS4.setLed(255, 0, 0);
                    for (short i = 1; i <= MOTORS_COUNT; i++)
                        Model::push_command(Command{ MOTOR_OFF, i, 0 });

                    this->updateModel(model_changed);

                    motorOnLast = false;
                    legsMovingLast = false;
                }
            }

            if (moveToOriginButton.turn(PS4.Circle())) {
                for (short i = 1; i <= MOTORS_COUNT; i++)
                    Model::push_command(Command{ MOTOR_NONE, i, 0 });

                this->updateModel(model_changed);
            }

            if (PS4.Square()) {
                Model::push_command(Command{ MOTOR_ON, 3, 0 });
                Model::push_command(Command{ MOTOR_ON, 9, 0 });

                this->updateModel(model_changed);
                vTaskDelay(500);
            }

            if (setOriginButton.turn(PS4.Options())) {
                for (short i = 1; i <= MOTORS_COUNT; i++)
                    Model::push_command(Command{ SET_ORIGIN, i, 0 });

                this->updateModel(model_changed);
                PS4.setLed(255, 0, 0);
            }

            if (legsMovingButton.turn(PS4.Cross())) {
                legsMovingLast = true;

                Model::motors[1].set_position_by_procent(MOVEMENT_STATES_POS[movement_tick][1-1]);
                Model::motors[2].set_position_by_procent(MOVEMENT_STATES_POS[movement_tick][2-1]);
                Model::motors[3].set_position_by_procent(MOVEMENT_STATES_POS[movement_tick][3-1]);

                Model::motors[4].set_position_by_procent(1 - MOVEMENT_STATES_POS[(movement_tick + (MAX_MOVENENT_TICKS / 2)) % MAX_MOVENENT_TICKS][1-1]);
                Model::motors[5].set_position_by_procent(1 - MOVEMENT_STATES_POS[(movement_tick + (MAX_MOVENENT_TICKS / 2)) % MAX_MOVENENT_TICKS][2-1]);
                Model::motors[6].set_position_by_procent(1 - MOVEMENT_STATES_POS[(movement_tick + (MAX_MOVENENT_TICKS / 2)) % MAX_MOVENENT_TICKS][3-1]);

                Model::motors[7].set_position_by_procent(1 - MOVEMENT_STATES_POS[(movement_tick + (MAX_MOVENENT_TICKS / 2)) % MAX_MOVENENT_TICKS][1-1]);
                Model::motors[8].set_position_by_procent(MOVEMENT_STATES_POS[(movement_tick + (MAX_MOVENENT_TICKS / 2)) % MAX_MOVENENT_TICKS][2-1]);
                Model::motors[9].set_position_by_procent(MOVEMENT_STATES_POS[(movement_tick + (MAX_MOVENENT_TICKS / 2)) % MAX_MOVENENT_TICKS][3-1]);

                Model::motors[10].set_position_by_procent(MOVEMENT_STATES_POS[movement_tick][1-1]);
                Model::motors[11].set_position_by_procent(1 - MOVEMENT_STATES_POS[movement_tick][2-1]);
                Model::motors[12].set_position_by_procent(1 - MOVEMENT_STATES_POS[movement_tick][3-1]);

                movement_tick++;

                if(movement_tick >= MAX_MOVENENT_TICKS) {
                    movement_tick = 0;
                }

                xSemaphoreGive(model_changed);
                vTaskDelay(500);
                xSemaphoreTake(model_changed, portMAX_DELAY);

                PS4.setRumble(20, 0);
                PS4.setLed(124, 0, 255);
            }
            else {
                if (legsMovingLast) {

                    movement_tick = 0;
                    legsMovingLast = false;

                    PS4.setRumble(0, 0);

                    if (motorOnLast)
                        PS4.setLed(0, 128, 0);
                    else
                        PS4.setLed(255, 0, 0);
                }
            }

            if (sharePosesButton.turn(PS4.Share())) {

                p_pos1 = float(128 + pos1) / 256;
                p_pos2 = float(128 + pos2) / 256;
                p_pos3 = float(128 + pos3) / 256;

                n_pos1 = float(128 + -pos1) / 256;
                n_pos2 = float(128 + -pos2) / 256;
                n_pos3 = float(128 + -pos3) / 256;

                Model::motors[1].set_position_by_procent(n_pos1);
                Model::motors[2].set_position_by_procent(n_pos2);
                Model::motors[3].set_position_by_procent(n_pos3);

                Model::motors[4].set_position_by_procent(p_pos1);
                Model::motors[5].set_position_by_procent(n_pos2);
                Model::motors[6].set_position_by_procent(n_pos3);

                Model::motors[7].set_position_by_procent(p_pos1);
                Model::motors[8].set_position_by_procent(p_pos2);
                Model::motors[9].set_position_by_procent(p_pos3);

                Model::motors[10].set_position_by_procent(n_pos1);
                Model::motors[11].set_position_by_procent(p_pos2);
                Model::motors[12].set_position_by_procent(p_pos3);

                /*
                Model::motors[1].set_position_by_procent(n_pos1);
                Model::motors[2].set_position_by_procent(n_pos2);
                Model::motors[3].set_position_by_procent(n_pos3);

                Model::motors[4].set_position_by_procent(p_pos1);
                Model::motors[5].set_position_by_procent(p_pos2);
                Model::motors[6].set_position_by_procent(p_pos3);

                Model::motors[7].set_position_by_procent(p_pos1);
                Model::motors[8].set_position_by_procent(n_pos2);
                Model::motors[9].set_position_by_procent(n_pos3);

                Model::motors[10].set_position_by_procent(n_pos1);
                Model::motors[11].set_position_by_procent(p_pos2);
                Model::motors[12].set_position_by_procent(p_pos3);
                */

                PS4.setRumble(20, 0);

                xSemaphoreGive(model_changed);
                taskYIELD();
                xSemaphoreTake(model_changed, portMAX_DELAY);
            } else {
                PS4.setRumble(0, 0);
            }

            PS4.sendToController(); // !!! Replace !!!
        }

        vTaskDelay(32);
    }
}

void JoystickController::updateModel(SemaphoreHandle_t model_changed) {
    xSemaphoreGive(model_changed);
    vTaskDelay(100);
    xSemaphoreTake(model_changed, portMAX_DELAY);
}

void JoystickController::cleanPairedDevices() {
    char bda_str[18];
    uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];

    initBluetooth();

    int count = esp_bt_gap_get_bond_device_num(); // Get the numbers of bonded/paired devices in the BT module

    if(count) {
        if(PAIR_MAX_DEVICES < count)
            count = PAIR_MAX_DEVICES;

        esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
        if(ESP_OK == tError) {
            for(int i = 0; i < count; i++) {
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

    disableBluetooth();
}

bool JoystickController::initBluetooth()
{
    if(!btStart())
        return false;

    if(esp_bluedroid_init() != ESP_OK)
        return false;

    if(esp_bluedroid_enable() != ESP_OK)
        return false;

    return true;
}

bool JoystickController::disableBluetooth()
{
    if(esp_bluedroid_disable() != ESP_OK)
        return false;

    if(esp_bluedroid_deinit() != ESP_OK)
        return false;

    if(!btStop())
        return false;

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