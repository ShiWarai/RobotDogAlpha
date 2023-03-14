#include "remote_debug.hpp"

void RemoteDebug::loop()
{
    extern SemaphoreHandle_t model_changed;

    BluetoothSerial* serial_bt = new BluetoothSerial();
    serial_bt->begin("ESP32 OTA Updates");

    Serial.println("🔁 Remote debug begin");

    String input;
    while (1) {

        if (serial_bt->available()) { // Если есть данные от Bluetooth
            input = serial_bt->readString(); // Считываем строку
            if (input == PIN_CODE) {
                serial_bt->println("Success"); // Отправляем сообщение об успехе
                trust_loop(serial_bt);
            }
            else { // Иначе
                serial_bt->println("Wrong pin-code"); // Отправляем сообщение об ошибке
                serial_bt->disconnect();
                continue;
            }
        }
        else { // Если нет данных от Bluetooth
            serial_bt->println("Write pin-code"); // Отправляем запрос на ввод пин-кода
            vTaskDelay(1000);
        }

        vTaskDelay(1);
    }

    delete serial_bt;
}

void RemoteDebug::updateModel(SemaphoreHandle_t model_changed) {
    xSemaphoreGive(model_changed);
    vTaskDelay(100);
    xSemaphoreTake(model_changed, portMAX_DELAY);
}

void RemoteDebug::trust_loop(BluetoothSerial* serial_bt)
{
    extern SemaphoreHandle_t model_changed;

    float data[12][3] = {
        {1.1, 2.2, 3.3},
        {4.4, 5.5, 6.6},
        {7.7, 8.8, 9.9},
        {10.1, 11.2, 12.3},
        {13.4, 14.5 ,15.6},
        {16.7 ,17.8 ,18.9},
        {19.,20.,21,},
        {22.,23.,24,},
        {25.,26.,27,},
        {28.,29.,30,},
    };

    float pos1, pos2, pos3;
    float p_pos1, p_pos2, p_pos3;
    float n_pos1, n_pos2, n_pos3;

    String buf;
    while(1) {
        if (serial_bt->available()) {
            buf = serial_bt->readString();

            switch (buf[0])
            {
            case '0':
            {
                serial_bt->println(data[0][2]);
                break;
            }
            case 'a':
            {
                short id = (buf[1] - 48) * 10 + (buf[2] - 48);

                if (id <= 0 || id > MOTORS_COUNT)
                {
                    serial_bt->println("Wrong id!");
                    break;
                }

                Model::push_command(Command{ MOTOR_ON, id, 0 });
                serial_bt->print("Motor on: "); serial_bt->println(id);
                break;
            }
            default:
                break;
            }

            xSemaphoreGive(model_changed);
            vTaskDelay(100);
            xSemaphoreTake(model_changed, portMAX_DELAY);
        }
    }


    serial_bt->disconnect();    
}
