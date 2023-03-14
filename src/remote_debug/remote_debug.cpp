#include "remote_debug.hpp"

void RemoteDebug::loop()
{
    extern SemaphoreHandle_t model_changed;

    BluetoothSerial* serial_bt = new BluetoothSerial();
    serial_bt->begin("RDB-1");

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

    serial_bt->println("message #1");
    serial_bt->disconnect();    
}
