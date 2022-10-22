#include "test_controller.hpp"


void TestController::loop()
{
    extern SemaphoreHandle_t model_changed;

    for (short i = 1; i <= MOTORS_COUNT; i++)
        Model::push_command(Command{ SET_ORIGIN, i, 0 });

    xSemaphoreGive(model_changed);
    taskYIELD();
    xSemaphoreTake(model_changed, portMAX_DELAY);

    Serial.println("🔁 Test cycle begin");
    while (1) {
        for (short i = 1; i <= MOTORS_COUNT; i++)
            Model::push_command(Command{ MOTOR_ON, i, 0 });

        xSemaphoreGive(model_changed);
        taskYIELD();
        xSemaphoreTake(model_changed, portMAX_DELAY);
        vTaskDelay(1000/portTICK_PERIOD_MS);

        Model::motors[1].set_position_by_procent(0.5);
        Model::motors[2].set_position_by_procent(0.5);
        Model::motors[3].set_position_by_procent(0.5);

        xSemaphoreGive(model_changed);
        taskYIELD();
        xSemaphoreTake(model_changed, portMAX_DELAY);
        vTaskDelay(1000/portTICK_PERIOD_MS);

        Model::motors[1].set_position_by_procent(1.0);
        Model::motors[2].set_position_by_procent(1.0);
        Model::motors[3].set_position_by_procent(1.0);

        xSemaphoreGive(model_changed);
        taskYIELD();
        xSemaphoreTake(model_changed, portMAX_DELAY);
        vTaskDelay(1000/portTICK_PERIOD_MS);

        Model::motors[1].set_position_by_procent(0.5);
        Model::motors[2].set_position_by_procent(0.5);
        Model::motors[3].set_position_by_procent(0.5);

        xSemaphoreGive(model_changed);
        taskYIELD();
        xSemaphoreTake(model_changed, portMAX_DELAY);
        vTaskDelay(1000/portTICK_PERIOD_MS);

        Model::motors[1].set_position_by_procent(0.0);
        Model::motors[2].set_position_by_procent(0.0);
        Model::motors[3].set_position_by_procent(0.0);

        xSemaphoreGive(model_changed);
        taskYIELD();
        xSemaphoreTake(model_changed, portMAX_DELAY);
        vTaskDelay(1000/portTICK_PERIOD_MS);

        for (short i = 1; i <= MOTORS_COUNT; i++)
            Model::push_command(Command{ MOTOR_OFF, i, 0 });

        xSemaphoreGive(model_changed);
        taskYIELD();
        xSemaphoreTake(model_changed, portMAX_DELAY);
        vTaskDelay(1000/portTICK_PERIOD_MS);

        vTaskDelay(1);
    }
}
