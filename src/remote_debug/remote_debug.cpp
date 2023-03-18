#include "remote_debug.hpp"

#define SERVICE_UUID                   "e7112e6c-c396-11ed-afa1-0242ac120002"
#define CHARACTERISTIC_UUID_BEGIN      0xffffff00

void RemoteDebug::loop()
{
    BLE bt;

    bt.begin();

    while(1)
        vTaskDelay(1);
}

void convert(float* input, uint8_t* output, size_t size) {
    // Предполагаем, что размер output равен 4 * size
    for (size_t i = 0; i < size; i++) {
        // Преобразуем указатель на float в указатель на uint8_t
        uint8_t* bytes = reinterpret_cast<uint8_t*>(&input[i]);
        // Копируем 4 байта из bytes в output
        memcpy(&output[4 * i], bytes, 4);
    }
}

void convert_inv(uint8_t* input, float* output, size_t size) {
    // Предполагаем, что размер input равен 4 * size
    for (size_t i = 0; i < size; i++) {
        // Преобразуем указатель на uint8_t в указатель на float
        float* value = reinterpret_cast<float*>(&input[4 * i]);
        // Копируем значение из value в output
        output[i] = *value;
    }
}

bool BLE::begin() {

    // Create the BLE Device
    BLEDevice::init("RDB-1");
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    BLECustomServerCallbacks* callbacks = new BLECustomServerCallbacks(pServer->getAdvertising());
    pServer->setCallbacks(callbacks);

    // Create the BLE Service
    pService = pServer->createService(BLEUUID(SERVICE_UUID), 1+4*2);

    // Security
    BLESecurity *pSecurity = new BLESecurity();
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;

    uint32_t passkey = 2284;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
    pSecurity->setCapability(ESP_IO_CAP_OUT);
    pSecurity->setKeySize(16);
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    // Serial.println(CHARACTERISTIC_UUID_BEGIN + 16*m + i);
    
    // Create characteristics
    BLECharacteristic* pMotorCharacteristic = pService->createCharacteristic(
                                CHARACTERISTIC_UUID_BEGIN,
                                BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
                            );
    pMotorCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
    
    const size_t vars_count = 60;
    float motors_data[vars_count];
    for(uint8_t m = 1; m <= MOTORS_COUNT; m++) {
        for(uint8_t i = 0; i < 5; i++) {
            switch (i)
            {
            case 0:
                motors_data[(m-1)*5 + i] = Model::motors[m].c_pos;
                break;
            case 1:
                motors_data[(m-1)*5 + i] = Model::motors[m].kp;
                break;
            case 2:
                motors_data[(m-1)*5 + i] = Model::motors[m].c_vel;
                break;
            case 3:
                motors_data[(m-1)*5 + i] = Model::motors[m].kd;
                break;
            case 4:
                motors_data[(m-1)*5 + i] = Model::motors[m].c_trq;
                break;
            default:
                break;
            }
        }
    }

    uint8_t data[vars_count*4];
    convert(motors_data, data, vars_count);
    
    pMotorCharacteristic->setValue(data, sizeof(data));
    pService->addCharacteristic(pMotorCharacteristic);

    pService->start();
    pServer->getAdvertising()->start();

    return true;
}

void RemoteDebug::updateModel(SemaphoreHandle_t model_changed) {
    xSemaphoreGive(model_changed);
    vTaskDelay(100);
    xSemaphoreTake(model_changed, portMAX_DELAY);
}