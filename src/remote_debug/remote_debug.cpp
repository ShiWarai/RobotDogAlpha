#include "remote_debug.hpp"

#define SERVICE_UUID                   "e7112e6c-c396-11ed-afa1-0242ac120002"
#define CHARACTERISTIC_UUID_BEGIN      0xffffff00

void RemoteDebug::loop()
{
    this->begin();

    while(1) {
        vTaskDelay(1);
    }
        
}

bool RemoteDebug::begin() {

    // Create the BLE Device
    BLEDevice::init("RDB-1");
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    BLECustomServerCallbacks* callbacks = new BLECustomServerCallbacks(pServer->getAdvertising());
    pServer->setCallbacks(callbacks);

    // Create the BLE Service
    pService = pServer->createService(BLEUUID(SERVICE_UUID), 1+3*2);

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
    
    // Create characteristics
    pMotorsCurrentCharacteristic = pService->createCharacteristic(
                                CHARACTERISTIC_UUID_BEGIN,
                                BLECharacteristic::PROPERTY_READ
                            );
    pMotorsCurrentCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
    pMotorsCurrentCharacteristic->setCallbacks(new BLEReadCharacteristicCallbacks());
    uploadModel(pMotorsCurrentCharacteristic);

    pMotorsTargetCharacteristic = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID_BEGIN + 1,
                                        BLECharacteristic::PROPERTY_WRITE
                                    );
    pMotorsTargetCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
    pMotorsTargetCharacteristic->setCallbacks(new BLEWriteCharacteristicCallbacks());

    pService->start();
    pServer->getAdvertising()->start();

    return true;
}