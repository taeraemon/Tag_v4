#include <Arduino.h>
#include "BLE_manager.h"
#include "Command_handler.h"  // 명령어 처리 모듈
#include "DeviceConfig.h"
BLEServer  *pServer  = NULL;
BLEService *pService = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;

static bool deviceConnected = false;
static bool oldDeviceConnected = false;

// BLE 인스턴스 시작. SSID 변경 하려면 다른 함수 호출   // TODO : 어떤 함수인지 설명
void StartBLE() {
    DeviceConfig& config = DeviceConfig::getInstance();  // DeviceConfig 싱글톤 인스턴스 가져오기
    const char* ssid = config.getSSID();               // SSID 가져오기
    BLEDevice::init(ssid);                            // BLE 초기화 시 SSID 사용

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());
    
    pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();
    pServer->getAdvertising()->start();
}

void handleBLEConnectionChanges() {
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
}

// BLE 연결 상태를 반환하는 함수
bool isBLEConnected() {
    return deviceConnected;
}

// 동작중에 BLE ssid 변경
void updateBLEDeviceName(const char* name) {
    esp_ble_gap_set_device_name(name);  // BLE advertise 이름 변경
    Serial.print("BLE device name updated to: ");
    Serial.println(name);
}

// BLE advertise interval 설정 함수
void setBLEAdvertisingInterval(int interval) {
    esp_ble_adv_params_t adv_params;
    adv_params.adv_int_min = interval;
    adv_params.adv_int_max = interval;  // min과 max를 동일하게 설정하여 고정 간격 설정

    // 설정된 advertise 파라미터를 이용하여 advertise 시작
    esp_ble_gap_start_advertising(&adv_params);

    Serial.print("BLE advertising interval set to: ");
    Serial.println(interval);
}

// 0~9 값을 받아 BLE 송신 전력 설정 함수
void adjustBLETransmitPower(int level) {
    esp_power_level_t blePowerLevel;

    switch (level) {
        case 9:
            blePowerLevel = ESP_PWR_LVL_P9;  // 9 dBm
            break;
        case 8:
            blePowerLevel = ESP_PWR_LVL_P6;  // 6 dBm
            break;
        case 7:
            blePowerLevel = ESP_PWR_LVL_P3;  // 3 dBm
            break;
        case 6:
            blePowerLevel = ESP_PWR_LVL_N0;  // 0 dBm
            break;
        case 5:
            blePowerLevel = ESP_PWR_LVL_N3;  // -3 dBm
            break;
        case 4:
            blePowerLevel = ESP_PWR_LVL_N6;  // -6 dBm
            break;
        case 3:
            blePowerLevel = ESP_PWR_LVL_N9;  // -9 dBm
            break;
        default:
            blePowerLevel = ESP_PWR_LVL_N12;  // -12 dBm (default)
            break;
    }

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, blePowerLevel);  // Advertise 전력 설정
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, blePowerLevel);  // 스캔 전력 설정

    Serial.print("BLE transmit power adjusted to: ");
    Serial.println(level);
}

// 기본 BLE 알림 전송 함수
void sendNotification(const char* message) {
    if (deviceConnected) {
        pTxCharacteristic->setValue(message);
        pTxCharacteristic->notify();
        Serial.print("Sent notification: ");
        Serial.println(message);
    }
}

// 배터리 상태 전송 함수
void notifyBatteryStatus(int batteryLevel) {
    char buf[50];
    sprintf(buf, "Battery: %d%%", batteryLevel);
    sendNotification(buf);
}

// WiFi 상태 전송 함수
void notifyWiFiStatus(const char* ssid, int rssi) {
    char buf[100];
    sprintf(buf, "WiFi SSID: %s, RSSI: %d dBm", ssid, rssi);
    sendNotification(buf);
}

// 콜백 클래스 멤버 함수 구현 - 명령어 수신
void MyCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    Serial.println("Received data:");
    Serial.println(rxValue.c_str());

    // 수신된 명령어 처리
    if (!rxValue.empty()) {
        processCommand(rxValue);
    }
}

// 연결 상태 콜백
void MyServerCallbacks::onConnect(BLEServer* pServer) {
    deviceConnected = true;
}

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
}
