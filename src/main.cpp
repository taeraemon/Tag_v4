#include <Arduino.h>
#include "soc/soc.h"          // Brownout detector 비활성화를 위해 추가
#include "soc/rtc_cntl_reg.h" // Brownout detector 비활성화를 위해 추가
#include "BLE_manager.h"
#include "WiFi_manager.h"
#include "RTC_manager.h"
#include "Battery_monitor.h"
#include "EEPROM_manager.h"
#include "Command_handler.h"
#include "DeviceConfig.h"
#include "LTE_manager.h"
#include "Transmission_manager.h"
#include <Wire.h> // BMP280 I2C 통신을 위해 추가
#include <Adafruit_BMP280.h> // BMP280 라이브러리 추가

Adafruit_BMP280 bmp; // BMP280 객체 선언 (I2C)
// #define SEALEVELPRESSURE_HPA (1013.25) // 필요시 주석 해제하여 사용

unsigned long previousMillis_scn = 0;
unsigned long previousMillis_bat = 0;

bool isWifiScanSent = false;
bool isServingcellSent = false;
bool isNeighbourcellSent = false;

void setup() {
    // Brownout detector 비활성화 (주의해서 사용할 것!)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    Serial2.begin(115200);

    Wire.begin(); // I2C 통신 시작
    if (!bmp.begin(0x77)) { // BMP280 센서 초기화
        while (1) {
            Serial.println(F("Could not find a valid BMP280 sensor, check wiring or I2C ADDR!"));
            delay(1000);
        }
    }
    Serial.println(F("BMP280 sensor initialized."));

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    // EEPROM 초기화
    initEEPROM();
    initRTC();
    printTime();

    // 배터리 상태 모니터링 초기화
    initBatteryMonitor();

    // DeviceConfig
    DeviceConfig& config = DeviceConfig::getInstance();
    loadDeviceConfigFromEEPROM();

    // BLE 및 WiFi 초기화
    StartBLE();
    StartWiFi();

    Serial.println("Setup complete");
}

void loop() {
    unsigned long currentMillis = millis();
    
    // DeviceConfig 인스턴스 가져오기
    DeviceConfig& config = DeviceConfig::getInstance();

    // BLE 연결 상태 처리
    handleBLEConnectionChanges();

    // BLE 연결 여부와 상관없이 WiFi 스캔 수행 (scan_toggle 상태에 따라)
    if (currentMillis - previousMillis_scn >= (config.getScanInterval() - 6000) && !isWifiScanSent) {
        if (config.getScanToggle() == 1) {  // scan_toggle 상태가 1일 때만 스캔
            ScanAndSend();  // WiFi 스캔
        }
        isWifiScanSent = true;
    }

    // LTE 데이터 수집 및 전송 처리
    if (currentMillis - previousMillis_scn >= (config.getScanInterval() - 2000) && !isServingcellSent) {
        LTE_manager_sendATCommand("AT+QENG=\"servingcell\"\r\n");  // Serving cell 정보 수집
        isServingcellSent = true;
    }

    if (currentMillis - previousMillis_scn >= (config.getScanInterval() - 1000) && !isNeighbourcellSent) {
        LTE_manager_sendATCommand("AT+QENG=\"neighbourcell\"\r\n");  // Neighbour cell 정보 수집
        isNeighbourcellSent = true;
    }

    // 주기적으로 서버로 데이터 전송
    if (currentMillis - previousMillis_scn >= config.getScanInterval()) {
        previousMillis_scn = currentMillis;  // 타이머 리셋
        
        isWifiScanSent = false;
        isServingcellSent = false;
        isNeighbourcellSent = false;

        // BMP280 센서 값 읽기 및 출력
        Serial.println(F("------------ BMP280 Sensor Data ------------"));
        Serial.print(F("Temperature = "));
        float temperature = bmp.readTemperature();
        Serial.print(temperature);
        Serial.println(F(" *C"));

        Serial.print(F("Pressure = "));
        float pressure = bmp.readPressure() / 100.0F;
        Serial.print(pressure); // hPa 단위로 변환
        Serial.println(F(" hPa"));

        Serial.print(F("Approx. Altitude = "));
        float altitude = bmp.readAltitude(1013.25F);
        Serial.print(altitude); // 표준 해수면 기압 (1013.25 hPa) 기준 고도
        Serial.println(F(" m"));
        Serial.println(F("--------------------------------------------"));

        // 주기적으로 TCP 연결하여 데이터 전송
        transmitDataHTTPBaro(altitude);  // 데이터를 패킷화하여 서버로 전송
    }

    if (currentMillis - previousMillis_bat >= 1000) {
        previousMillis_bat = currentMillis;
        
        getAverageBatteryLevel();
    }

    // 시리얼 버퍼 읽기 및 처리
    LTE_manager_readSerialBuffer();
}
