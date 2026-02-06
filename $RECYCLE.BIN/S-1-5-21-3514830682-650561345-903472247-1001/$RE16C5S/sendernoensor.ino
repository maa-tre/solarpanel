// FILE: ESP32_Multi_Sensor_Node_Sender.ino
// Merged code for a multi-sensor ESP32 node that reads from
// DHT22, LDR, Thermistor, Voltage Sensor, and ACS712 Current Sensor,
// then sends the data via ESP-NOW.
//
// This version has been modified to include a unique SENDER_ID
// in the data packet, allowing a central receiver to handle
// data from multiple sender nodes.

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Required for esp_wifi_set_channel()
#include <DHT.h>      // For DHT22 Temperature/Humidity Sensor
#include <math.h>     // For log() in Thermistor calculation
#include <ACS712.h>   // For ACS712 Current Sensor

// --- SENDER CONFIGURATION (CHANGE FOR EACH SENDER) ---
// This is the unique identifier for this specific sender node.
// Change this to '2' for the second sender, '3' for the third, etc.
const int SENDER_ID = 1; // <<< IMPORTANT: CHANGE THIS TO '2' FOR THE SECOND SENDER

// --- Wi-Fi & ESP-NOW Configuration ---
uint8_t centralNodeAddress[] = {0x10, 0x52, 0x1C, 0xA7, 0x54, 0x08}; // Central Node MAC
const uint8_t COMMON_WIFI_CHANNEL = 11; // Must match Central Node/router 2.4GHz channel

// --- Sensor Pin Definitions ---
#define DHTPIN               4
#define DHTTYPE              DHT22
#define LDR_PIN              36
#define THERMISTOR_PIN       34
#define VOLTAGE_SENSOR_PIN   39
#define CURRENT_SENSOR_PIN   13

// --- Thermistor Constants ---
const float THERMISTOR_NOMINAL_RESISTANCE      = 10000.0;
const float THERMISTOR_NOMINAL_TEMP_C          = 25.0;
const float BETA_VALUE                         = 4000.0;
const float FIXED_RESISTOR_VALUE               = 10000.0;
const float TEMPERATURE_CALIBRATION_OFFSET_C = 2.0;

// --- ACS712 Current Sensor Constants ---
// 5A version: 185 mV/A, 20A version: 100 mV/A, 30A version: 66 mV/A
const float currentSensorSensitivity = 100; // 20A module

// --- ESP32 ADC Constants ---
const float ESP32_ADC_MAX = 4095.0;
const float ESP32_V_REF   = 3.3;

// --- Voltage Sensor Calibration ---
const float SENSOR_VOLTAGE_DIVIDER_RATIO = 7.5757;
const float VOLTAGE_CALIBRATION_OFFSET_V = -1.75;

// --- General Measurement Constants ---
const int NUM_ADC_READINGS_AVG = 50;
const int MEASUREMENT_INTERVAL_MS = 5000;

// --- Sensor Objects ---
DHT dht(DHTPIN, DHTTYPE);
ACS712 ACS(CURRENT_SENSOR_PIN, ESP32_V_REF, (int)ESP32_ADC_MAX, (int)currentSensorSensitivity);

// --- Data Structure (MUST match Receiver) ---
// We added the 'senderId' to uniquely identify this node.
typedef struct struct_message {
    int   senderId; // ADDED: Unique ID for each sender
    int   ldrValue;
    float dhtTemp;
    float humidity;
    float thermistorTemp;
    float voltage;
    float current;
    bool  valid;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// --- ESP-NOW Send Callback ---
void OnDataSent(const esp_now_send_info_t* send_info, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status to ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", send_info->des_addr[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.print(" : ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("Packet delivery failed! Confirm channel and receiver status.");
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("\n--- ESP32 Sensor Node (Sender) Starting ---");
    Serial.printf("This is Sender ID: %d\n", SENDER_ID); // Print unique ID

    // Initialize sensors
    dht.begin();
    Serial.println("DHT Sensor Initialized.");

    Serial.println("Calibrating ACS712 (no current flow)...");
    delay(2000);
    ACS.autoMidPoint();
    Serial.printf("Calibration complete. Zero Offset: %.0f ADC\n", ACS.getMidPoint());

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    Serial.println("ADC Configured.");

    // Wi-Fi & ESP-NOW setup
    WiFi.mode(WIFI_STA);
    Serial.print("Sender MAC Address: ");
    Serial.println(WiFi.macAddress());

    esp_wifi_set_channel(COMMON_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("ESP-NOW Channel set to: %u\n", COMMON_WIFI_CHANNEL);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW. Restarting...");
        ESP.restart();
    }

    esp_now_register_send_cb(OnDataSent);

    memcpy(peerInfo.peer_addr, centralNodeAddress, 6);
    peerInfo.channel = COMMON_WIFI_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer. Restarting...");
        ESP.restart();
    }

    Serial.println("--- ESP-NOW Sender Ready ---");
}

// --- Loop ---
unsigned long lastMeasurementTime = 0;

void loop() {
    if (millis() - lastMeasurementTime >= MEASUREMENT_INTERVAL_MS) {
        lastMeasurementTime = millis();
        Serial.printf("\n--- Reading Sensors for Sender %d ---\n", SENDER_ID);

        // Reset everything to 0 at the start
        myData.senderId = SENDER_ID;
        myData.ldrValue = 0;
        myData.dhtTemp = 0;
        myData.humidity = 0;
        myData.thermistorTemp = 0;
        myData.voltage = 0;
        myData.current = 0;
        myData.valid = true;

        // --- LDR ---
        int ldrRaw = analogRead(LDR_PIN);
        if (!isnan(ldrRaw)) myData.ldrValue = ldrRaw;
        Serial.printf("LDR: %d\n", myData.ldrValue);

        // --- DHT22 ---
        float dhtTempRead = dht.readTemperature();
        float dhtHumidityRead = dht.readHumidity();
        if (!isnan(dhtTempRead) && !isnan(dhtHumidityRead)) {
            myData.dhtTemp = dhtTempRead;
            myData.humidity = dhtHumidityRead;
        } else {
            Serial.println("DHT Error → Sending 0s");
        }
        Serial.printf("DHT Temp: %.2f C | Humidity: %.2f %%\n", myData.dhtTemp, myData.humidity);

        // --- Thermistor ---
        long sumThermistorADC = 0;
        for (int i = 0; i < NUM_ADC_READINGS_AVG; i++) {
            sumThermistorADC += analogRead(THERMISTOR_PIN);
            delayMicroseconds(100);
        }
        float avgThermistorADC = sumThermistorADC / (float)NUM_ADC_READINGS_AVG;
        if (avgThermistorADC > 0 && avgThermistorADC < ESP32_ADC_MAX) {
            float resistance = FIXED_RESISTOR_VALUE * ((ESP32_ADC_MAX / avgThermistorADC) - 1.0);
            float temp_k_inv = (1.0 / (THERMISTOR_NOMINAL_TEMP_C + 273.15)) +
                               (log(resistance / THERMISTOR_NOMINAL_RESISTANCE) / BETA_VALUE);
            myData.thermistorTemp = (1.0 / temp_k_inv) - 273.15 + TEMPERATURE_CALIBRATION_OFFSET_C;
        } else {
            Serial.println("Thermistor Error → Sending 0");
        }
        Serial.printf("Thermistor Temp: %.2f C\n", myData.thermistorTemp);

        // --- Voltage Sensor ---
        long sumVoltageADC = 0;
        for (int i = 0; i < NUM_ADC_READINGS_AVG; i++) {
            sumVoltageADC += analogRead(VOLTAGE_SENSOR_PIN);
            delayMicroseconds(100);
        }
        float avgVoltageADC = sumVoltageADC / (float)NUM_ADC_READINGS_AVG;
        float espVoltage = avgVoltageADC / ESP32_ADC_MAX * ESP32_V_REF;
        float voltageCalc = (espVoltage * SENSOR_VOLTAGE_DIVIDER_RATIO) + VOLTAGE_CALIBRATION_OFFSET_V;
        if (!isnan(voltageCalc)) {
            myData.voltage = voltageCalc;
        } else {
            Serial.println("Voltage Sensor Error → Sending 0");
        }
        Serial.printf("Voltage: %.2f V\n", myData.voltage);

        // --- Current Sensor ---
        // Use high oversampling (1000+) to filter ESP32 ADC noise
        float mA = ACS.mA_DC(1000); 

        // Basic noise gate: If reading is less than 20mA, show 0
        if (abs(mA) < 20) mA = 0;

        if (!isnan(mA)) {
            myData.current = mA;
        } else {
            Serial.println("Current Sensor Error → Sending 0");
            myData.current = 0;
        }
        Serial.print("Current: ");
        Serial.print(mA / 1000.0, 3); // Display in Amps
        Serial.println(" A");

        // Send via ESP-NOW
        esp_err_t result = esp_now_send(centralNodeAddress, (uint8_t*)&myData, sizeof(myData));
        Serial.println(result == ESP_OK ? "Packet queued." : "Error queuing packet.");
    }
}
