// FILE: ESP32_Multi_Sensor_Node_Sender.ino
// Second sender with updated voltage/current logic
// Uses same calibration as first sender

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <DHT.h>
#include <math.h>

// --- SENDER CONFIGURATION ---
const int SENDER_ID = 2; // IMPORTANT: This is different!

// --- Wi-Fi & ESP-NOW Configuration ---
uint8_t centralNodeAddress[] = {0x88, 0x57, 0x21, 0x8E, 0xC2, 0xBC};
const uint8_t COMMON_WIFI_CHANNEL = 1;

// --- Sensor Pin Definitions ---
#define DHTPIN               4      // DIFFERENT from first sender
#define DHTTYPE              DHT22
#define LDR_PIN              35     // DIFFERENT from first sender
#define THERMISTOR_PIN       34     // DIFFERENT from first sender
#define VOLTAGE_SENSOR_PIN   33     // DIFFERENT from first sender
#define CURRENT_SENSOR_PIN   32     // DIFFERENT from first sender
#define RELAY_PIN            2

// --- Thermistor Constants ---
const float THERMISTOR_NOMINAL_RESISTANCE = 10000.0;
const float THERMISTOR_NOMINAL_TEMP_C = 25.0;
const float BETA_VALUE = 4000.0;
const float FIXED_RESISTOR_VALUE = 10000.0;
const float TEMPERATURE_CALIBRATION_OFFSET_C = 2.0;

// --- USE SAME CALIBRATION AS FIRST SENDER ---
const float V_PIN_ZERO = 1.490;      // Same as first sender
const float REAL_SENSITIVITY = 0.1755; // Same as first sender
const float ESP_VREF = 3.3;
const float ESP32_ADC_MAX = 4095.0;

// --- Voltage Sensor Calibration ---
// May need different offset for second sender
const float VOLTAGE_CALIBRATION_OFFSET_V =0; // Adjust if needed -1.75V
// --- General Measurement Constants ---
const int NUM_ADC_READINGS_AVG = 50;
const int MEASUREMENT_INTERVAL_MS = 5000;
const int CURRENT_SENSOR_SAMPLES = 500;

// --- Sensor Objects ---
DHT dht(DHTPIN, DHTTYPE);

// Data structures (same as before)
typedef struct struct_command {
    char command[32];
} struct_command;

typedef struct struct_message {
    int   senderId;
    int   ldrValue;
    float dhtTemp;
    float humidity;
    float thermistorTemp;
    float voltage;
    float current; // In Amps
    bool  valid;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// Callback functions (same as before)
void OnDataSent(const esp_now_send_info_t* send_info, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
    struct_command received_cmd;
    memcpy(&received_cmd, incomingData, sizeof(received_cmd));
    Serial.printf("Command received: %s\n", received_cmd.command);
    
    if (strcmp(received_cmd.command, "TOGGLE_RELAY") == 0) {
        digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
        Serial.printf("Relay toggled. State: %s\n", digitalRead(RELAY_PIN) ? "HIGH" : "LOW");
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("\n--- ESP32 Sensor Node (Sender 2) Starting ---");
    Serial.printf("Sender ID: %d\n", SENDER_ID);

    // Initialize pins
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    dht.begin();

    // Wi-Fi setup
    WiFi.mode(WIFI_STA);
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());

    // ESP-NOW setup
    //esp_wifi_set_channel(COMMON_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_channel(COMMON_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("ESP-NOW Channel set to: %u\n", COMMON_WIFI_CHANNEL);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed. Restarting...");
        ESP.restart();
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    memcpy(peerInfo.peer_addr, centralNodeAddress, 6);
    peerInfo.channel = COMMON_WIFI_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer. Restarting...");
        ESP.restart();
    }

    Serial.println("--- Sender 2 Ready ---");
}

unsigned long lastMeasurementTime = 0;

void loop() {
    if (millis() - lastMeasurementTime >= MEASUREMENT_INTERVAL_MS) {
        lastMeasurementTime = millis();
        
        // --- Read sensors ---
        myData.senderId = SENDER_ID;
        myData.valid = true;

        // LDR
        myData.ldrValue = analogRead(LDR_PIN);
        
        // DHT22
        myData.dhtTemp = dht.readTemperature();
        myData.humidity = dht.readHumidity();
        if (isnan(myData.dhtTemp) || isnan(myData.humidity)) {
            myData.valid = false;
        }

        // Thermistor
        long sumThermistor = 0;
        for (int i = 0; i < NUM_ADC_READINGS_AVG; i++) {
            sumThermistor += analogRead(THERMISTOR_PIN);
            delayMicroseconds(100);
        }
        float avgADC = sumThermistor / (float)NUM_ADC_READINGS_AVG;
        if (avgADC > 0 && avgADC < ESP32_ADC_MAX) {
            float resistance = FIXED_RESISTOR_VALUE * ((ESP32_ADC_MAX / avgADC) - 1.0);
            float temp_k_inv = (1.0 / (THERMISTOR_NOMINAL_TEMP_C + 273.15)) +
                               (log(resistance / THERMISTOR_NOMINAL_RESISTANCE) / BETA_VALUE);
            myData.thermistorTemp = (1.0 / temp_k_inv) - 273.15 + TEMPERATURE_CALIBRATION_OFFSET_C;
        } else {
            myData.valid = false;
        }

        // --- VOLTAGE (New Logic) ---
        float pinMV = analogReadMilliVolts(VOLTAGE_SENSOR_PIN);
        myData.voltage = (pinMV / 1000.0) * 5.0 + VOLTAGE_CALIBRATION_OFFSET_V;

        // --- CURRENT (New Logic) - Using same calibration as sender 1 ---
        long adcSum = 0;
        for(int i = 0; i < CURRENT_SENSOR_SAMPLES; i++) {
            adcSum += analogRead(CURRENT_SENSOR_PIN);
            delayMicroseconds(50);
        }
        float avgCurrentADC = adcSum / (float)CURRENT_SENSOR_SAMPLES;
        float voltageAtPin = (avgCurrentADC / ESP32_ADC_MAX) * ESP_VREF;
        myData.current = (voltageAtPin - V_PIN_ZERO) / REAL_SENSITIVITY;
        
        // Noise filter
        if (abs(myData.current) < 0.03) myData.current = 0.00;

        // Debug output
        Serial.printf("--- Sender %d ---\n", SENDER_ID);
        Serial.printf("LDR: %d, Temp: %.1fC, Hum: %.1f%%\n", 
                     myData.ldrValue, myData.dhtTemp, myData.humidity);
        Serial.printf("Voltage: %.2fV, Current: %.3fA\n", 
                     myData.voltage, myData.current);

        // Send data
        esp_now_send(centralNodeAddress, (uint8_t*)&myData, sizeof(myData));
    }
}