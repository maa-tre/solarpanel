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

// --- SENDER CONFIGURATION (CHANGE FOR EACH SENDER) ---
// This is the unique identifier for this specific sender node.
// Change this to '2' for the second sender, '3' for the third, etc.
const int SENDER_ID = 1; // <<< IMPORTANT: CHANGE THIS TO '2' FOR THE SECOND SENDER

// --- Wi-Fi & ESP-NOW Configuration ---
uint8_t centralNodeAddress[] = {0x88, 0x57, 0x21, 0x8E, 0xC2, 0xBC}; // Central Node MAC
const uint8_t COMMON_WIFI_CHANNEL = 1;

// --- Sensor Pin Definitions ---
#define DHTPIN               22 //changed from pin 26
#define DHTTYPE              DHT22
#define LDR_PIN              32
#define THERMISTOR_PIN       33
#define VOLTAGE_SENSOR_PIN   35
#define CURRENT_SENSOR_PIN   34
#define RELAY_PIN            13 // Added: Relay control pin

// --- Thermistor Constants ---
const float THERMISTOR_NOMINAL_RESISTANCE      = 10000.0;
const float THERMISTOR_NOMINAL_TEMP_C          = 25.0;
const float BETA_VALUE                         = 4000.0;
const float FIXED_RESISTOR_VALUE               = 10000.0;
const float TEMPERATURE_CALIBRATION_OFFSET_C = 2.0;

// --- EMPIRICAL CALIBRATION CONSTANTS FOR CURRENT SENSOR ---
// Based on your measurements: 1.49V = 0A | 1.569V = 0.45A
const float V_PIN_ZERO       = 1.490;   
const float REAL_SENSITIVITY = 0.1755;  // (1.569 - 1.490) / 0.45
const float ESP_VREF         = 3.3;     
const float ESP32_ADC_MAX    = 4095.0;  

// --- Voltage Sensor Calibration ---
// Updated: Using direct milliVolt reading and 5x multiplier
const float VOLTAGE_CALIBRATION_OFFSET_V = 0.0;

// --- General Measurement Constants ---
const int NUM_ADC_READINGS_AVG = 50;
const int MEASUREMENT_INTERVAL_MS = 5000;
const int CURRENT_SENSOR_SAMPLES = 500; // For the new current measurement logic

// --- Sensor Objects ---
DHT dht(DHTPIN, DHTTYPE);

// --- Command Structure (for receiving commands from central node) ---
typedef struct struct_command {
    char command[32]; // e.g., "TOGGLE_RELAY", "ACTIVATE_RELAY", "DEACTIVATE_RELAY"
} struct_command;

// --- Data Structure (MUST match Receiver) ---
// We added the 'senderId' to uniquely identify this node.
typedef struct struct_message {
    int   senderId; // ADDED: Unique ID for each sender
    int   ldrValue;
    float dhtTemp;
    float humidity;
    float thermistorTemp;
    float voltage;
    float current; // Now in Amps (not mA)
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

// --- ESP-NOW Receive Callback (for commands from central node) ---
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
    struct_command received_cmd;
    memcpy(&received_cmd, incomingData, sizeof(received_cmd));

    Serial.printf("Command received: %s\n", received_cmd.command);

    if (strcmp(received_cmd.command, "ACTIVATE_RELAY") == 0) {
        Serial.println("Activating relay...");
        digitalWrite(RELAY_PIN, LOW); // Assuming active LOW
        Serial.printf("Relay pin %d set to LOW (activated)\n", RELAY_PIN);
    } else if (strcmp(received_cmd.command, "DEACTIVATE_RELAY") == 0) {
        Serial.println("Deactivating relay...");
        digitalWrite(RELAY_PIN, HIGH); // Off
        Serial.printf("Relay pin %d set to HIGH (deactivated)\n", RELAY_PIN);
    } else if (strcmp(received_cmd.command, "TOGGLE_RELAY") == 0) {
        Serial.println("Toggling relay...");
        digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
        Serial.printf("Relay pin %d state: %s\n", RELAY_PIN, digitalRead(RELAY_PIN) ? "HIGH" : "LOW");
    } else {
        Serial.printf("Unknown command: %s\n", received_cmd.command);
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("\n--- ESP32 Sensor Node (Sender) Starting ---");
    Serial.printf("This is Sender ID: %d\n", SENDER_ID); // Print unique ID

    // Initialize Relay Pin
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); // Default to OFF (assuming active LOW)
    Serial.println("Relay Pin Initialized (default OFF).");

    // Initialize sensors
    dht.begin();
    Serial.println("DHT Sensor Initialized.");

    // Calibration note for current sensor
    Serial.println("Current sensor using empirical calibration:");
    Serial.printf("  Zero-point voltage: %.3f V\n", V_PIN_ZERO);
    Serial.printf("  Sensitivity: %.4f V/A\n", REAL_SENSITIVITY);

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
    esp_now_register_recv_cb(OnDataRecv); // Register receive callback for commands

    memcpy(peerInfo.peer_addr, centralNodeAddress, 6);
    peerInfo.channel = COMMON_WIFI_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer. Restarting...");
        ESP.restart();
    }

    Serial.println("--- ESP-NOW Sender Ready ---");
    Serial.println("Ready to receive relay commands from central node.");
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

        // --- VOLTAGE SENSOR (NEW LOGIC) ---
        // Using analogReadMilliVolts for better accuracy
        float pinMV = analogReadMilliVolts(VOLTAGE_SENSOR_PIN);
        myData.voltage = (pinMV / 1000.0) * 5.0; // Multiply by 5 (1:5 voltage divider)
        
        if (!isnan(myData.voltage)) {
            // Apply calibration offset if needed
            myData.voltage += VOLTAGE_CALIBRATION_OFFSET_V;
        } else {
            Serial.println("Voltage Sensor Error → Sending 0");
        }
        Serial.printf("Voltage: %.2f V (Pin reading: %.1f mV)\n", myData.voltage, pinMV);

        // --- CURRENT SENSOR (NEW EMPIRICAL LOGIC) ---
        // Take multiple samples for noise reduction
        long adcRawSum = 0;
        for(int i = 0; i < CURRENT_SENSOR_SAMPLES; i++) {
            adcRawSum += analogRead(CURRENT_SENSOR_PIN);
            delayMicroseconds(50);
        }
        float avgADC = (float)adcRawSum / CURRENT_SENSOR_SAMPLES;
        
        // Calculate voltage seen at the ESP32 Pin
        float voltageAtPin = (avgADC / ESP32_ADC_MAX) * ESP_VREF;
        
        // Calculate Current using Empirical Sensitivity
        myData.current = (voltageAtPin - V_PIN_ZERO) / REAL_SENSITIVITY;

        // Noise gate: ignore very small currents
        if (abs(myData.current) < 0.03) myData.current = 0.00;
        
        Serial.printf("Current Sensor - Pin Voltage: %.3f V | Current: %.3f A\n", 
                     voltageAtPin, myData.current);
        Serial.printf("Current: %.2f A\n", myData.current);

        // Send via ESP-NOW
        esp_err_t result = esp_now_send(centralNodeAddress, (uint8_t*)&myData, sizeof(myData));
        Serial.println(result == ESP_OK ? "Packet queued." : "Error queuing packet.");
        
        // Print relay status
        Serial.printf("Relay Status: %s\n", digitalRead(RELAY_PIN) ? "OFF (HIGH)" : "ON (LOW)");
    }
}