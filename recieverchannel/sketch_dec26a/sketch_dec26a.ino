// FILE: ESP32_Central_Node_Receiver_Gateway.ino
// Updated for Ngrok HTTPS support
// This ESP32 acts as a central node, receiving data from multiple
// sensor nodes via ESP-NOW and forwarding the aggregated data
// to a Flask server via HTTPS (ngrok).

#include <esp_now.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>  // Added for HTTPS support
#include <map>
#include <ArduinoJson.h>
#include "esp_wifi.h" // For setting Wi-Fi channel explicitly

// --- Wi-Fi Credentials ---
const char* ssid     = "Acerhotspot"; // <<< Your hotspot SSID
const char* password = "123456780"; // <<< Your hotspot password
//const char* ssid     = "OnePlus";
//const char* password = "aaaaaaaa"; // <<< Your hotspot password

// --- Fixed Wi-Fi Channel ---
const uint8_t FIXED_CHANNEL = 11; // Both ESP-NOW and Wi-Fi must match
const char* flaskServerUrl = "http://192.168.1.69:5000/log_data"; //computer ip
const char* commandServerBaseUrl = "http://192.168.1.69:5000/get-command/"; // Base URL for commands */

// --- Ngrok Server URL (HTTPS) ---
// Replace with your actual ngrok URL
//const char* flaskServerUrl = "https://24d8d774b99d.ngrok-free.app/log_data";
//const char* commandServerBaseUrl = "https://24d8d774b99d.ngrok-free.app/get-command/";

// --- MAC Addresses of Sender Devices ---
// IMPORTANT: Replace with the actual MAC addresses of your sender devices
uint8_t sender1_mac[] = {0x5C, 0x01, 0x3B, 0x4C, 0xD3, 0x18}; // <<< UPDATE THIS WITH SENDER 1 MAC
uint8_t sender2_mac[] = {0x10, 0x52, 0x1C, 0xA7, 0x54, 0x08}; // <<< UPDATE THIS WITH SENDER 2 MAC

// --- Structure to hold sensor data ---
typedef struct struct_message {
    int   senderId;
    int   ldrValue;
    float dhtTemp;
    float humidity;
    float thermistorTemp;
    float voltage;
    float current;
    bool  valid;
} struct_message;

// --- Structure for commands sent TO senders ---
typedef struct struct_command {
    char command[32]; // e.g., "TOGGLE_RELAY"
} struct_command;

// Store both sensor data and timestamp
struct SenderData {
    struct_message data;
    unsigned long lastReceivedTimestamp;
};

// Map: senderId -> latest data
std::map<int, SenderData> incomingDataMap;

// Data send timing
unsigned long lastFlaskSendTime = 0;
unsigned long flaskSendInterval = 2000; // 2s (reduced for faster dashboard updates)
unsigned long senderTimeoutInterval = 25000; // 25s
unsigned long lastCommandPollTime = 0;
const unsigned long commandPollInterval = 5000; // 5s

// Create WiFiClientSecure object for HTTPS
WiFiClientSecure client;

// --- ESP-NOW Receive Callback ---
void OnDataRecv(const esp_now_recv_info* recv_info, const uint8_t* incomingDataPtr, int len) {
    struct_message tempIncomingData;
    memcpy(&tempIncomingData, incomingDataPtr, sizeof(tempIncomingData));

    incomingDataMap[tempIncomingData.senderId].data = tempIncomingData;
    incomingDataMap[tempIncomingData.senderId].lastReceivedTimestamp = millis();

    Serial.printf(
        "Data received from Sender ID: %d | MAC: %02X:%02X:%02X:%02X:%02X:%02X\n"
        "  -> LDR: %d, DHT Temp: %.2f C, Humidity: %.2f %%\n"
        "  -> Thermistor Temp: %.2f C, Voltage: %.2f V, Current: %.2f mA, Valid: %s\n",
        tempIncomingData.senderId,
        recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
        recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5],
        tempIncomingData.ldrValue,
        tempIncomingData.dhtTemp,
        tempIncomingData.humidity,
        tempIncomingData.thermistorTemp,
        tempIncomingData.voltage,
        tempIncomingData.current,
        tempIncomingData.valid ? "True" : "False"
    );
}

// --- ESP-NOW Send Callback ---
void OnDataSent(const esp_now_send_info_t* send_info, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status to ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", send_info->des_addr[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.print(" : ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// --- Send Aggregated Data to Flask via HTTPS ---
void sendAggregatedDataToFlask() {
    if (WiFi.status() == WL_CONNECTED) {
        if (incomingDataMap.empty()) {
            Serial.println("No data received yet. Skipping POST request.");
            return;
        }

        HTTPClient http;
        
        // Configure HTTPS client
        client.setInsecure(); // Skip certificate verification for ngrok
        client.setTimeout(10000); // 10 second timeout
        
        http.begin(client, flaskServerUrl);
        http.addHeader("Content-Type", "application/json");

        const int capacity = JSON_ARRAY_SIZE(incomingDataMap.size()) + incomingDataMap.size() * JSON_OBJECT_SIZE(8);
        DynamicJsonDocument jsonDoc(capacity);
        JsonArray records = jsonDoc.to<JsonArray>();

        std::vector<int> sendersToRemove;

        for (auto const& [senderId, senderData] : incomingDataMap) {
            if (millis() - senderData.lastReceivedTimestamp < senderTimeoutInterval) {
                JsonObject record = records.createNestedObject();
                record["senderId"]           = senderData.data.senderId;
                record["ldrValue"]           = senderData.data.ldrValue;
                record["dhtTemp"]            = senderData.data.dhtTemp;
                record["humidity"]           = senderData.data.humidity;
                record["thermistorTemp"]     = senderData.data.thermistorTemp;
                record["voltage"]            = senderData.data.voltage;
                record["current"]            = senderData.data.current;
                record["valid"]              = senderData.data.valid;
                record["gateway_timestamp_ms"] = millis();
            } else {
                Serial.printf("Data from sender ID %d is stale. Removing from map.\n", senderId);
                sendersToRemove.push_back(senderId);
            }
        }
        
        for (int id : sendersToRemove) {
            incomingDataMap.erase(id);
        }

        if (records.size() == 0) {
            Serial.println("All data is stale. Skipping POST request.");
            http.end();
            return;
        }

        String jsonPayload;
        serializeJson(jsonDoc, jsonPayload);

        Serial.print("Sending to Ngrok (HTTPS): ");
        Serial.println(jsonPayload);

        int httpResponseCode = http.POST(jsonPayload);

        if (httpResponseCode > 0) {
            Serial.printf("[HTTPS] POST... code: %d\n", httpResponseCode);
            if (httpResponseCode == HTTP_CODE_OK || httpResponseCode == HTTP_CODE_CREATED) {
                String payload = http.getString();
                Serial.printf("Response from server: %s\n", payload.c_str());
            }
        } else {
            Serial.printf("[HTTPS] POST failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
        }

        http.end();
    } else {
        Serial.println("WiFi not connected. Cannot send data to Flask.");
    }
}

// --- Send Command to a Specific Sender ---
void sendCommandToSender(int senderId, const char* command) {
    struct_command cmd_to_send;
    strncpy(cmd_to_send.command, command, sizeof(cmd_to_send.command) - 1);
    cmd_to_send.command[sizeof(cmd_to_send.command) - 1] = '\0'; // Ensure null termination

    uint8_t* target_mac = nullptr;
    if (senderId == 1) {
        target_mac = sender1_mac;
    } else if (senderId == 2) {
        target_mac = sender2_mac;
    } else {
        Serial.printf("No MAC address registered for sender ID: %d\n", senderId);
        return;
    }

    esp_err_t result = esp_now_send(target_mac, (uint8_t *) &cmd_to_send, sizeof(cmd_to_send));
   
    if (result == ESP_OK) {
        Serial.printf("Command '%s' sent to sender %d successfully.\n", command, senderId);
    } else {
        Serial.printf("Error sending command '%s' to sender %d.\n", command, senderId);
    }
}

// --- Poll Server for Commands for a Specific Station via HTTPS ---
void pollForCommandsForStation(int stationId) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        
        // Build the URL for this specific station
        String url = String(commandServerBaseUrl) + String(stationId);
        
        Serial.printf("Polling for commands for station %d...\n", stationId);
        
        // Reconfigure client for each request
        client.setInsecure();
        client.setTimeout(5000);
        
        http.begin(client, url);
        int httpResponseCode = http.GET();

        if (httpResponseCode == HTTP_CODE_OK) {
            String payload = http.getString();
            Serial.printf("Command received for station %d: %s\n", stationId, payload.c_str());

            DynamicJsonDocument doc(256);
            DeserializationError error = deserializeJson(doc, payload);

            if (error) {
                Serial.print("deserializeJson() failed: ");
                Serial.println(error.c_str());
                http.end();
                return;
            }

            // Use .as<int>() for safer type conversion
            int station_id = doc["station_id"].as<int>();
            const char* command = doc["command"];
            
            if (station_id > 0 && command) {
                // Double-check station ID matches
                if (station_id == stationId) {
                    sendCommandToSender(station_id, command);
                } else {
                    Serial.printf("Warning: Station ID mismatch. Expected %d, got %d\n", stationId, station_id);
                }
            }

        } else if (httpResponseCode == HTTP_CODE_NO_CONTENT) {
            Serial.printf("No pending commands for station %d.\n", stationId);
        } else {
            Serial.printf("[HTTPS] GET failed for station %d, error: %s\n", stationId, http.errorToString(httpResponseCode).c_str());
        }
        http.end();
    } else {
        Serial.println("WiFi not connected. Cannot poll for commands.");
    }
}

// --- Poll Server for Commands for All Stations ---
void pollForCommands() {
    // Poll for commands for station 1
    pollForCommandsForStation(1);
    
    // Add a small delay between polls if needed
    delay(100);
    
    // Poll for commands for station 2
    pollForCommandsForStation(2);
}

// --- Test Ngrok Connection ---
void testNgrokConnection() {
    Serial.println("Testing ngrok connection...");
    
    HTTPClient http;
    client.setInsecure();
    client.setTimeout(5000);
    
    String testUrl = "https://b606e1da1602.ngrok-free.app/get_stations";
    http.begin(client, testUrl);
    
    int httpCode = http.GET();
    Serial.printf("Test connection to ngrok: %d\n", httpCode);
    
    if (httpCode > 0) {
        String response = http.getString();
        Serial.println("Response: " + response);
    } else {
        Serial.printf("Connection test failed: %s\n", http.errorToString(httpCode).c_str());
    }
    
    http.end();
}

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("ESP32 Central Node (Receiver/Gateway) Starting...");
    Serial.println("This gateway handles stations: 1 and 2");

    WiFi.mode(WIFI_STA);

    // Force Wi-Fi & ESP-NOW to channel 11
    esp_wifi_set_channel(FIXED_CHANNEL, WIFI_SECOND_CHAN_NONE);

    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected.");
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("ESP32 MAC Address: ");
    Serial.println(WiFi.macAddress());

    // Verify actual channel
    wifi_second_chan_t secondCh;
    uint8_t primaryCh;
    esp_wifi_get_channel(&primaryCh, &secondCh);
    Serial.printf("Connected on channel: %d\n", primaryCh);

    // Test ngrok connection
    testNgrokConnection();


    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        ESP.restart();
    }

    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);

    // Register sender peers
    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = FIXED_CHANNEL;
    peerInfo.encrypt = false;
    
    // Register sender 1
    memcpy(peerInfo.peer_addr, sender1_mac, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer for sender 1");
        ESP.restart();
    }
    Serial.println("Registered sender 1");

    // Register sender 2
    memcpy(peerInfo.peer_addr, sender2_mac, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer for sender 2");
        ESP.restart();
    }
    Serial.println("Registered sender 2");

    Serial.println("ESP-NOW Receiver/Gateway setup complete. Waiting for sensor data...");
}

void loop() {
    if (millis() - lastFlaskSendTime > flaskSendInterval) {
        sendAggregatedDataToFlask();
        lastFlaskSendTime = millis();
    }

    if (millis() - lastCommandPollTime > commandPollInterval) {
        pollForCommands();
        lastCommandPollTime = millis();
    }
}