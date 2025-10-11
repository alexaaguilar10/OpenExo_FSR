/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <WiFi.h>

#define FORCE_SENSOR_PIN_1 2 //ESP32 Xiao pin GPIO2 (ADC0)
#define FORCE_SENSOR_PIN_2 3 //ESP32 Xiao pin GPIO3 (ADC0)

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x80, 0xf3, 0xda, 0x5d, 0x95, 0xb8}; //80:f3:da:5d:95:b8

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  // board id
  int id;
  int analogReading1;
  int analogReading2;
} struct_message;

// Create a struct_message called fsrData
struct_message fsrData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // Change this id for each sender
  // Sender 1 --> fsrData.id = 1;
  // Sender 2 --> fsrData.id = 2;
  fsrData.id = 1;

  // Set values to send
  fsrData.analogReading1 = analogRead(FORCE_SENSOR_PIN_1);
  fsrData.analogReading2 = analogRead(FORCE_SENSOR_PIN_2);
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &fsrData, sizeof(fsrData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);
}