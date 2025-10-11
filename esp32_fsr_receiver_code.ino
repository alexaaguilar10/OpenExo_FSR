/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  int analogReading1;
  int analogReading2;
} struct_message;

// Create a struct_message called fsrData
struct_message fsrData;

// create separate structures to hold data from each ESP32
struct_message board1Data; // data from ESP32 sender #1
struct_message board2Data; // data from ESP32 sender #2

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&fsrData, incomingData, sizeof(fsrData));
  Serial.println("-----------------------------");
  Serial.print("Data received from board ID: ");
  Serial.println(fsrData.id);

  // update the correct board’s stored data based on its id
  if (fsrData.id == 1) 
  {
    board1Data = fsrData;
    Serial.println("FSR reading 1: ");
    Serial.println(fsrData.analogReading1);
    Serial.println();
    Serial.println("FSR Reading 2: ");
    Serial.println(fsrData.analogReading2);
    Serial.println();
    
  }
  else if (fsrData.id == 2) 
  {
    board2Data = fsrData; 
    Serial.println("FSR reading 1: ");
    Serial.println(fsrData.analogReading1);
    Serial.println();
    Serial.println("FSR Reading 2: ");
    Serial.println(fsrData.analogReading2);
    Serial.println();
  }
  else 
  {
    // its not reading anything
    Serial.println("Error");
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {

}