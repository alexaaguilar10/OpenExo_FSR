/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

/* 
 * StructMessage
 * -------------
 * Structure holds the sensor readings that each ESP32 board sends.
 *   - id: identifier for the board (1 for Board #1, 2 for Board #2)
 *   - analog_reading_1: the first sensor value read from the board
 *   - analog_reading_2: the second sensor value read from the board
 *
 * The sender and receiver must have the same structure definitions,
 * otherwise the data could be interpreted incorrectly.
 */
struct StructMessage 
{
    int id;
    int analog_reading_1;
    int analog_reading_2;
};


// Create a struct_message called fsr_data
struct_message fsr_data;

// create separate structures to hold data from each ESP32
struct_message board1_data; // data from ESP32 sender #1
struct_message board2_data; // data from ESP32 sender #2

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&fsr_data, incomingData, sizeof(fsr_data));
  Serial.println("-----------------------------");
  Serial.print("Data received from board ID: ");
  Serial.println(fsr_data.id);

  // check which board sent the data based on the board ID
  if (fsr_data.id == 1) 
  {
    board1_data = fsr_data;
    // print both sensor readings from board #1
    Serial.println("FSR reading 1: ");
    Serial.println(fsr_data.analogReading1);
    Serial.println();
    Serial.println("FSR Reading 2: ");
    Serial.println(fsr_data.analogReading2);
    Serial.println();
    
  }
  else if (fsr_data.id == 2) 
  {
    board2_data = fsr_data; 
    // print both sensor readings from board #2
    Serial.println("FSR reading 1: ");
    Serial.println(fsr_data.analogReading1);
    Serial.println();
    Serial.println("FSR Reading 2: ");
    Serial.println(fsr_data.analogReading2);
    Serial.println();
  }
  else 
  {
    // if the ID doesn’t match known boards, report an error
    Serial.println("Error");
  }
}

/* 
 * This function prepares the device for receiving data via ESP-NOW.
 *
 * Steps:
 *   1. Start the Serial Monitor for debugging and output.
 *   2. Set the ESP32 to Wi-Fi Station mode (needed for ESP-NOW communication).
 *   3. Initialize ESP-NOW and check for errors.
 *   4. Register the data-receiving callback function.
 */
void setup() 
{
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

// This function runs continuously after setup() finishes.
void loop() 
{

}