#include <Wire.h>      // Allows I2C communication
#include <esp_now.h>   // Library for ESP-NOW wireless communication
#include <WiFi.h>      // Required for WiFi/ESP-NOW setup

#define SLAVE_ADDR 0x08  // I2C address this ESP32 will use as a slave

typedef struct DataStruct
{
    // First pair of readings (from board 1)
    struct pair1 {
      float reading1;
      float reading2;
    } pair1;

    // Second pair of readings (from board 2)
    struct pair2 {
      float reading1;
      float reading2;
    } pair2;
};

struct struct_message 
{
    int id;               // ID number of the sending board (1 or 2)
    float analog_reading_1; // First analog reading
    float analog_reading_2; // Second analog reading
};

// packet that Teensy will receive over I2C
DataStruct fsrData;              

// Temporary storage for the latest received ESP-NOW packet
struct_message recvd_fsr_data;   

// These two hold readings from each individual ESP32 sender
struct_message board1_data;      
struct_message board2_data;      

void requestEvent() {

  // load board #1 readings into the struct
  fsrData.pair1.reading1 = board1_data.analog_reading_1;
  fsrData.pair1.reading2 = board1_data.analog_reading_2;

  // load board #2 readings into the struct
  fsrData.pair2.reading1 = board2_data.analog_reading_1;
  fsrData.pair2.reading2 = board2_data.analog_reading_2;

  // send the entire struct as raw bytes over I2C back to the Teensy
  Wire.write((byte*) &fsrData, sizeof(fsrData));
}


void onDataRecv(const uint8_t * mac, const uint8_t * incomingData, int len) {

  // Copy the received bytes into our struct
  memcpy(&recvd_fsr_data, incomingData, sizeof(recvd_fsr_data));

  // Check which sender the packet came from based on ID
  if (recvd_fsr_data.id == 1) {
    // Save to board 1 struct
    board1_data = recvd_fsr_data; 
  }

  else if (recvd_fsr_data.id == 2) {
    // Save to board 2 struct
    board2_data = recvd_fsr_data; 
  }

  else {
    // if ID doesn’t match 1 or 2, something is wrong
    Serial.println("Error reading FSR data: Unknown board ID.");
  }
}

void setup() {

  Serial.begin(9600); // Start serial monitor for debugging messages

  WiFi.mode(WIFI_STA); // ESP-NOW requires WiFi in Station mode (not Access Point)

  // initialize ESP-NOW. If this fails, print an error message.
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW.");
    return;
  }

  // register our callback function so we get notified when new data arrives
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));

  // configure this ESP32 as an I2C Slave using address 0x08
  Wire.begin(SLAVE_ADDR);

  // register function that sends data to the Teensy whenever it requests
  Wire.onRequest(requestEvent);

  Serial.println("ESP32 Slave Ready");
}

void loop() {

}
