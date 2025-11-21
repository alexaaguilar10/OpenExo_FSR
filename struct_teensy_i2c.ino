#include <Wire.h>  // Library that allows the board to use I2C communication

// The I2C address of the slave device (ESP32 in this case)
#define SLAVE_ADDR 0x08

// A struct used to store the readings received from the slave.
// It contains two groups (pair1 and pair2), each with two float readings.
typedef struct DataStruct
{
    // First pair of readings
    struct pair1 {
      float reading1;
      float reading2;
    } pair1;

    // Second pair of readings
    struct pair2 {
      float reading1;
      float reading2;
    } pair2;
};

DataStruct fsrData; // Variable that will store the data received from the slave

int msDelay = 5; 
int timeCounter = 0;

// How many packets are expected in 2 seconds
int avgInterval = 2000 / msDelay;

// Variables used to add up readings 
int dataSum1_1 = 0;
int dataSum1_2 = 0;

int dataSum2_1 = 0;
int dataSum2_2 = 0;

// Variables that store the final averages
int dataAvg1_1 = 0;
int dataAvg1_2 = 0;

int dataAvg2_1 = 0;
int dataAvg2_2 = 0;

void setup() {
  Wire.begin();          // Start I2C as a Master device
  Serial.begin(9600);    // Start Serial Monitor at 9600 baud (speed)
  delay(1000);           // Give everything time to power up
  Serial.println("Teensy Master Ready");
}

void loop() {

  // ask the slave device to send sizeof(fsrData) bytes
  // this is how much space our struct takes
  Wire.requestFrom(SLAVE_ADDR, (uint8_t)sizeof(fsrData));

  // check if we received the correct number of bytes
  if (Wire.available() == sizeof(fsrData)) 
  {
    // read all bytes from the I2C buffer into our struct
    // (byte*) converts the struct into a byte pointer so readBytes can fill it
    Wire.readBytes((byte*) &fsrData, sizeof(fsrData));
  }
  else 
  {
    // if bytes do NOT match expected size, something is wrong
    Serial.println("Error: Wrong amount of bytes received.");
  }
  
  // increase time counter 
  timeCounter++;

  // add the readings to the running sum for averaging
  dataSum1_1 += fsrData.pair1.reading1;
  dataSum1_2 += fsrData.pair1.reading2;

  dataSum2_1 += fsrData.pair2.reading1;
  dataSum2_2 += fsrData.pair2.reading2;

  // if we have collected enough packets to equal 2 seconds
  if (timeCounter == avgInterval) 
  {
    // calculate average values (sum ÷ number of samples)
    dataAvg1_1 = dataSum1_1 / timeCounter;
    dataAvg1_2 = dataSum1_2 / timeCounter;

    dataAvg2_1 = dataSum2_1 / timeCounter;
    dataAvg2_2 = dataSum2_2 / timeCounter;

    // reset counters for the next 2-second window
    timeCounter = 0;

    dataSum1_1 = 0;
    dataSum1_2 = 0;

    dataSum2_1 = 0;
    dataSum2_2 = 0;

    // print the average values for Pair 1
    Serial.printf("FSR Pair 1: \n");
    Serial.printf("FSR Reading Avg 1: %d \n", dataAvg1_1);
    Serial.printf("FSR Reading Avg 2: %d \n", dataAvg1_2);
    Serial.println();

    // print the average values for Pair 2
    Serial.printf("FSR Pair 2: \n");
    Serial.printf("FSR Reading Avg 1: %d \n", dataAvg2_1);
    Serial.printf("FSR Reading Avg 2: %d \n", dataAvg2_2);
    Serial.println();
  }

  delay(5);
}

