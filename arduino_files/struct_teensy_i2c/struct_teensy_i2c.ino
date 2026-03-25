#include <Wire.h>  // Library that allows the board to use I2C communication

// The I2C address of the slave device (ESP32 in this case)
#define SLAVE_ADDR 0x08

// Register map
#define REG_LEFT_HEEL 0x00  // 4 bytes (float)
#define REG_LEFT_TOE 0x04  // 4 bytes
#define REG_RIGHT_HEEL 0x08  // 4 bytes
#define REG_RIGHT_TOE 0x0C  // 4 bytes

#define REG_MAP_SIZE 16    // total register space (4 floats × 4 bytes each)

#define FLOAT_SIZE 4 // size of float, to be used in Wire.write()

// floats to store requested data in
float leftHeel = 0;
float leftToe = 0;
float rightHeel = 0;
float rightToe = 0;

// delay and time counters, for averaging
float msDelay = 5; 
float timeCounter = 0;

// How many packets are expected in 2 seconds
float avgInterval = 2000 / msDelay;

// Variables used to add up readings 
float dataSumLH = 0;
float dataSumLT = 0;

float dataSumRH = 0;
float dataSumRT = 0;

// Variables that store the final averages
float dataAvgLH = 0;
float dataAvgLT = 0;

float dataAvgRH = 0;
float dataAvgRT = 0;

// function (definitely not stolen from the existing I2CHandler file) to make reading values over I2C easier
void read_i2c(uint8_t* ret, uint8_t addr, uint8_t reg, uint8_t len)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  
  if (Wire.endTransmission() != 0)
  {
    Serial.println("Failed to send register.");
  }

  uint8_t bytesRead = Wire.requestFrom(addr, len);
  if (bytesRead != len)
  {
    Serial.println("Not enough bytes received.");
  }

  for (uint8_t i=0; i<len; i++)
  {
      ret[i] = Wire.read();
  }
}


void setup() {
  Wire.begin();          // Start I2C as a Master device
  Serial.begin(9600);    // Start Serial Monitor at 9600 baud (speed)
  delay(2000);           // Give everything time to power up
  Serial.println("Teensy Master Ready");
}

void loop() {

  read_i2c((uint8_t*)&leftHeel, SLAVE_ADDR, REG_LEFT_HEEL, FLOAT_SIZE);
  read_i2c((uint8_t*)&leftToe, SLAVE_ADDR, REG_LEFT_TOE, FLOAT_SIZE);
  read_i2c((uint8_t*)&rightHeel, SLAVE_ADDR, REG_RIGHT_HEEL, FLOAT_SIZE);
  read_i2c((uint8_t*)&rightToe, SLAVE_ADDR, REG_RIGHT_TOE, FLOAT_SIZE);

  // debug option to print raw data
//  Serial.printf("LH: %f  LT: %f  RH: %f  RT: %f\n",
//               leftHeel, leftToe, rightHeel, rightToe);

  // increase time counter 
  timeCounter++;

  // add the readings to the running sum for averaging
  dataSumLH += leftHeel;
  dataSumLT += leftToe;

  dataSumRH += rightHeel;
  dataSumRT += rightToe;

  // if we have collected enough packets to equal 2 seconds
  if (timeCounter == avgInterval) 
  {
    // calculate average values (sum ÷ number of samples)
    dataAvgLH = dataSumLH / timeCounter;
    dataAvgLT = dataSumLT / timeCounter;

    dataAvgRH = dataSumRH / timeCounter;
    dataAvgRT = dataSumRT / timeCounter;

    // reset counters for the next 2-second window
    timeCounter = 0;

    dataSumLH = 0;
    dataSumLT = 0;

    dataSumRH = 0;
    dataSumRT = 0;

    // print the average values for Pair 1
    Serial.printf("FSR Left Pair: \n");
    Serial.printf("Toe Avg: %f \n", dataAvgLH);
    Serial.printf("Heel Avg: %f \n", dataAvgLT);
    Serial.println();

    // print the average values for Pair 2
    Serial.printf("FSR Right Pair: \n");
    Serial.printf("Toe Avg: %f \n", dataAvgRH);
    Serial.printf("Heel Avg: %f \n", dataAvgRT);
    Serial.println();
  }

  delay(5);
}

