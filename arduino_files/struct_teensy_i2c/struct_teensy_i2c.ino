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

// How many packets are expected in X seconds
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
    return;
  }

  // delay slightly to ensure esp32 has received the correct register
  delay(10);

  uint8_t bytesRead = Wire.requestFrom(addr, len);
  if (bytesRead != len)
  {
    Serial.println("Not enough bytes received.");
    return;
  }

  for (uint8_t i=0; i<len; i++)
  {
      ret[i] = Wire.read();
  }
}

// Read the entire register map in a single transaction (safer, avoids repeated small transactions)
void read_i2c_block(uint8_t* ret, uint8_t addr, uint8_t startReg, uint8_t len)
{
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission() != 0)
  {
    Serial.println("Failed to set start register for block read.");
    return;
  }

  // short delay to let slave prepare
  delay(2);

  uint8_t bytesRead = Wire.requestFrom(addr, len);
  if (bytesRead != len)
  {
    Serial.print("Block read: expected "); Serial.print(len);
    Serial.print(" bytes but got "); Serial.println(bytesRead);
    return;
  }

  for (uint8_t i = 0; i < len; ++i) {
    ret[i] = Wire.read();
  }
}

float read_wireless(uint8_t addr, uint8_t reg, uint8_t len)
{
  uint8_t dataBlock[len];                        // initialize temporary array to store values, size = len
  read_i2c_block(dataBlock, addr, 0x00, len);    // read block of data over i2c, store in dataBlock

  float requestedVal = 0;                        // initialize temporary float to store requested data point
  memcpy(&requestedVal, &dataBlock[reg], 4);     // copy data from requested register in dataBlock over to requestedVal
  // debug option to print values within read_wireless
  // Serial.print("Requested register "); Serial.print(reg);
  // Serial.print(", fetched value: "); Serial.print(requestedVal);
  // Serial.println();

  return requestedVal;
}


void setup() 
{
  Wire.begin();          // Start I2C as a Master device
  Serial.begin(9600);    // Start Serial Monitor at 9600 baud (speed)
  delay(2000);           // Give everything time to power up
  Serial.println("Teensy Master Ready");
}

void loop() 
{
  // use read_wireless to avoid register-pointer races, while still retaining capability to request specific values
  leftHeel = read_wireless(SLAVE_ADDR, REG_LEFT_HEEL, REG_MAP_SIZE);
  leftToe = read_wireless(SLAVE_ADDR, REG_LEFT_TOE, REG_MAP_SIZE);
  rightHeel = read_wireless(SLAVE_ADDR, REG_RIGHT_HEEL, REG_MAP_SIZE);
  rightToe = read_wireless(SLAVE_ADDR, REG_RIGHT_TOE, REG_MAP_SIZE);
  

  // debug option to print raw data
  // Serial.printf("LH: %f  LT: %f  RH: %f  RT: %f\n",
  //             leftHeel, leftToe, rightHeel, rightToe);

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
    Serial.printf("Toe Avg: %f \n", dataAvgLT);
    Serial.printf("Heel Avg: %f \n", dataAvgLH);
    Serial.println();

    // print the average values for Pair 2
    Serial.printf("FSR Right Pair: \n");
    Serial.printf("Toe Avg: %f \n", dataAvgRT);
    Serial.printf("Heel Avg: %f \n", dataAvgRH);
    Serial.println();
  }

  delay(5);
}

