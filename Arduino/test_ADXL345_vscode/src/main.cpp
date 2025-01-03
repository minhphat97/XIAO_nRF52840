//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>

//Assign the Chip Select signal to pin 10. 
int CS = 1;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;  //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1
char chip_ID = 0x00;
//This buffer will hold values read from the ADXL345 registers.

//These variables will be used to hold the x,y and z axis accelerometer values.

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value) {
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values) {
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if (numBytes > 1)address = address | 0x40;

  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for (int i = 0; i < numBytes; i++) {
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}


void setup() {
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.beginTransaction(SPISettings(1000000, LSBFIRST, SPI_MODE3)); // modify back to MSBFIRST
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);

  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);

  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode
}

void loop() {
  char values_dataX0[10], values_dataX1[10], values_dataY0[10], values_dataY1[10], values_dataZ0[10], values_dataZ1[10], value_ID[3];
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 1, values_dataX0);
  readRegister(DATAX1, 1, values_dataX1);
  readRegister(DATAY0, 1, values_dataY0);
  readRegister(DATAY1, 1, values_dataY1);
  readRegister(DATAZ0, 1, values_dataZ0);
  readRegister(DATAZ1, 1, values_dataZ1);
  readRegister(chip_ID, 1, value_ID);
  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values_dataX1[0] and values_dataX0[0].
  int16_t rawX = ((int16_t)values_dataX1[0] << 8) | values_dataX0[0];
  float accelX = rawX * 0.00390625; 
  //The Y value is stored in values_dataY1[0] and values_dataY0[0].
  int16_t rawY = ((int16_t)values_dataY1[0] << 8) | values_dataY0[0];
  float accelY = rawY * 0.00390625; 
  //The Z value is stored in values_dataZ1[0] and values_dataZ0[0].
  int16_t rawZ = ((int16_t)values_dataZ1[0] << 8) | values_dataZ0[0];
  float accelZ = rawZ * 0.00390625; 
  int16_t chipID = (int16_t)value_ID[0];
  //Print the results to the terminal.
  Serial.print("Acceleration X: ");
        Serial.print(accelX);
        Serial.println(" g");
  Serial.print("Acceleration Y: ");
        Serial.print(accelY);
        Serial.println(" g");
  Serial.print("Acceleration Z: ");
        Serial.print(accelZ);
        Serial.println(" g");
  Serial.print("Chip_ID: ");
        Serial.print(chipID);
  delay(500);
}


