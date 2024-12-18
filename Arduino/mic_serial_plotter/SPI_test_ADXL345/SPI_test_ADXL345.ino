#include <SparkFun_ADXL345.h>

/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
ADXL345 adxl = ADXL345(3);           // Use when you want to use Hardware SPI, ADXL345(CS_PIN);
//ADXL345 adxl = ADXL345();             // Use when you need I2C

/****************** VARIABLES ******************/
/*                                             */
int AccelMinX = 0;
int AccelMaxX = 0;
int AccelMinY = 0;
int AccelMaxY = 0;
int AccelMinZ = 0;
int AccelMaxZ = 0; 

int accX = 0;
int accY = 0;
int accZ = 0;

int pitch = 0;
int roll = 0;

/************** DEFINED VARIABLES **************/
/*                                             */
#define offsetX   0       // OFFSET values
#define offsetY   0
#define offsetZ   0

#define gainX     1     // GAIN factors
#define gainY     1
#define gainZ     1 

/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setup()
{
Serial.begin(9600);                 // Start the serial terminal
Serial.println("SparkFun ADXL345 Accelerometer Breakout Calibration");
Serial.println();

adxl.powerOn();                     // Power on the ADXL345

adxl.setRangeSetting(2);           // Give the range settings
                                    // Accepted values are 2g, 4g, 8g or 16g
                                    // Higher Values = Wider Measurement Range
                                    // Lower Values = Greater Sensitivity

adxl.setSpiBit(0);                // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
                                    // It is set to 1 by Default.
                                    // SPI pins on the ATMega328 as reference in SPI Library are 11, 12, and 13

}

/****************** MAIN CODE ******************/
/*  Accelerometer Readings and Min/Max Values  */
void loop()
{
Serial.println("Send any character to display values.");
while (!Serial.available()){}       // Waiting for character to be sent to Serial
Serial.println();

// Get the Accelerometer Readings
int x,y,z;                          // init variables hold results
adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z

if(x < AccelMinX) AccelMinX = x;
if(x > AccelMaxX) AccelMaxX = x;

if(y < AccelMinY) AccelMinY = y;
if(y > AccelMaxY) AccelMaxY = y;

if(z < AccelMinZ) AccelMinZ = z;
if(z > AccelMaxZ) AccelMaxZ = z;

Serial.print("Accel Minimums: "); Serial.print(AccelMinX); Serial.print("  ");Serial.print(AccelMinY); Serial.print("  "); Serial.print(AccelMinZ); Serial.println();
Serial.print("Accel Maximums: "); Serial.print(AccelMaxX); Serial.print("  ");Serial.print(AccelMaxY); Serial.print("  "); Serial.print(AccelMaxZ); Serial.println();
Serial.println();


/* Note: Must perform offset and gain calculations prior to seeing updated results
/  Refer to SparkFun ADXL345 Hook Up Guide: https://learn.sparkfun.com/tutorials/adxl345-hookup-guide
/  offsetAxis = 0.5 * (Acel+1g + Accel-1g)
/  gainAxis = 0.5 * ((Acel+1g - Accel-1g)/1g) */

// UNCOMMENT SECTION TO VIEW NEW VALUES
//accX = (x - offsetX)/gainX;         // Calculating New Values for X, Y and Z
//accY = (y - offsetY)/gainY;
//accZ = (z - offsetZ)/gainZ;

//Serial.print("New Calibrated Values: "); Serial.print(accX); Serial.print("  "); Serial.print(accY); Serial.print("  "); Serial.print(accZ);
//Serial.println(); 

while (Serial.available())
{
    Serial.read();                    // Clear buffer
}
}
