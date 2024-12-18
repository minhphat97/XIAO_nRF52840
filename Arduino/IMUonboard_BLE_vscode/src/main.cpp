/*****************************************************************************/
//  HighLevelExample.ino
//  Hardware:      Grove - 6-Axis Accelerometer&Gyroscope
//	Arduino IDE:   Arduino-1.65
//	Author:	       Lambor
//	Date: 	       Oct,2015
//	Version:       v1.0
//
//  Modified by:
//  Data:
//  Description:
//
//	by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
/*******************************************************************************/

#include "LSM6DS3.h"
#include <ArduinoBLE.h>
#include "Wire.h"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

// Define BLE service and characteristics
BLEService accelService("1810");            // Custom service UUID
BLEFloatCharacteristic xChar("2A56", BLERead | BLENotify);  // X-axis characteristic
BLEFloatCharacteristic yChar("2A57", BLERead | BLENotify);  // Y-axis characteristic
BLEFloatCharacteristic zChar("2A58", BLERead | BLENotify);  // Z-axis characteristic


void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    while (!Serial);
    //Call .begin() to configure the IMUs
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

    // Initialize BLE
    if (!BLE.begin()) {
      Serial.println("Starting BLE failed!");
      while (1);
    }

    // Set BLE device name and service
    BLE.setDeviceName("XIAO-Accel");
    BLE.setLocalName("XIAO-Accel");
    BLE.setAdvertisedService(accelService);

    // Add characteristics to the service
    accelService.addCharacteristic(xChar);
    accelService.addCharacteristic(yChar);
    accelService.addCharacteristic(zChar);

    // Add the service
    BLE.addService(accelService);

    // Start advertising
    BLE.advertise();
    Serial.println("BLE device is now advertising...");
}

void loop() {
    // Wait for a BLE central device to connect
    BLEDevice central = BLE.central();
    if (central) {
      Serial.print("Connected to central: ");
      Serial.println(central.address());

      
      while (central.connected()) {
      
      // Simulated accelerometer data
      float x_raw = myIMU.readFloatAccelX();  // Replace with actual sensor data
      float y_raw = myIMU.readFloatAccelY();
      float z_raw = myIMU.readFloatAccelZ();
      // Update characteristic values
      xChar.writeValue(x_raw);
      yChar.writeValue(y_raw);
      zChar.writeValue(z_raw);

      // Debug output
      Serial.println("Sending accelerometer data:");
      Serial.print("X: ");
      Serial.println(x_raw);
      Serial.print("Y: ");
      Serial.println(y_raw);
      Serial.print("Z: ");
      Serial.println(z_raw);

      delay(200);  // Update every second
    }
    Serial.println("Central disconnected");
  }
    /*
    //Accelerometer
    Serial.print("\nAccelerometer in g:\n");
    Serial.print(" X1 = ");
    Serial.println(myIMU.readFloatAccelX(), 4);
    Serial.print(" Y1 = ");
    Serial.println(myIMU.readFloatAccelY(), 4);
    Serial.print(" Z1 = ");
    Serial.println(myIMU.readFloatAccelZ(), 4);

    //Gyroscope
    Serial.print("\nGyroscope:\n");
    Serial.print(" X1 = ");
    Serial.println(myIMU.readFloatGyroX(), 4);
    Serial.print(" Y1 = ");
    Serial.println(myIMU.readFloatGyroY(), 4);
    Serial.print(" Z1 = ");
    Serial.println(myIMU.readFloatGyroZ(), 4);

    //Thermometer
    Serial.print("\nThermometer:\n");
    Serial.print(" Degrees C1 = ");
    Serial.println(myIMU.readTempC(), 4);
    Serial.print(" Degrees F1 = ");
    Serial.println(myIMU.readTempF(), 4);

    delay(1000); */
}