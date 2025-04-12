#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>
#include "../include/sensors.h"
#include "../include/communication.h"
#include "../include/hardware.h"

// Builtin LED for basic testing
#define LED_BUILTIN 13


// Global objects and variables
Adafruit_BNO08x bno;  // Updated to use BNO08x instead of BNO055
Adafruit_BMP3XX bmp;  // BMP390 altimeter

double quants[4] = {1.0, 0.0, 0.0, 0.0}; // Quaternion data
double gyroRates[3] = {0.0, 0.0, 0.0}; // Gyroscope rates



void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial5.begin(115200);
    delay(2000);  // Wait for serial to initialize

    // Initialize imu and altimeter
    if (!initializeSensors(&bno, &bmp)) {
        Serial.println("Sensor initialization failed!");
        while (1);  // Halt execution if sensors fail to initialize
    }


        
}

void loop(){

  updateIMU(&bno, gyroRates, quants); // Update IMU data

  // Print quaternion data
  Serial.printf("Quaternion: w=%.6f, x=%.6f, y=%.6f, z=%.6f\n", quants[0], quants[1], quants[2], quants[3]);

  //convert to euler angles
  double roll = atan2(2*(quants[0]*quants[1] + quants[2]*quants[3]), 1 - 2*(quants[1]*quants[1] + quants[2]*quants[2]));
  double pitch = asin(2*(quants[0]*quants[2] - quants[3]*quants[1]));
  double yaw = atan2(2*(quants[0]*quants[3] + quants[1]*quants[2]), 1 - 2*(quants[2]*quants[2] + quants[3]*quants[3]));
  Serial.printf("Euler angles: roll=%.6f, pitch=%.6f, yaw=%.6f\n", roll, pitch, yaw);





}