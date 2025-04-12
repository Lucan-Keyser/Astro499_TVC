#include <Arduino.h>
#include <Wire.h>
#include "../include/sensors.h"

// Global objects and variables
Adafruit_BNO08x bno;  // BNO085 IMU

// Data arrays
double quants[4] = {1.0, 0.0, 0.0, 0.0};  // Quaternion [w, x, y, z]
double gyroRates[3] = {0.0, 0.0, 0.0};    // Gyro rates [roll, pitch, yaw]
double eulerAngles[3] = {0.0, 0.0, 0.0};  // Euler angles [roll, pitch, yaw]


void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    delay(2000);  // Wait for serial to initialize
    
    Serial.println("Quaternion and Euler Angle Test");

    // Initialize IMU
    Wire.begin();
    Wire.setClock(400000); // Set to 400kHz I2C speed
    
    if (!bno.begin_I2C(0x4A, &Wire)) {
        Serial.println("Failed to find BNO085 sensor");
        while (1) { delay(10); }
    }
    
    // Enable the gyroscope reports
    if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
        Serial.println("Could not enable gyroscope reports");
        while (1) { delay(10); }
    }
    
    Serial.println("BNO085 initialized successfully");
    Serial.println("Keep the device still for 3 seconds to calibrate");
    delay(3000);
}

void loop() {
    // Update IMU data and quaternion
    updateIMU(&bno, gyroRates, quants);


}