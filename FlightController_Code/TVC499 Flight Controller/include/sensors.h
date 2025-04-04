/** sensors.h
* ===========================================================
* Name: Flight Controller Sensors Interface
* Section: TVC499
* Project: Flight Controller
* Purpose: Sensor data acquisition and processing
* ===========================================================
*/

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BLE_UART.h>
#include <utility/imumaths.h>

// Constants
#define REF_PRESSURE_HPA 700  // Default reference pressure in hPa

/**
 * @brief Initialize IMU and altimeter sensors
 * @param bno Pointer to BNO055 sensor object
 * @param bmp Pointer to BMP3XX sensor object
 * @return True if initialization successful, false otherwise
 */
bool initializeSensors(Adafruit_BNO055* bno, Adafruit_BMP3XX* bmp);

/**
 * @brief Get orientation, gyro, acceleration, and gravity data from IMU
 * @param bno Pointer to BNO055 sensor object
 * @param gyroRates Array to store gyroscope rates [x,y,z]
 * @param gyroOffsets Array to store gyroscope offsets [x,y,z]
 * param localAngles Array to store local orientation angles [x,y,z]
 * param linearAccel Array to store linear acceleration [x,y,z]
 * param accelData Array to store accelerometer data [x,y,z]
 * param gravData Array to store gravity data [x,y,z]
 * @param dt Time delta since last reading (seconds)
 */
void updateIMU(Adafruit_BNO055* bno, double gyroRates[3], double gyroOffsets[3]);

/**
 * @brief Get altitude and pressure data from altimeter
 * @param bmp Pointer to BMP3XX sensor object
 * @param altData Array to store altitude, pressure, and temperature [altitude,pressure,temp]
 * @param refPressure Reference pressure for altitude calculation
 * @return True if reading successful, false otherwise
 */
bool updateAltimeter(Adafruit_BMP3XX* bmp, double altData[3], float refPressure);

/**
 * @brief Extract data from sensor event into data array
 * @param event Pointer to sensor event structure
 * @param data Array to store extracted data [x,y,z]
 */
void returnData(sensors_event_t* event, double data[3]);

/**
 * @brief Reset and calibrate all sensors
 * @param bno Pointer to BNO055 sensor object
 * @param bmp Pointer to BMP3XX sensor object
 * @param altData Array to store altitude data
 * @param refPressure Pointer to reference pressure value
 * @param gyroOffsets Array to store gyroscope offsets [x,y,z]
 */
void resetSensors(Adafruit_BNO055* bno, Adafruit_BMP3XX* bmp, double altData[3], float* refPressure, double gyroOffsets[3]);


/**
 * @brief IMU zeroing function to reset gyro data
 * @param bno Pointer to BNO055 sensor object
 * @param gyroOffsets Array to store gyroscope offsets [x,y,z]
 */
void zeroIMU(Adafruit_BNO055* bno, double gyroOffsets[3]);

/**
 * @brief Set reference pressure for altitude calculation
 * @param bmp Pointer to BMP3XX sensor object
 * @param altData Array to store altitude data
 * @param refPressure Pointer to reference pressure value
 */
void zeroAltimeter(Adafruit_BMP3XX* bmp, double altData[3], float* refPressure);

#endif // SENSORS_H