#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>
#include <PWMServo.h>
#include "../include/sensors.h"
#include "../include/communication.h"
#include "../include/hardware.h"
#include "../include/sensors.h"
#include "../include/control.h"

// Builtin LED for basic testing
#define LED_BUILTIN 13
#define BLINK_INTERVAL 500  // Blink every 500ms

#define PYRO1_FIRE 28
#define PYRO2_FIRE 29

// Global objects and variables
Adafruit_BNO08x bno;  // Updated to use BNO08x instead of BNO055
sh2_SensorValue_t sensorValue;
Adafruit_BMP3XX bmp;  // BMP390 altimeter
PWMServo yawServo;  // Yaw servo
PWMServo pitchServo;  // Pitch servo
int yawServoPin = 2;  // Pin for yaw servo
int pitchServoPin = 3;  // Pin for pitch servo

// Sensor data arrays
double gyroRates[3] = {0.0, 0.0, 0.0}; //in radians/sec
double quaternions[4] = {1, 0, 0, 0}; //Quaterinon vector
double eulerAngles[3] = {0.0, 0.0, 0.0}; // Yaw, Pitch, Roll in degrees
double accelerometer[3] = {0.0, 0.0, 0.0}; //accelerometer values, x,y,z
double dt = 0; 
double prevTime = 0;// Current gyro rates
// double gyroOffsets[3] = {0.0, 0.0, 0.0};  // Gyro offsets for calibration


// Define the analog pins
const int analogPin1 = 25;  // A11 on Teensy
const int analogPin2 = 26;  // A12 on Teensy


// Variables to store the analog values
int analogValue1 = 0;
int analogValue2 = 0;

// Variables to store the voltage values
float voltage1 = 0.0;
float voltage2 = 0.0;

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial5.begin(115200);
    // delay(2000);  // Wait for serial to initialize

    prevTime = micros();
    // Serial.println("Teensy Analog Voltage Reader");
    // Serial.println("Reading from pins 25(A11) and 26(A12)");

    //   // Initialize Pyro Pins
    // pinMode(PYRO1_FIRE, OUTPUT);
    // pinMode(PYRO2_FIRE, OUTPUT);
    // digitalWrite(PYRO1_FIRE, LOW);
    // digitalWrite(PYRO2_FIRE, LOW);
    yawServo.attach(yawServoPin);  // Attach yaw servo to pin
    pitchServo.attach(pitchServoPin);  // Attach pitch servo to pin
    initializeSensors(&bno, &bmp);// Initialize sensors
    initializeQuaternions(&bno, quaternions, accelerometer);  //one second
    playAlertTone(1000, 2000);
}

void loop() {
  // Calculate time delta
  double currentTime = micros();
  dt = (currentTime - prevTime) / 1000000.0; // Convert microseconds to seconds
  prevTime = currentTime;

  // Update IMU data
  updateIMU(&bno, gyroRates, quaternions, eulerAngles, accelerometer, dt);
  // initializeQuaternions(&bno, quaternions, accelerometer);  //one second
  
  control(quaternions, gyroRates, pitchServo, yawServo); // Update IMU data
  
  // Serial.printf("dt: %.6f\n", dt);
  // Serial.printf("x: %.6f, y: %.6f, z: %.6f\n", accelerometer[0], accelerometer[1], accelerometer[2]);
  // Serial.printf("Roll: %.6f, Pitch: %.6f, Yaw: %.6f\n", eulerAngles[0], eulerAngles[1], eulerAngles[2]);
}