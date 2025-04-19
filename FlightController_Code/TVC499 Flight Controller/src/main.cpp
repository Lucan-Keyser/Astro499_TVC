#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>
#include <PWMServo.h>
#include "../include/sensors.h"
#include "../include/communication.h"
#include "../include/hardware.h"
#include "../include/sensors.h"
#include "../include/control.h"
#include "../include/state.h"
#include "../include/logging.h"
#include "../include/config.h"

// Builtin LED for basic testing
#define LED_BUILTIN 13
#define BLINK_INTERVAL 500  // Blink every 500ms

#define PYRO1_FIRE 28
#define PYRO2_FIRE 29

// Global objects and variables
Adafruit_BNO08x bno;  // Updated to use BNO08x instead of BNO055
sh2_SensorValue_t sensorValue;
RH_RF95 rf95;  // LoRa radio object
Adafruit_BMP3XX bmp;  // BMP390 altimeter
PWMServo yawServo;  // Yaw servo
PWMServo pitchServo;  // Pitch servo
int STATE = 0;
int yawServoPin = 2;  // Pin for yaw servo
int pitchServoPin = 3;  // Pin for pitch servo

// Sensor data arrays
double gyroRates[3] = {0.0, 0.0, 0.0}; //in radians/sec
double quaternions[4] = {1, 0, 0, 0}; //Quaterinon vector
double eulerAngles[3] = {0.0, 0.0, 0.0}; // Yaw, Pitch, Roll in degrees
double accelerometer[3] = {0.0, 0.0, 0.0}; //accelerometer values, x,y,z
double refPressure = 1000; // Reference pressure in hPa
double altData[3] = {0.0, 0.0, 0.0}; // Altitude data [altitude (m), pressure (PA), temperature]
double dt = 0; 
double prevTime = 0;// Current previous loop time
// double gyroOffsets[3] = {0.0, 0.0, 0.0};  // Gyro offsets for calibration
double lastSendTime = 0; // Last time telemetry was sent


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
    // pinMode(BNO_RESET_PIN, OUTPUT);
    // delay(1000);
    // digitalWrite(PYRO2_FIRE, LOW);
    yawServo.attach(yawServoPin);  // Attach yaw servo to pin
    pitchServo.attach(pitchServoPin);  // Attach pitch servo to pin
    playAlertTone(5000, 2000);
    initializeCommunication(&rf95); // Initialize LoRa communication
    delay(500);  // Wait for LoRa to initialize
    initializeSensors(&bno, &bmp, quaternions, accelerometer, refPressure);// Initialize sensors
    playAlertTone(1000, 2000);
}

void loop() {

  // Calculate time delta
  double currentTime = micros();
  dt = (currentTime - prevTime) / 1000000.0; // Convert microseconds to seconds
  prevTime = currentTime;

  // Update IMU data
  updateIMU(&bno, gyroRates, quaternions, eulerAngles, accelerometer, dt);

  //control attitude
  stateMachine(&bno, &bmp, STATE, accelerometer, eulerAngles, altData, quaternions, refPressure); // Update state machine

  control(quaternions, gyroRates, pitchServo, yawServo); // Update IMU data
  logGlobalData (gyroRates, quaternions, eulerAngles, accelerometer, refPressure, altData, STATE, dt);
  if (millis() - lastSendTime > TELEMETRY_INTERVAL) {
    sendToLog(&rf95);
  }
  lastSendTime = millis();


   // Update altitude data

  // Serial.print("Altitude: ");
  // Serial.println(altData[0]); // Print altitude
  // // initializeQuaternions(&bno, quaternions, accelerometer);  //one second
  Serial.printf("dt: %.6f\n", dt);
  // Serial.printf("x: %.6f, y: %.6f, z: %.6f\n", accelerometer[0], accelerometer[1], accelerometer[2]);
  // Serial.printf("x: %.6f, y: %.6f, z: %.6f\n", gyroRates[0], gyroRates[1], gyroRates[2]);
  // Serial.printf("Roll: %.6f, Pitch: %.6f, Yaw: %.6f\n", eulerAngles[0], eulerAngles[1], eulerAngles[2]);
}