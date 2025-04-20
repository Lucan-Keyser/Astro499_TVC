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
#include "../include/ringbuffer.h"

// Global objects and variables
Adafruit_BNO08x bno;  // Updated to use BNO08x instead of BNO055
sh2_SensorValue_t sensorValue;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_BMP3XX bmp;  // BMP390 altimeter
PWMServo yawServo;  // Yaw servo
PWMServo pitchServo;  // Pitch servo
int STATE = 0;


// Sensor data arrays
double gyroRates[3] = {0.0, 0.0, 0.0}; //in radians/sec
double quaternions[4] = {1, 0, 0, 0}; //Quaterinon vector
double eulerAngles[3] = {0.0, 0.0, 0.0}; // Yaw, Pitch, Roll in degrees
double accelerometer[3] = {0.0, 0.0, 0.0}; //accelerometer values, x,y,z
double refPressure = 1000; // Reference pressure in hPa
double altData[3] = {0.0, 0.0, 0.0}; // Altitude data [altitude (m), pressure (PA), temperature]
double continuity[2] = {0.0, 0.0}; // Initialize continuity array
double dt = 0; 
double prevTime = 0;// Current previous loop time
double lastSendTime = 0; 
double eventStartTime = millis();

bool launchTriggered = false; // Launch trigger flag
bool separationTriggered = false; // Separation trigger flag


void setup() {
    Serial.begin(115200);
    Serial5.begin(115200);

    prevTime = micros();

    initializeBuffer();
    
    initializeLogging();

    initializeHardware(separationTriggered, launchTriggered); // Initialize hardware components

    pitchServo.attach(PITCH_SERVO_PIN);  // Attach pitch servo to pin J2
    yawServo.attach(YAW_SERVO_PIN);  // Attach yaw servo to pin J1

    initializeCommunication(&rf95); // Initialize LoRa communication
    delay(100);  // Wait for LoRa to initialize

    initializeSensors(&bno, &bmp, quaternions, accelerometer, refPressure);// Initialize sensors

    playAlertTone(1000, 2000); // Play alert tone for 2 seconds let the user know we are initializing

    double gimbalInit[2] = {0.0, 0.0}; // Initialize gimbal position

    moveServos(gimbalInit, pitchServo, yawServo); // Initialize gimbal position
}

void loop() {
  double currentTime = micros();
  dt = (currentTime - prevTime) / 1000000.0; 
  prevTime = currentTime;

  checkPyroContinuity(continuity); 

  updateIMU(&bno, gyroRates, quaternions, eulerAngles, accelerometer, dt);

  stateMachine(&bno, &bmp, &rf95, STATE, accelerometer, eulerAngles, altData, quaternions, refPressure, launchTriggered, separationTriggered); 

  if(STATE == 3) { // Ascending state hit the acceleration threshold
    control(quaternions, gyroRates, pitchServo, yawServo);
  }

  logGlobalData(gyroRates, quaternions, eulerAngles, accelerometer, refPressure, altData, STATE, dt, continuity);
  sendToLog(&rf95);

  checkForCommands(&rf95); 
}