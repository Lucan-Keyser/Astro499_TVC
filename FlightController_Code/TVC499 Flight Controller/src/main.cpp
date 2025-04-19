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
double lastSendTime = 0; // Last time telemetry was sent
String command = ""; // Command received from LoRa or Serial
bool separationTriggered = false; // Flag to track if separation has been triggered
bool launchTriggered = false; // Flag to track if launch has been triggered

// Define the analog pins
const int analogPin1 = 25;  // A11 on Teensy
const int analogPin2 = 26;  // A12 on Teensy

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial5.begin(115200);

    prevTime = micros();
    
    // Initialize hardware pins
    yawServo.attach(yawServoPin);  // Attach yaw servo to pin
    pitchServo.attach(pitchServoPin);  // Attach pitch servo to pin
    
    // Play startup tone
    playAlertTone(5000, 2000);
    
    // Initialize LoRa communication
    if (initializeCommunication(&rf95)) {
        Serial.println("LoRa communication initialized successfully");
    } else {
        Serial.println("WARNING: LoRa initialization failed");
    }
    
    delay(500);  // Wait for LoRa to initialize
    
    // Initialize sensors
    if (initializeSensors(&bno, &bmp, quaternions, accelerometer, refPressure)) {
        Serial.println("Sensors initialized successfully");
    } else {
        Serial.println("WARNING: Sensor initialization failed");
    }
    
    playAlertTone(1000, 2000);
    lastSendTime = millis(); // Initialize last send time
}

void loop() {
    // Calculate time delta
    double currentTime = micros();
    dt = (currentTime - prevTime) / 1000000.0; // Convert microseconds to seconds
    prevTime = currentTime;

    // Update IMU data
    updateIMU(&bno, gyroRates, quaternions, eulerAngles, accelerometer, dt);
    Serial.print("Gyro Rates: ");
    Serial.print(gyroRates[0]); Serial.print(", ");
    Serial.print(gyroRates[1]); Serial.print(", ");
    Serial.print(gyroRates[2]); Serial.println(" rad/s");


    // Update altitude data periodically (not every cycle to save time)
    if (STATE == UNPOWERED_ASCENT || (millis() % 100 < 10)) {  // In unpowered ascent or every ~100ms
        updateAltimeter(&bmp, altData, refPressure);
    }

    // Check for commands (both serial and LoRa)
    checkForCommands(&rf95, &command);
    readSerial(&command, &separationTriggered, &launchTriggered);
    
    // Control attitude based on current state
    stateMachine(&bno, &bmp, STATE, accelerometer, eulerAngles, altData, quaternions, refPressure);
    
    // Apply control algorithm to stabilize rocket
    control(quaternions, gyroRates, pitchServo, yawServo);
    
    // Log all global data for telemetry
    logGlobalData(gyroRates, quaternions, eulerAngles, accelerometer, refPressure, altData, STATE, dt);
    
    // Send telemetry at the specified interval without blocking
    if (millis() - lastSendTime >= 500) {
        Serial.println("Sending telemetry data...");
        lastSendTime = millis();
        if (sendToLog(&rf95)) {
            Serial.println("Telemetry sent successfully");
            // Only update the timer if we successfully queued a packet
            lastSendTime = millis();
        }
    }
    
    // Optional debug output (can be disabled for performance)
    // Serial.printf("dt: %.6f\n", dt);
}