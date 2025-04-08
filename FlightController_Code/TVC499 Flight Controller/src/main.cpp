#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>
#include "../include/sensors.h"
#include "../include/communication.h"
#include "../include/hardware.h"

// Builtin LED for basic testing
#define LED_BUILTIN 13
#define BLINK_INTERVAL 500  // Blink every 500ms

#define PYRO1_FIRE 28
#define PYRO2_FIRE 29

// Global objects and variables
Adafruit_BNO08x bno;  // Updated to use BNO08x instead of BNO055
Adafruit_BMP3XX bmp;  // BMP390 altimeter

// Sensor data arrays
double gyroRates[3] = {0.0, 0.0, 0.0};    // Current gyro rates
double gyroOffsets[3] = {0.0, 0.0, 0.0};  // Gyro offsets for calibration


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
    delay(2000);  // Wait for serial to initialize

        
    Serial.println("Teensy Analog Voltage Reader");
    Serial.println("Reading from pins 25(A11) and 26(A12)");

      // Initialize Pyro Pins
    pinMode(PYRO1_FIRE, OUTPUT);
    pinMode(PYRO2_FIRE, OUTPUT);
    digitalWrite(PYRO1_FIRE, LOW);
    digitalWrite(PYRO2_FIRE, LOW);

}

void loop(){
    //updateIMU(&bno, gyroRates, gyroOffsets);  // Update IMU data

      // Read the analog values
    analogValue1 = analogRead(analogPin1);
    analogValue2 = analogRead(analogPin2);

    // Convert analog values to voltage (Teensy default reference is 3.3V)
    // Analog reading is 10-bit (0-1023)
    voltage1 = analogValue1 * (3.3 / 1023.0);
    voltage2 = analogValue2 * (3.3 / 1023.0);

    // Print the results to Serial monitor
    Serial.print("Pin 25(A11): ");
    Serial.print(analogValue1);
    Serial.print(" raw, ");
    Serial.print(voltage1);
    Serial.print("V | Pin 26(A12): ");
    Serial.print(analogValue2);
    Serial.print(" raw, ");
    Serial.print(voltage2);
    Serial.println("V");

    // Small delay to make the output readable
    delay(500);



}