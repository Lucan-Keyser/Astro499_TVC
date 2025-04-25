#include <Arduino.h>
#include <FlightComputer.h>


FlightComputer flightComputer; // Flight computer object
// Sensor data arrays

void setup() {
  flightComputer.initialize();
  // Serial.println("Flight computer initialized successfully!");
}
    

void loop() {
  // Serial.println("Main start");
  flightComputer.update(); // Update flight computer
}