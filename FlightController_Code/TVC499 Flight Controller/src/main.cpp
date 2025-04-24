#include <Arduino.h>
#include <FlightComputer.h>



FlightComputer flightComputer; // Flight computer object
// Sensor data arrays


void setup() {
  flightComputer.initialize(); // Initialize flight computer
}
    

void loop() {
  flightComputer.update(); // Update flight computer
}