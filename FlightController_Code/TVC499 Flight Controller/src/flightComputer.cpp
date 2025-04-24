#include <flightComputer.h>
#include <Arduino.h>

bool FlightComputer::initialize() {
    bool success = true;
    Serial.begin(115200);

    success = sensors.initialize(); // Initialize sensors
    success = hardware.initialize();
    success = actuators.initialize(); // Initialize actuators
    success = control.initialize(); // Initialize control system
    success = communication.initialize(); // Initialize communication system
    success = logdata.initialize(); // Initialize logging system
    success = state.initialize(); // Initialize state machine


    // initializeBuffer();
    
    // initializeLogging();


    // initializeCommunication(&rf95); // Initialize LoRa communication

}

void FlightComputer::update() {
    // Update sensors
    sensors.updateIMU();

    
    // Update control system
    control.control();
    
    // Update hardware
    hardware.update(); // Update hardware state


    state.executeState(); // Execute state machine logic
}

