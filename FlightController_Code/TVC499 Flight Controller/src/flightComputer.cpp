#include <flightComputer.h>
#include <Arduino.h>

bool FlightComputer::initialize() {
    bool success = true;
    Serial.begin(115200);

    success = sensors.initialize(); // Initialize sensors
    // Serial.println("Sensors initialized successfully!");
    success = actuators.initialize(); // Initialize actuators
    // Serial.println("Actuators initialized successfully!");
    success = control.initialize(); // Initialize control system
    // Serial.println("Control system initialized successfully!");
    success = communication.initialize(); // Initialize communication system
    // Serial.println("Communication system initialized successfully!");
    success = logdata.initialize(); // Initialize logging system
    // Serial.println("Logging system initialized successfully!");
    success = state.initialize(); // Initialize state machine
    // Serial.println("State machine initialized successfully!");  
    success = hardware.initialize();
    // Serial.println("Hardware initialized successfully!");

    // initializeBuffer();
    
    // initializeLogging();


    // initializeCommunication(&rf95); // Initialize LoRa communication
    return success;
}

void FlightComputer::update() {
    // Update sensors
    // Serial.println("Looping");
    sensors.updateIMU();
    // Serial.println("Sensors updated successfully!");

    
    // // Update control system
    control.control();
    // Serial.println("Control system updated successfully!");

    // // Update hardware
    hardware.update(); // Update hardware state
    // Serial.println("Hardware updated successfully!");


    state.executeState(); // Execute state machine logic
    // Serial.println("State machine executed successfully!");
}

