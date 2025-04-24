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

    communication.sendData(); // Send telemetry data
    
    // Log data
    // logFlightData(sensors.getGyroRates(), sensors.getQuaternions(), sensors.getEulerAngles(), sensors.getAccelerometer(), sensors.getPressure(), sensors.getAltitude(), sensors.getTemperature(), sensors.getState(), sensors.getDt(), hardware.getPyroContinuity1(), hardware.getPyroContinuity2());

  
    // stateMachine(&bno, &bmp, &rf95, STATE, accelerometer, eulerAngles, altData, quaternions, refPressure, launchTriggered, separationTriggered); 
  
  
    // logGlobalData(gyroRates, quaternions, eulerAngles, accelerometer, refPressure, altData, STATE, dt, continuity);
    // sendToLog(&rf95);
  
    // checkForCommands(&rf95); 
}

