/** communication.cpp
* ===========================================================
* Name: Flight Controller Communication Implementation
* Section: TVC499
* Project: Flight Controller
* Purpose: LoRa radio communication and serial interface
* ===========================================================
*/

#include "../include/communication.h"
#include "../include/hardware.h"
#include "../include/config.h"




bool Communication::initialize() {
    bool success = true;
    Serial.begin(115200); // Initialize serial communication for debugging
    Serial5.begin(115200); // Initialize serial port for logging
    //Serial.println("Initializing communication...");
    success = initializeLoRa(); // Initialize LoRa radio
    return success;
}

bool Communication::initializeLoRa() {
    bool success = true;
    if (!rf95.init()) {
        Serial.println("RF95 LoRa init failed!");
        success = false;
    }
    
    // Configure radio parameters https://www.rfwireless-world.com/calculators/LoRa-Data-Rate-Calculator.html

    rf95.setFrequency(RF95_FREQ);
    rf95.setTxPower(23, false);
    rf95.setSpreadingFactor(9);
    rf95.setSignalBandwidth(500000);
    rf95.setCodingRate4(5);
    
    Serial.println("LoRa radio initialized");
    return success;
}

String Communication::checkForCommands() {
    String command = ""; 

    if (rf95.available()) {
        // Buffer for received message
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; // Buffer for received message 251 bytes
        uint8_t len = sizeof(buf);
        
        // Read the message
        if (rf95.recv(buf, &len)) { //recv returns true if a message is received, buf is the buffer, len is the length of the message
            buf[len] = '\0';  // Null-terminate buffer so we can treat it as a string
            String receivedCommand = String((char*)buf); 
            receivedCommand.trim(); // Remove any leading/trailing whitespace
            
            Serial.print("LoRa received: "); 
            Serial.println(receivedCommand);
            
            command = receivedCommand;
            }
        } else if (Serial.available()) { // Check if data is available on Serial5
            String receivedCommand = Serial.readStringUntil('\n'); // Read the command until newline character
            receivedCommand.trim(); // Remove any leading/trailing whitespace
            
            Serial.print("Serial received: "); 
            Serial.println(receivedCommand);
            
            command = receivedCommand;
        } else {
            command = ""; // No command received
        }
    
    return command; 
}


void Communication::sendData(int state, int sdLogState) {
    // Check if enough time has passed since the last telemetry send
    if (millis() - lastTelemetryTime < TELEMETRY_INTERVAL) {
        return; // Not enough time has passed, skip sending telemetry
    } else {

        TelemetryData data;
        data.roll = sensors.getEulerAngles()[0]; // Get roll angle from sensors
        data.pitch = sensors.getEulerAngles()[1]; // Get pitch angle from sensors
        data.yaw = sensors.getEulerAngles()[2]; // Get yaw angle from sensors
        data.altitude = sensors.getAltitude(); // Get altitude from sensors
        data.pitchServo = actuators.getServoAngles()[0]; // Get pitch servo angle from actuators
        data.yawServo = actuators.getServoAngles()[1]; // Get yaw servo angle from actuators
        data.continuity1 =  hardware.getPyroContinuity1(); // Get continuity 1 value from hardware
        data.continuity2 =  hardware.getPyroContinuity2(); // Get continuity 2 value from hardware
        data.dt = sensors.getDt(); // Get time delta from sensors
      
        data.state = state;
        data.sdLogState = sdLogState;
      
              
        rf95.send((uint8_t*)&data, sizeof(data));
      

        // Update the last send time
        lastTelemetryTime = millis(); // Update the last send time
    }
    
}

void Communication::sendDataNoDelay(int state, int sdLogState) {

    // Create telemetry data structure
    TelemetryData data;
    data.roll = sensors.getEulerAngles()[0]; // Get roll angle from sensors
    data.pitch = sensors.getEulerAngles()[1]; // Get pitch angle from sensors
    data.yaw = sensors.getEulerAngles()[2]; // Get yaw angle from sensors
    data.altitude = sensors.getAltitude(); // Get altitude from sensors
    data.pitchServo = actuators.getServoAngles()[0]; // Get pitch servo angle from actuators
    data.yawServo = actuators.getServoAngles()[1]; // Get yaw servo angle from actuators
    data.continuity1 =  hardware.getPyroContinuity1(); // Get continuity 1 value from hardware
    data.continuity2 =  hardware.getPyroContinuity2(); // Get continuity 2 value from hardware
    data.dt = sensors.getDt(); // Get time delta from sensors
  
    data.state = state;
    data.sdLogState = sdLogState;
  
          
    rf95.send((uint8_t*)&data, sizeof(data));
}
    