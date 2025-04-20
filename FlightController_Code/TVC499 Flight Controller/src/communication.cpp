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

double lastTelemetryTime = millis(); // Last time telemetry was sent
#pragma pack(push, 1)
struct TelemetryData {
  float yaw; // Yaw angle in degrees
  float pitch; // Pitch angle in degrees
  float roll; // Roll angle in degrees
  float altitude; // Altitude in meters
  float yawServo; // Yaw servo angle in degrees
  float pitchServo; // Pitch servo angle in degrees
  float continuity1; // Continuity 1 value - 0 or 1 
  float continuity2; // Continuity 2 value - 0 or 1
  float dt; // Time delta in seconds for main loop
  float state; // Flight state - 0 to 7
  float serialBoolean; // Serial SD boolean value - 0 1 or 2
};

bool initializeCommunication(RH_RF95* rf95) {

    if (!rf95->init()) {
        Serial.println("RF95 LoRa init failed!");
        return false;
    }
    
    // Configure radio parameters https://www.rfwireless-world.com/calculators/LoRa-Data-Rate-Calculator.html

    rf95->setFrequency(RF95_FREQ);
    rf95->setTxPower(23, false);
    rf95->setSpreadingFactor(9);
    rf95->setSignalBandwidth(500000);
    rf95->setCodingRate4(5);
    
    Serial.println("LoRa radio initialized");
    return true;
}

String checkForCommands(RH_RF95* rf95) {
    String command = ""; 

    if (rf95->available()) {
        // Buffer for received message
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; // Buffer for received message 251 bytes
        uint8_t len = sizeof(buf);
        
        // Read the message
        if (rf95->recv(buf, &len)) { //recv returns true if a message is received, buf is the buffer, len is the length of the message
            buf[len] = '\0';  // Null-terminate buffer so we can treat it as a string
            String receivedCommand = String((char*)buf); 
            receivedCommand.trim(); // Remove any leading/trailing whitespace
            
            Serial.print("LoRa received: "); 
            Serial.println(receivedCommand);
            
            command = receivedCommand;
            }
        }
    return command; 
}


void sendData(RH_RF95* rf95, double eulerAngles[3], double altData[3], double pitchServoAngle, double yawServoAngle, double continuity1, double continuity2, double dt, int state, double serialBoolean) {
    // Check if enough time has passed since the last telemetry send
    if (millis() - lastTelemetryTime < TELEMETRY_INTERVAL) {
        return; // Not enough time has passed, skip sending telemetry
    } else {


        // Create telemetry data structure
        TelemetryData data;
        data.roll = eulerAngles[0];
        data.pitch = eulerAngles[1];
        data.yaw = eulerAngles[2];
        data.altitude = altData[0];
        data.pitchServo = pitchServoAngle;
        data.yawServo = yawServoAngle;
        data.continuity1 =  continuity1;
        data.continuity2 =  continuity2;
        data.dt = dt;
        data.state = state;
        data.serialBoolean = serialBoolean;

        
        rf95->send((uint8_t*)&data, sizeof(data));

        // Update the last send time
        lastTelemetryTime = millis(); // Update the last send time
    }
    
}

void sendDataNoDelay(RH_RF95* rf95, double eulerAngles[3], double altData[3], double pitchServoAngle, double yawServoAngle, double continuity1, double continuity2, double dt, int state, double serialBoolean) {

        TelemetryData data;
        data.roll = eulerAngles[0];
        data.pitch = eulerAngles[1];
        data.yaw = eulerAngles[2];
        data.altitude = altData[0];
        data.pitchServo = pitchServoAngle;
        data.yawServo = yawServoAngle;
        data.continuity1 =  continuity1;
        data.continuity2 =  continuity2;
        data.dt = dt;
        data.state = state;
        data.serialBoolean = serialBoolean; // Add serial boolean to telemetry data

        rf95->send((uint8_t*)&data, sizeof(data)); // Send the telemetry data

        // Update the last send time
        lastTelemetryTime = millis(); // Update the last send time
}
    