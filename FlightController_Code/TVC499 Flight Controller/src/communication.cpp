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

#pragma pack(push, 1)
struct TelemetryData {
  double yaw;
  double pitch;
  double roll;
  double altitude;
  double yawServo;
  double pitchServo;
};

bool initializeCommunication(RH_RF95* rf95) {
    // Initialize RF95 module
    if (!rf95->init()) {
        Serial.println("RF95 LoRa init failed!");
        return false;
    }
    
    rf95->setFrequency(RF95_FREQ);
    rf95->setTxPower(23, false);
    rf95->setSpreadingFactor(9);
    rf95->setSignalBandwidth(500000);
    rf95->setCodingRate4(5);
    
    Serial.println("LoRa radio initialized");
    return true;
}

void checkForCommands(RH_RF95* rf95, String* command) {
    unsigned long startTime = micros();

    // Check for incoming LoRa commands
    if (rf95->available()) {
        // Buffer for received message
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        
        // Read the message
        if (rf95->recv(buf, &len)) { //recv returns true if a message is received, buf is the buffer, len is the length of the message
            buf[len] = '\0';  // Null-terminate buffer so we can treat it as a string
            String receivedCommand = String((char*)buf); //receivedCommand is a string
            receivedCommand.trim(); // Remove any leading/trailing whitespace
            
            Serial.print("LoRa received: "); 
            Serial.println(receivedCommand);
            
            // Store other commands for processing
            *command = receivedCommand;
        }
    }
    
    unsigned long endTime = micros();
    double dt = (endTime - startTime) / 1000000.0; // Convert microseconds to seconds
}

void readSerial(String* command, bool* separationTriggered, bool* launchTriggered) { 
    // Check for incoming serial commands
    if (Serial.available()) {
        // Read command from Serial
        *command = Serial.readString();
        command->trim();
        Serial.print("Received command: ");
        Serial.println(*command);
        
        // Process specific commands
        if (command->equals("SEPARATE")) {
            // Trigger separation or launch sequence
            Serial.println("Triggering PYROS!");
            triggerSeparation(separationTriggered);
        }
        if (command ->equals("LAUNCH")) {
            // Trigger launch sequence
            Serial.println("Triggering Launch!");
            triggerLaunch(separationTriggered);
        }
    }
}

bool sendData(RH_RF95* rf95, double eulerAngles[3], double altData[3], double pitchServoAngle, double yawServoAngle) {
    // Don't use waitPacketSent - make it non-blocking
    
    // Check if RF95 is busy - don't try to send if it's still transmitting
    if (rf95->mode() == RHGenericDriver::RHModeTx) {
        // Radio is busy sending, don't attempt a new transmission
        return false;
    }
    
    // Create telemetry data structure
    TelemetryData data;
    data.roll = eulerAngles[0];
    data.pitch = eulerAngles[1];
    data.yaw = eulerAngles[2];
    data.altitude = altData[0];
    data.yawServo = yawServoAngle;
    data.pitchServo = pitchServoAngle;

    // Debug print (can be commented out in production for performance)
    // Serial.print("Sending telemetry: Roll=");
    // Serial.print(data.roll);
    // Serial.print(", Pitch=");
    // Serial.print(data.pitch);
    // Serial.print(", Alt=");
    // Serial.println(data.altitude);
    
    // Send the telemetry data without waiting
    if (rf95->send((uint8_t*)&data, sizeof(TelemetryData))) {
        // Packet queued for sending, but not waiting
        return true;
    } else {
        Serial.println("Failed to queue telemetry data");
        return false;
    }
}