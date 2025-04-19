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
  float yaw;
  float pitch;
  float roll;
  float altitude;
  float yawServo;
  float pitchServo;
  float continuity1;
  float continuity2;
  float dt;
  float state;
};

bool initializeCommunication(RH_RF95* rf95) {
    // // Initialize LoRa radio
    // pinMode(RFM95_RST, OUTPUT);
    // digitalWrite(RFM95_RST, LOW); // Reset the radio
    // delay(10);
    // digitalWrite(RFM95_RST, HIGH); // Release the reset
    // // Allow time for the radio to initialize
    // delay(10);
    
    // Initialize RF95 module
    if (!rf95->init()) {
        Serial.println("RF95 LoRa init failed!");
        return false;
    }
    
    // Configure radio parameters https://www.rfwireless-world.com/calculators/LoRa-Data-Rate-Calculator.html
    // rf95->setFrequency(RF95_FREQ);
    // rf95->setTxPower(20, false);  // 20 dBm power level, maximum power for LoRa (2 - 20 dBm)
    // rf95->setCodingRate4(5); // Coding rate 4/5 for max data rate (5-8)
    // rf95->setSpreadingFactor(6); // SF6 for max data rate (6-12)
    // rf95->setSignalBandwidth(500000); // 500 kHz bandwidth for max data rate (125-500 kHz)

    rf95->setFrequency(RF95_FREQ);
    rf95->setTxPower(23, false);
    rf95->setSpreadingFactor(9);
    rf95->setSignalBandwidth(500000);
    rf95->setCodingRate4(5);
    
    Serial.println("LoRa radio initialized");
    return true;
}

String checkForCommands(RH_RF95* rf95) {
    String command = ""; // Initialize command string
    //start timer
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
            command = receivedCommand;
            //Serial.println("Received command: " + receivedCommand);
            }
        }
    return command; // Return the received command
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

void sendData(RH_RF95* rf95, double eulerAngles[3], double altData[3], double pitchServoAngle, double yawServoAngle, double continuity1, double continuity2, double dt, int state) {
    // Check if enough time has passed since the last telemetry send
    if (millis() - lastTelemetryTime < TELEMETRY_INTERVAL) {
        return; // Not enough time has passed, skip sending telemetry
    } else {

        // Serial.println("Telemetry data sending !");

        // Create telemetry data structure
        TelemetryData data;
        data.roll = eulerAngles[0];
        data.pitch = eulerAngles[1];

        data.yaw = eulerAngles[2];

        data.altitude = altData[0];

        data.pitchServo = pitchServoAngle;

        data.yawServo = yawServoAngle;

        data.continuity1 =  continuity1; // Check pyro continuity

        data.continuity2 =  continuity2; // Check pyro continuity

        data.dt = dt;
        
        data.state = state;

        // Send the telemetry data
        if (rf95->send((uint8_t*)&data, sizeof(TelemetryData))) {
            // Serial.println("Telemetry data sent!");
        } else {
            Serial.println("Failed to send telemetry data!");
        }

        // Update the last send time
        lastTelemetryTime = millis(); // Update the last send time
    }
    
}
