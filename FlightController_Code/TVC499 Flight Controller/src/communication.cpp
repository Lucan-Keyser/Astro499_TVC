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

bool initializeCommunication(RH_RF95* rf95, unsigned long* lastTelemetryTime) {
    // Initialize LoRa radio
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, LOW); // Reset the radio
    delay(10);
    digitalWrite(RFM95_RST, HIGH); // Release the reset
    // Allow time for the radio to initialize
    delay(10);
    
    // Initialize RF95 module
    if (!rf95->init()) {
        Serial.println("RF95 LoRa init failed!");
        return false;
    }
    
    // Configure radio parameters https://www.rfwireless-world.com/calculators/LoRa-Data-Rate-Calculator.html
    rf95->setFrequency(RF95_FREQ);
    rf95->setTxPower(20, false);  // 20 dBm power level, maximum power for LoRa (2 - 20 dBm)
    rf95->setCodingRate4(5); // Coding rate 4/5 for max data rate (5-8)
    rf95->setSpreadingFactor(6); // SF6 for max data rate (6-12)
    rf95->setSignalBandwidth(500000); // 500 kHz bandwidth for max data rate (125-500 kHz)
    
    // Initialize telemetry timing
    *lastTelemetryTime = millis();
    
    Serial.println("LoRa radio initialized");
    return true;
}

void checkForCommands(RH_RF95* rf95, String* command) {

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
            *command = receivedCommand;
            //Serial.println("Received command: " + receivedCommand);
            }
        }
    
    //end timer
    unsigned long endTime = micros();
    double dt = (endTime - startTime) / 1000000.0; // Convert microseconds to seconds
    printf("dt_LoRa_com_check: %f\n", dt); // Print time delta
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

void sendData(RH_RF95* rf95, double quatAngles[3], double altData[3], PWMServo* yawServo, PWMServo* pitchServo) {
    // Create buffer for message
    char message[60]; 

    // Record start time
    unsigned long startTime = micros();
    
    // Extract orientation data, convert to degrees
    float yaw = quatAngles[0] * RAD_TO_DEG;
    float pitch = quatAngles[1] * RAD_TO_DEG;
    float roll = quatAngles[2] * RAD_TO_DEG;
    
    // Altimeter data
    float altitude = altData[0];
    
    // Read servo positions
    int yawServoAngle = yawServo->read();
    int pitchServoAngle = pitchServo->read();
    
    // Format the data into a string, rounding to second decimal place
    snprintf(message, sizeof(message),
             "YPR: %.2f,%.2f,%.2f; Alt: %.2f; Servo: %d,%d",
             yaw, pitch, roll, altitude, yawServoAngle, pitchServoAngle);
    
    // Send the message via LoRa
    rf95->send((uint8_t *)message, strlen(message));
    rf95->waitPacketSent(); //possibly remove to let program still run without pausing code?

    unsigned long endTime = micros();

    // Calculate and print the time delta
    double dt = (endTime - startTime) / 1000000.0; // Convert microseconds to seconds
    printf("dt_LoRa_send: %f\n", dt); // Print time delta
    //WE WANT TO USE THIS TO THEN FIND MAIN LOOP TIME FOR EXECTUION OF CONTROL ALGORITHIM AND COMPUTATION
    //THEN WE CAN ACCOUNT FOR THE TIME FOR PACKET SEND AND REMOVE WAITPACKETSENT TO LET CODE STILL RUN BUT 
    //GIVE ENOUGH TIME FOR THE PACKET TO SEND
}