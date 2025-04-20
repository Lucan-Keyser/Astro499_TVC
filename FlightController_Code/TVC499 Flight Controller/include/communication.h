/** communication.h
* ===========================================================
* Name: Flight Controller Communication Interface
* Section: TVC499
* Project: Flight Controller
* Purpose: LoRa radio communication and serial interface
* ===========================================================
*/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <RH_RF95.h>
#include <PWMServo.h>

// RF95 LoRa radio settings
#define RFM95_CS 10
#define RFM95_RST 23
#define RFM95_INT 1
#define RF95_FREQ 915.0
#define TELEMETRY_INTERVAL 50  // Send telemetry every 50ms

extern double lastTelemetryTime;


/**
 * @brief Initialize LoRa communication and Serial interface
 * @param rf95 Pointer to RF95 LoRa radio object
 * @return True if initialization successful, false otherwise
 */
bool initializeCommunication(RH_RF95* rf95);

/**
 * @brief Check for incoming LoRa commands
 * @param rf95 Pointer to RF95 LoRa radio object
 * @param command Pointer to command string
 */
String checkForCommands(RH_RF95* rf95);

/**
 * @brief Process serial commands looking for "SEPARATE" or "LAUNCH"
 * @param command Pointer to command string
 * @param separationTriggered Pointer to separation flag
 * @param launchTriggered Pointer to launch flag
 */
void readSerial(String* command, bool& separationTriggered, bool& launchTriggered);

/**
 * @brief Send telemetry data over LoRa (non-blocking)
 * @param rf95 Pointer to RF95 LoRa radio object
 * @param quatAngles Array of quaternion angles [yaw,pitch,roll]
 * @param altData Array of altitude data [altitude,pressure,temp]
 * @param pitchServoAngle Pitch servo angle
 * @param yawServoAngle Yaw servo angle
 * @return True if packet was queued successfully, false otherwise
 */
<<<<<<< HEAD
void sendData(RH_RF95* rf95, double quatAngles[3], double altData[3], double pitchServoAngle, double yawServoAngle, double continuity1, double continuity2, double dt, int state, double serailBoolean);

void sendDataNoDelay(RH_RF95* rf95, double quatAngles[3], double altData[3], double pitchServoAngle, double yawServoAngle, double continuity1, double continuity2, double dt, int state, double serailBoolean);
=======
bool sendData(RH_RF95* rf95, double quatAngles[3], double altData[3], double pitchServoAngle, double yawServoAngle);
>>>>>>> main

#endif // COMMUNICATION_H