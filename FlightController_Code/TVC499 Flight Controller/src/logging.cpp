#include "../include/logging.h"
#include "../include/config.h"
#include "../include/communication.h"
#include "../include/hardware.h"

// Create the flight data buffer
FlightDataBuffer flightLog;

//have to change variable names to make them local to file
double gyro[3] = {0.0, 0.0, 0.0}; //in radians/sec
double quat[4] = {1, 0, 0, 0}; //Quaterinon vector
double euler[3] = {0.0, 0.0, 0.0}; // Yaw, Pitch, Roll in degrees
double accel[3] = {0.0, 0.0, 0.0}; //accelerometer values, x,y,z
double refP = 1000; // Reference pressure in hPa
double alt[3] = {0.0, 0.0, 0.0}; // Altitude data [altitude (m), pressure (PA), temperature]
double dT = 0; 
double gimbal[2] = {0.0, 0.0}; //pitch and yaw torque
double servo[2] =  {0.0, 0.0};
double state = 0;
double cont[2] = {0.0,0.0}; // Initialize continuity array
double serialBoolean = 0;


// Last send time for telemetry

void initializeLogging() {
    // Initialize the flight data buffer
    flightLog.resetBuffer();
    
    // Test Serial5 connection
    Serial5.println("FLIGHT CONTROLLER INITIALIZED");
    Serial5.println("LOGGING SYSTEM READY");
    
    Serial.println("Flight logging system initialized");
}

void logFlightData() {
    
    // Create flight data entry
    FlightDataEntry entry;
    
    // Fill in data
    entry.timestamp = millis();
    
    // Copy all flight data
    memcpy(entry.gyro, gyro, sizeof(entry.gyro));
    memcpy(entry.quaternions, quat, sizeof(entry.quaternions));
    memcpy(entry.eulerAngles, euler, sizeof(entry.eulerAngles));
    memcpy(entry.accelerometer, accel, sizeof(entry.accelerometer));
    memcpy(entry.servoPositions, servo, sizeof(entry.servoPositions));
    memcpy(entry.gimbalPositions, gimbal, sizeof(entry.gimbalPositions));
    memcpy(entry.continuity, cont, sizeof(entry.continuity));
    
    // Individual values
    entry.altitude = alt[0];
    entry.pressure = alt[1];
    entry.temperature = alt[2];
    entry.flightState = state;
    entry.dt = dT;
    
    // Store in the buffer - unused variable warning will be suppressed
    /*bool success =*/ flightLog.storeData(entry);
    
    // Debug info (only log occasionally to avoid slowing down)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 5000) {
        Serial.print("Flight log entries: ");
        Serial.print(flightLog.getEntryCount());
        Serial.print(", Buffer full: ");
        Serial.println(flightLog.isFull() ? "YES" : "NO");
        lastDebugTime = millis();
    }
}

void logGlobalData (double* gyroRates, double* quaternions, double* eulerAngles, double* accelerometer, double refPressure, double* altData, double st, double dt, double* continuity) {
  for(int i = 0; i < 3; i++) gyro[i] = gyroRates[i];
  for(int i = 0; i < 4; i++) quat[i] = quaternions[i];
  for(int i = 0; i < 3; i++) euler[i] = eulerAngles[i];
  for(int i = 0; i < 3; i++) accel[i] = accelerometer[i];
  for(int i = 0; i < 3; i++) alt[i] = altData[i];
  for(int i = 0; i < 2; i++) cont[i] = continuity[i];
  refP = refPressure;
  state = st;
  dT = dt;

  
}

void logControlData (double* gim, double* serv) {
  for(int i = 0; i < 2; i++) gimbal[i] = gim[i];
  for(int i = 0; i < 2; i++) servo[i] = serv[i];
}

void sendToLog (RH_RF95* rf95) { //log all data that's been updated
  sendData(rf95, euler, alt, servo[0], servo[1], cont[0], cont[1], dT, state, serialBoolean); //send data to LoRa
  // Serial.println("Data sent to LoRa!");
}

void updateSerialLog (RH_RF95* rf95, double setpoint) {
  serialBoolean = setpoint; //set boolean to true to indicate that data is being sent to serial
  sendDataNoDelay(rf95, euler, alt, servo[0], servo[1], cont[0], cont[1], dT, state, serialBoolean); //send data to LoRa
}

// void checkForLogCommands() {
//     if (Serial.available()) {
//         String command = Serial.readString();
//         command.trim();
        
//         if (command == "DUMP") {
//             flightLog.dumpToSerial();
//         } else if (command == "RESET") {
//             flightLog.resetBuffer();
//             Serial.println("Flight log buffer reset");
//         }
//     }
// }