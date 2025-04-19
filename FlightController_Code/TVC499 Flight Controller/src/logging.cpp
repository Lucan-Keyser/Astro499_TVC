#include <Arduino.h>
#include "../include/logging.h"
#include "../include/config.h"
#include "../include/communication.h"
#include "../include/hardware.h"
#include <RH_RF95.h> // Include the header file for RH_RF95
#include <DMAChannel.h>

// Define DMA channel
DMAChannel serialTxDMA;
uint8_t serialBuffer[256];  // Buffer to hold data
volatile bool dmaActive = false;

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
static unsigned long lastMessageTime = 0;
// Add these variables at the top of logging.cpp
static unsigned long debugTimerDMA = 0;
static unsigned long callCounter = 0;
static unsigned long successCounter = 0;
static unsigned long failCounter = 0;
static bool firstTransferAttempt = true;

void setupSerialDMA() {
  // Configure DMA for Serial5 TX
  serialTxDMA.destination(IMXRT_LPUART8.DATA);  // Serial5 is LPUART8 on Teensy 4.1
  serialTxDMA.triggerAtHardwareEvent(DMAMUX_SOURCE_LPUART8_TX);
  serialTxDMA.disableOnCompletion();
  serialTxDMA.attachInterrupt(dmaCompleteCallback);
  
  // Debug info
  Serial.println("DMA configured for Serial5 (LPUART8)");
  Serial.println("=====================================");
  Serial.println("Initial debugging message");
  Serial5.println("FLIGHT CONTROLLER LOGGING STARTED");
  Serial5.println("=================================");
  delay(100); // Give serial time to send
}

void dmaCompleteCallback() {
  successCounter++;
  dmaActive = false;
  
  // Debug via regular serial to the computer
  Serial.print("DMA transfer #");
  Serial.print(callCounter);
  Serial.println(" completed");
}

bool isDMAActive() {
  return dmaActive;
}

void sendSerialDMA(const uint8_t* data, size_t size) {
  callCounter++;
  
  // Debug stats every 10 calls
  if (callCounter % 10 == 0) {
    Serial.println("DMA STATS:");
    Serial.print("Calls: ");
    Serial.print(callCounter);
    Serial.print(", Successes: ");
    Serial.print(successCounter);
    Serial.print(", Failures: ");
    Serial.print(failCounter);
    Serial.print(", Current state: ");
    Serial.println(dmaActive ? "ACTIVE" : "INACTIVE");
  }
  
  if (dmaActive) {
    failCounter++;
    Serial.print("DMA #");
    Serial.print(callCounter);
    Serial.println(" SKIPPED - Previous transfer still active");
    return;
  }
  
  if (firstTransferAttempt) {
    Serial.println("First DMA transfer attempt!");
    Serial5.println("First line via direct Serial5");
    delay(10);
    firstTransferAttempt = false;
  }
  
  // Print debug info about what we're sending
  Serial.print("Starting DMA transfer #");
  Serial.print(callCounter);
  Serial.print(" of ");
  Serial.print(size);
  Serial.println(" bytes");
  
  // For the first few transfers, print the first 20 bytes to debug
  if (callCounter < 5) {
    Serial.print("First 20 bytes: ");
    for (int i = 0; i < min(20, (int)size); i++) {
      Serial.print((char)data[i]);
    }
    Serial.println();
  }
  
  // Copy data to buffer
  if (size > sizeof(serialBuffer)) {
    Serial.print("WARNING: Data truncated from ");
    Serial.print(size);
    Serial.print(" to ");
    Serial.print(sizeof(serialBuffer));
    Serial.println(" bytes");
    size = sizeof(serialBuffer);
  }
  memcpy(serialBuffer, data, size);
  
  // Set up and start DMA transfer
  serialTxDMA.sourceBuffer(serialBuffer, size);
  dmaActive = true;
  
  // Record what we're trying to send for debugging
  if (callCounter < 5) {
    Serial.print("Sending data via DMA #");
    Serial.println(callCounter);
  }
  
  serialTxDMA.enable();
  
  // Register completion time for calculation
  debugTimerDMA = millis();
}




void printToCSV_DMA() {
  if (millis() - lastMessageTime > 1000) { // every second
    Serial5.println("DMA active status: " + String(dmaActive ? "Active" : "Inactive"));
    lastMessageTime = millis();
  }
  // Create a buffer to hold the formatted data
  char buffer[256];
  int pos = 0;
  
  // Format data into the buffer
  for(int i = 0; i < 3; i++) {
    pos += sprintf(buffer + pos, "%.3f,", gyro[i]);
  }

  // for(int i = 0; i < 4; i++) {
  //   pos += sprintf(buffer + pos, "%.6f,", quat[i]);
  // }

  // // Print Euler angles (degrees)
  // for(int i = 0; i < 3; i++) {
  //   pos += sprintf(buffer + pos, "%.6f,", euler[i]);
  // }

  // // Print accelerometer data
  // for(int i = 0; i < 3; i++) {
  //   pos += sprintf(buffer + pos, "%.6f,", accel[i]);
  // }

  // // Print reference pressure
  // pos += sprintf(buffer + pos, "%.6f,", refP);

  // // Print altimeter data
  // for(int i = 0; i < 3; i++) {
  //   pos += sprintf(buffer + pos, "%.6f,", alt[i]);
  // }

  // for(int i = 0; i < 2; i++) {
  //   pos += sprintf(buffer + pos, "%.6f,", gimbal[i]);
  // }

  // for(int i = 0; i < 2; i++) {
  //   pos += sprintf(buffer + pos, "%.6f,", servo[i]);
  // }

  // for(int i = 0; i < 2; i++) {
  //   pos += sprintf(buffer + pos, "%.6f,", cont[i]);
  // }

  // //print state
  // pos += sprintf(buffer + pos, "%.6f,", state);
 

  // // Print delta time
  // pos += sprintf(buffer + pos, "%.6f,", dT);

  pos += sprintf(buffer + pos, "\r\n"); // Add a newline at the end
  // End the line
  sendSerialDMA((uint8_t*)buffer, pos);
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
  printToCSV_DMA(); // Print to CSV using DMA
  sendData(rf95, euler, alt, servo[0], servo[1], cont[0], cont[1], dT, state); //send data to LoRa
  // Serial.println("Data sent to LoRa!");
}

void printToCSV() {
    // Print headers (optional, good for first row)
    // Serial.println("GyroX,GyroY,GyroZ,QuatW,QuatX,QuatY,QuatZ,Yaw,Pitch,Roll,AccelX,AccelY,AccelZ,RefPressure,Altitude,Pressure,Temperature,DeltaTime");
  
    // Print gyro data (rad/s)
    for(int i = 0; i < 3; i++) {
      Serial5.print(gyro[i]);
      Serial5.print(",");
    }
  
    // Print quaternion data
    for(int i = 0; i < 4; i++) {
      Serial5.print(quat[i]);
      Serial5.print(",");
    }
  
    // Print Euler angles (degrees)
    for(int i = 0; i < 3; i++) {
      Serial5.print(euler[i]);
      Serial5.print(",");
    }
  
    // Print accelerometer data
    for(int i = 0; i < 3; i++) {
      Serial5.print(accel[i]);
      Serial5.print(",");
    }
  
    // Print reference pressure
    Serial5.print(refP);
    Serial5.print(",");
  
    // Print altimeter data
    for(int i = 0; i < 3; i++) {
      Serial5.print(alt[i]);
      Serial5.print(",");
    }

    for(int i = 0; i < 2; i++) {
      Serial5.print(gimbal[i]);
      Serial5.print(",");
    }

    for(int i = 0; i < 2; i++) {
      Serial5.print(servo[i]);
      Serial5.print(",");
    }

    for(int i = 0; i < 2; i++) {
      Serial5.print(cont[i]);
      Serial5.print(",");
    }

    //print state
    Serial5.print(state);
    Serial5.print(",");
   

    // Print delta time
    Serial5.print(dT);

    

    // End the line
    Serial5.println();
  }



