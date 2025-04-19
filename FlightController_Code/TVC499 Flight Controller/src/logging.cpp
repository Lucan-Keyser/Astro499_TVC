#include <Arduino.h>
#include "../include/logging.h"
#include "../include/config.h"
#include "../include/communication.h"
#include "../include/hardware.h"
#include <RH_RF95.h> // Include the header file for RH_RF95
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
  printToCSV();
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



