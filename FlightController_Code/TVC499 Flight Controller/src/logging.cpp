#include <Arduino.h>
#include "../include/logging.h"
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

void logGlobalData (double* gyroRates, double* quaternions, double* eulerAngles, double* accelerometer, double refPressure, double* altData, double dt) {
    for(int i = 0; i < 3; i++) gyro[i] = gyroRates[i];
    for(int i = 0; i < 4; i++) quat[i] = quaternions[i];
    for(int i = 0; i < 3; i++) euler[i] = eulerAngles[i];
    for(int i = 0; i < 3; i++) accel[i] = accelerometer[i];
    for(int i = 0; i < 3; i++) alt[i] = altData[i];
    refP = refPressure;
    dT = dt;
    
}

void logControlData (double* gim, double* serv) {
    for(int i = 0; i < 2; i++) gimbal[i] = gim[i];
    for(int i = 0; i < 2; i++) servo[i] = serv[i];
}

void sendToLog () { //log all data that's been updated
  printToCSV();
}

void printToCSV() {
    // Print headers (optional, good for first row)
    // Serial.println("GyroX,GyroY,GyroZ,QuatW,QuatX,QuatY,QuatZ,Yaw,Pitch,Roll,AccelX,AccelY,AccelZ,RefPressure,Altitude,Pressure,Temperature,DeltaTime");
  
    // Print gyro data (rad/s)
    for(int i = 0; i < 3; i++) {
      Serial.print(gyro[i]);
      Serial.print(",");
    }
  
    // Print quaternion data
    for(int i = 0; i < 4; i++) {
      Serial.print(quat[i]);
      Serial.print(",");
    }
  
    // Print Euler angles (degrees)
    for(int i = 0; i < 3; i++) {
      Serial.print(euler[i]);
      Serial.print(",");
    }
  
    // Print accelerometer data
    for(int i = 0; i < 3; i++) {
      Serial.print(accel[i]);
      Serial.print(",");
    }
  
    // Print reference pressure
    Serial.print(refP);
    Serial.print(",");
  
    // Print altimeter data
    for(int i = 0; i < 3; i++) {
      Serial.print(alt[i]);
      Serial.print(",");
    }

    for(int i = 0; i < 2; i++) {
      Serial.print(gimbal[i]);
      Serial.print(",");
    }

    for(int i = 0; i < 2; i++) {
      Serial.print(servo[i]);
      Serial.print(",");
    }
  
    // Print delta time
    Serial.print(dT);
  
    // End the line
    Serial.println();
  }



