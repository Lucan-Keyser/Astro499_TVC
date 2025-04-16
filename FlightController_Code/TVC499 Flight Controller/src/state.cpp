#include "../include/state.h"
#include "../include/config.h"
#include <math.h>

double startLaunch = 0;
double apogeeTimeLast = 0; //last time we checked apogee
double previousAltitude = 0; //last altitude we checked
bool detectingApogee = false; //are we checking for apogee?
int apogeeMeasurementCount = 0; //how many times have we checked for apogee?
double altitudeSum = 0; //sum of altitudes for averaging
double averageAltitude = 0; //average altitude for apogee detection

void stateMachine(int& state, double* accelerometer, double* eulerAngles, double* altData) {
    switch (state) {
        case PAD_IDLE:
            break;
        case COUNTDOWN:
            break;
        case ASCENT:
            break;
        case UNPOWERED_ASCENT:
            break;
        case DESCENT:
            break;
        case ABORT:
            break;
    }

    updateState(state, accelerometer, eulerAngles, altData);
    // Serial.println(state);
}

void updateState(int& state, double* accelerometer, double* eulerAngles, double* altData) {
    switch (state) {
        case PAD_IDLE: //monitor launch criteria, have to include criteria
            if (Serial.available()) {
                // Read command from Serial
                String command = Serial.readString();
                command.trim();
                Serial.print("Received command: ");
                Serial.println(command);
                if (command == "g") {
                    state++;
                }
            }
            break;
        case COUNTDOWN:
            if (accelerometer[0] <= -15) { //x acceleration
                startLaunch = millis();
                state++;
            }
            break;
        case ASCENT: {
            double accelVector = sqrt(accelerometer[0] * accelerometer[0] + accelerometer[1] * accelerometer[1] + accelerometer[2] * accelerometer[2]);
            if (checkAbort(eulerAngles)) {
                state = -1;
            } else if (((millis() - startLaunch) >= BURN_TIME) && (accelVector <= FREE_FALL_ACCEL)) { //if we should be done burning and we're in freefall
                state++;
                apogeeTimeLast = millis(); //reset apogee timer
            }
            break;}
        case UNPOWERED_ASCENT: {
            if (detectApogee(altData)) { //if we detect apogee, we are in unpowered ascent
                state++;
            } 
            break; }
        case DESCENT:
            break;
        case ABORT:
            break;
    }
}

bool checkAbort(double* eulerAngles) {
    if ((abs(eulerAngles[1])) >= ABORT_CRITERIA || (abs(eulerAngles[2]) >= ABORT_CRITERIA)) { //if we pitch too far
        return true;
    } else {
        return false;
    }
}

bool detectApogee(double* altData) {
    if (detectingApogee) { //if we are checking for apogee, check for it
        if ((millis() - apogeeTimeLast) >= APOGEE_DETECTION_DELAY) {
            apogeeTimeLast = millis(); //reset timer for future iterations
            apogeeMeasurementCount++; //increment measurement count
            altitudeSum = altitudeSum + altData[0]; //add altitude to sum for averaging
            
            if (apogeeMeasurementCount >= APOGEE_DETECTION_COUNT) { //if we have enough measurements, check for apogee
                detectingApogee = false; //stop checking for apogee
                apogeeMeasurementCount = 0; //reset measurement count
                averageAltitude = altitudeSum / APOGEE_DETECTION_COUNT; //calculate average altitude
                altitudeSum = 0; //reset altitude sum for future iterations

                
                if (previousAltitude > averageAltitude + ALTITUDE_THRESHOLD) { //have we descended below previous altitudes?
                    return true; //apogee!
                } else {
                    previousAltitude = averageAltitude; //reset altitude to check in the future. 
                    return false; //no apogee!
                }
            } else {
                return false; //not enough measurements yet
            }

        }
    } else { //if we are below ground level, we can't be at apogee
        if ((millis() - apogeeTimeLast) > APOGEE_DETECTION_INTERVAL) { //if its time to check apogee, lets check it!
            detectingApogee = true; //reset timer for future iterations
        }
        return false;
    }
    
}

// boolean detectApogee() { //have we apogeed?
//     if ((millis()-apogeeLast)>apogeeDt) {//if its time to check apogee, lets check it!
//       apogeeLast = millis(); //reset timer for future iterations
  
//       if (previousAltitude>altData[0]+deltaH) { //have we descended below previous altitudes?
//         return true; //apogee!
//       }
  
//       previousAltitude = altData[0]; //reset altitude to check in the future. 
//       return false; //no apogee!
  
//     } else { 
//       return false; //no apogee!
//     }
//   }
  