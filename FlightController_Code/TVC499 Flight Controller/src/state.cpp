#include "../include/state.h"
#include "../include/config.h"
#include "../include/sensors.h"
#include "../include/control.h"
#include "../include/logging.h" 
#include "../include/ringbuffer.h"
#include "../include/hardware.h"
#include <RH_RF95.h>
#include <math.h>
#include <communication.h>


double startLaunch = 0;
double apogeeTimeLast = 0; //last time we checked apogee
double previousAltitude = 0; //last altitude we checked
bool detectingApogee = false; //are we checking for apogee?
int apogeeMeasurementCount = 0; //how many times have we checked for apogee?
double altitudeSum = 0; //sum of altitudes for averaging
double averageAltitude = 0; //average altitude for apogee detection


void stateMachine(Adafruit_BNO08x* bno, Adafruit_BMP3XX* bmp, RH_RF95* rf95, int& state, double* accelerometer, double* eulerAngles, double* altData, double* quaternions, double& refPressure, bool& launchTriggered, bool& separationTriggered) {
    switch (state) {
        case PAD_IDLE:
            break;
        case CALIBRATE:
            break;
        case COUNTDOWN:
            triggerLaunch(launchTriggered); //trigger launch sequence
            break;
        case ASCENT:
            logFlightData(); //log data
            break;
        case UNPOWERED_ASCENT:
            updateAltimeter(bmp, altData, refPressure); //kills dt, moving to state machine. on unpowered ascent we should be fine
            logFlightData(); //log data
            break;
        case DESCENT:
            triggerSeparation(separationTriggered); //trigger separation sequence
            updateSerialLog(rf95, 1);
            dumpToSerial(); //dump flight log to serial
            updateSerialLog(rf95, 2);
            break;
        case ABORT:
            triggerSeparation(separationTriggered); //trigger separation sequence
            updateSerialLog(rf95, 1);
            dumpToSerial(); //dump flight log to serial
            updateSerialLog(rf95, 2);
            break;
        case GROUND_IDLE:
            break;
    }

    updateState(bno, bmp, rf95, state, accelerometer, eulerAngles, altData, quaternions, refPressure);
    // Serial.println(state);
}

void updateState(Adafruit_BNO08x* bno, Adafruit_BMP3XX* bmp, RH_RF95* rf95, int& state, double* accelerometer, double* eulerAngles, double* altData, double* quaternions, double& refPressure) {
    switch (state) {
        case PAD_IDLE: //monitor launch criteria, have to include criteria
            if (checkForCommands(rf95) == "CALIBRATE") { //check for commands from LoRa
                resetSensors(bno, bmp, quaternions, accelerometer, refPressure);
                Serial5.println("Begin Calibration!");
                countDownMusic(); //play countdown music
                state++;
            }
            break;
        case CALIBRATE:
            if (checkForCommands(rf95) == "LAUNCH") { //check for commands from LoRa
                Serial5.println("Begin Launch!");
                state++;
            }
            break;
        case COUNTDOWN:
            if (accelerometer[0] <= 15) { //if we are on the pad and we are not moving, we are ready to launch
                startLaunch = millis();
                state++;
            }
            break;
        case ASCENT: {
            double accelVector = sqrt(accelerometer[0] * accelerometer[0] + accelerometer[1] * accelerometer[1] + accelerometer[2] * accelerometer[2]);
            if (checkAbort(eulerAngles)) {
                state = ABORT;
            } else if (((millis() - startLaunch) >= BURN_TIME) && (accelVector <= FREE_FALL_ACCEL)) { //if we should be done burning and we're in freefall
                state++;
                apogeeTimeLast = millis(); //reset apogee timer
            }
            break;}
        case UNPOWERED_ASCENT: {
            if (detectApogee(altData)) { //if we detect apogee, we are in descent
                state++;
            } 
            break; }
        case DESCENT:
            state = GROUND_IDLE; //go to ground idle state
            break;
        case ABORT:
            state = GROUND_IDLE; //go to ground idle state
            break;
        case GROUND_IDLE:
            break;
    }
    if (checkForCommands(rf95) == "RESET") { //check for commands from LoRa
        state = 0; //reset state machine
    }
}

bool checkAbort(double* eulerAngles) {
    if ((abs(eulerAngles[1])) >= ABORT_CRITERIA || (abs(eulerAngles[2]) >= ABORT_CRITERIA)) { //if we pitch/yaw too far
        return true;
    } else {
        return false;
    }
}

bool detectApogee(double* altData) {
    if (detectingApogee) { //if we are checking for apogee, gather data
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
            detectingApogee = true; //gather apogee data
        }
        return false;
    }
    
}
