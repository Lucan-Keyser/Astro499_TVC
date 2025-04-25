#include "../include/state.h"
#include "../include/config.h"
#include "../include/sensors.h"
#include "../include/control.h"
#include "../include/logdata.h" 
#include "../include/hardware.h"
#include <RH_RF95.h>
#include <math.h>
#include <communication.h>




bool State::initialize() {
    // Initialize the state machine
    state = PAD_IDLE; // Set initial state to PAD_IDLE
    return true;
}

void State::executeState() {
    // Execute the state machine
    switch (state) {
        case PAD_IDLE:
            if (communication.checkForCommands() == "CALIBRATE") { //check for commands from LoRa
                sensors.resetSensors();
                hardware.music();
                state++;
            }
            break;
        case CALIBRATE:
            if (communication.checkForCommands() == "LAUNCH") { //check for commands from LoRa
                state++;
            }

            break;
        case COUNTDOWN:
            hardware.setLaunchBool(true); //trigger launch sequence

            if (sensors.checkLaunch()) { //if we are on the pad and we are not moving, we are ready to launch
                startLaunch = millis();
                control.updateControlBoolean(true); //set control mode to launch
                state++;
            }
            break;
        case ASCENT:
            hardware.setLaunchBool(false); //trigger launch sequence
            logdata.logFlightData(state); //log data

            if (checkAbort(sensors.getEulerAngles())) { //if we detect an abort condition, we are in abort state
                state = ABORT;
            } else if (((millis() - startLaunch) >= BURN_TIME)) { //if we should be done burning and we're in freefall
                apogeeTimeLast = millis(); //reset apogee timer
                state++;
                
            }
            break;
        case UNPOWERED_ASCENT:
            sensors.updateAltimeter(); //kills dt, moving to state machine. on unpowered ascent we should be fine
            logdata.logFlightData(state); //log data
            
            if (detectApogee(sensors.getAltitude())) { //if we detect apogee, we are in descent
                control.updateControlBoolean(false); //turn off control mode
                state++;
            } 
            break;
        case DESCENT:
            hardware.setSeparationBool(true); //trigger separation sequence
            sdLogState = 1; //set SD logging state to 1
            communication.sendDataNoDelay(state, sdLogState); //send data to LoRa
            // Use the new high-speed SD writing function instead of dumpToSerial
            if (logdata.dumpToSD()) {
                sdLogState = 2;
                communication.sendDataNoDelay(state, sdLogState); //send data to LoRa
            } else {
                sdLogState = 3;
                communication.sendDataNoDelay(state, sdLogState); //send data to LoRa
            }
            state = GROUND_IDLE; //move to ground idle state
            
            break;
        case ABORT:
            hardware.setSeparationBool(true); //trigger separation sequence
            sdLogState = 1; //set SD logging state to 1
            communication.sendDataNoDelay(state, sdLogState); //send data to LoRa
            // Use the new high-speed SD writing function instead of dumpToSerial
            if (logdata.dumpToSD()) {
                sdLogState = 2;
                communication.sendDataNoDelay(state, sdLogState); //send data to LoRa
            } else {
                sdLogState = 3;
                communication.sendDataNoDelay(state, sdLogState); //send data to LoRa
            }
            state = GROUND_IDLE; //move to ground idle state
            break;
        case GROUND_IDLE:

            break;
    }

    if (communication.checkForCommands() == "RESET") { //check for commands from LoRa
        state = 0; //reset state machine
    }

    communication.sendData(state, sdLogState); //send data to LoRa
    
   
}

bool State::checkAbort(double* eulerAngles) {
    if ((abs(eulerAngles[1])) >= ABORT_CRITERIA || (abs(eulerAngles[2]) >= ABORT_CRITERIA)) { //if we pitch/yaw too far
        return true;
    } else {
        return false;
    }
}

bool State::detectApogee(double altitude) {
    if (detectingApogee) { //if we are checking for apogee, gather data
        if ((millis() - apogeeTimeLast) >= APOGEE_DETECTION_DELAY) {
            apogeeTimeLast = millis(); //reset timer for future iterations
            apogeeMeasurementCount++; //increment measurement count
            altitudeSum = altitudeSum + altitude; //add altitude to sum for averaging
            
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
    return true; //if we get here, something went wrong, return true to avoid infinite loop
    
}
