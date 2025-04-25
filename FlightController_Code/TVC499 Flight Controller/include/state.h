#ifndef STATE_H
#define STATE_H

#include "../include/state.h"
#include "../include/config.h"
#include "../include/sensors.h"
#include "../include/control.h"
#include "../include/logdata.h" 
#include "../include/hardware.h"
#include "../include/communication.h"
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP3XX.h>
#include <utility/imumaths.h>
#include <RH_RF95.h>

class State {
    private:
        SensorSystem& sensors; // Sensor system object
        Actuators& actuators; // Actuator system object
        Hardware& hardware; // Hardware system object
        Control& control; // Control system object
        LogData& logdata; // Logging system object
        Communication& communication; // Communication system object

        double startLaunch = 0; // Start time of launch sequence
        double apogeeTimeLast = 0; //last time we checked apogee
        double previousAltitude = 0; //last altitude we checked
        bool detectingApogee = false; //are we checking for apogee?
        int apogeeMeasurementCount = 0; //how many times have we checked for apogee?
        double altitudeSum = 0; //sum of altitudes for averaging
        double averageAltitude = 0; //average altitude for apogee detection
        int sdLogState = 0; // State of the SD card logging


        int state = 0; // Current state of the flight computer

    public:

        State(SensorSystem& sensors, Control& control, Actuators& actuators, Hardware& hardware, Communication& communication, LogData& logdata) : sensors(sensors),
        control(control), actuators(actuators), hardware(hardware), communication(communication), logdata(logdata) {}
       
        bool initialize();
        void executeState();
        bool checkAbort(double* eulerAngles); // Check for abort condition
        bool detectApogee(double altitude); // Check for apogee detection
};




void stateMachine(Adafruit_BNO08x* bno, Adafruit_BMP3XX* bmp, RH_RF95* rf95, int& state, double* accelerometer, double* eulerAngles, double* altData, double* quaternions, double& refPressure, bool& launchTriggered, bool& separationTriggered);

void updateState(Adafruit_BNO08x* bno, Adafruit_BMP3XX* bmp, RH_RF95* rf95, int& state, double* accelerometer, double* eulerAngles, double* altData, double* quaternions, double& refPressure);

bool checkAbort(double* eulerAngles);

bool detectApogee(double* altData);

#endif // SENSORS_H