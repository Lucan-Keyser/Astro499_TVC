#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <PWMServo.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP3XX.h>
#include <sensors.h>
#include <actuators.h>
#include <BasicLinearAlgebra.h>
#include "../include/config.h"
using namespace BLA;
class Control {
    private:
        SensorSystem& sensors; // Sensor system object
        Actuators& actuators; // Actuator system object

        double* quaternions[4];
        double* omega[3]; // Gyroscope rates [x, y, z]
        double gimbalAngles[2] = {0.0, 0.0}; // Gimbal angles [pitch, yaw]
        double torque[2] = {0.0, 0.0}; // Control torque [pitch, yaw]   
       
        bool controlActive = false; // Control active flag

        Matrix<3, 6, double> K = {0, 0,	0, 0, 0, 0,
                                0, K_PROPORTIONAL_GAIN, 0,	0, K_DERIVATIVE_GAIN, 0,
                                0, 0, K_PROPORTIONAL_GAIN, 0, 0, K_DERIVATIVE_GAIN};

    public:

        Control(SensorSystem& sensors, Actuators& actuators) : sensors(sensors), actuators(actuators) {}
       
        bool initialize();
        void control();
        void updateControlBoolean(bool setpoint);

        double* getGimbalAngles() { return gimbalAngles; }
        bool getControlActive() { return controlActive; } // Getter for control active flag


    };



#endif // CONTROL_H