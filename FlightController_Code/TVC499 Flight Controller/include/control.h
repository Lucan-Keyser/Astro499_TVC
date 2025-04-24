#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <PWMServo.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP3XX.h>
#include <sensors.h>
#include <actuators.h>
#include <BasicLinearAlgebra.h>
class Control {
    private:
        SensorSystem& sensors; // Sensor system object
        Actuators& actuators; // Actuator system object

        double* quaternions[4];
        double* omega[3]; // Gyroscope rates [x, y, z]
        double gimbalAngles[2] = {0.0, 0.0}; // Gimbal angles [pitch, yaw]
        double torque[2] = {0.0, 0.0}; // Control torque [pitch, yaw]   
       
        bool controlActive = false; // Control active flag

        Matrix<3, 6, double> K = {10.0000000000000,	0,	0,	1.01242283656583,	0,	0,
                         0,	10.0000000000000,	0,	0,	1.04880884817015,	0,
                        0,	0,	10.0000000000000,	0,	0,	1.04880884817015};

    public:

        Control(SensorSystem& sensors, Actuators& actuators) : sensors(sensors), actuators(actuators) {}
       
        bool initialize();
        void control();
        void updateControlBoolean(bool setpoint);

        double* getGimbalAngles() { return gimbalAngles; }
        bool getControlActive() { return controlActive; } // Getter for control active flag


    };



#endif // CONTROL_H