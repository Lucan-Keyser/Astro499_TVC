#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>
#include <PWMServo.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP3XX.h>
#include <sensors.h>
#include <BasicLinearAlgebra.h>

class Actuators {
    private:
        double servoAngles[2] = {0.0, 0.0}; // Servo angles [pitch, yaw]
        PWMServo* pitchServo; // Pointer to pitch servo object
        PWMServo* yawServo; // Pointer to yaw servo object

    

    public:
        Actuators(){}
       
        bool initialize();

        void moveServos(double* gimbalAngles);


        double* getServoAngles() { return servoAngles; }

    };



#endif // CONTROL_H