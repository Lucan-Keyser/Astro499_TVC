#ifndef KALMAN_H
#define KALMAN_H

#include <Arduino.h>
#include <FastLED.h>
#include <sensors.h>
#include <BasicLinearAlgebra.h>
#include <ArduinoEigen.h>

// using namespace Eigen;
using namespace BLA;


class Kalman {
    private:
        SensorSystem& sensors;
        Matrix<9, 1, double> states = {0,0,0,0,0,0,0,0,0};
        double gxb, gyb, gzb;
        double gx, gy, gz;
        double axb, ayb, azb;
        double ax, ay, az;
        double q0, q1, q2, q3;
        


    public:
        Kalman(SensorSystem sensors) : sensors(sensors) {}

        void pullSensors();
        void prediction();
        void rotate();
        void update();
        


    };



#endif // KALMAN_H