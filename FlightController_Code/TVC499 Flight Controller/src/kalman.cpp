#include "../include/kalman.h"



void Kalman::pullSensors() {
    gxb = sensors.getGyroRates()[0];
    gyb = sensors.getGyroRates()[1];
    gzb = sensors.getGyroRates()[2];

    axb = sensors.getAccelerometer()[0];
    ayb = sensors.getAccelerometer()[1];
    azb = sensors.getAccelerometer()[2];

    q0 = sensors.getQuaternions()[0];
    q1 = sensors.getQuaternions()[1];
    q2 = sensors.getQuaternions()[2];
    q3 = sensors.getQuaternions()[3];

    rotate();
}

void Kalman::prediction() {

}

void Kalman::update() {
    
}

void Kalman::rotate() {
    Matrix<3,3, double> DCM = {1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2),
                                2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1),
                                2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)};
    
    Matrix<3,1, double> accelBody = {axb, ayb, azb};

    Matrix<3,1, double> accelEarth;

    accelEarth = DCM * accelBody;

    ax = accelEarth(0) - 9.81;
    ay = accelEarth(1);
    az = accelEarth(2);
}