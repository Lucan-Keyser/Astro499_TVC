#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <PWMServo.h>

void control(double* quaternions, double* omega, PWMServo pitchServo, PWMServo yawServo);

void moveServos (double* input, PWMServo pitchServo, PWMServo yawServo);



#endif // CONTROL_H