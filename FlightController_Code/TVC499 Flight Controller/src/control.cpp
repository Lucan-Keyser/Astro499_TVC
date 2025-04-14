#include "../include/control.h"
#include <BasicLinearAlgebra.h>
#include <PWMServo.h>
using namespace BLA;


Matrix<3, 6> K = {10.0000000000000,	0,	0,	1.01242283656583,	0,	0,
                    0,	10.0000000000000,	0,	0,	1.04880884817015,	0,
                    0,	0,	10.0000000000000,	0,	0,	1.04880884817015,};
// K is the gain matrix for the LQR controller

double const SERVO_RATIO = 3.3; //Ratio of gimbal to servo angle
double const GIMBAL_MAX = DEG_TO_RAD * 8; //Max servo angle in rad
double const PITCH_SERVO_OFFFSET = 0.0; //Pitch servo offset in degrees
double const YAW_SERVO_OFFFSET = 0.0; //Yaw servo offset in degrees
double const MOMENT_OF_INERTIA = 0.1; //Moment of inertia in kg*m^2
double const MOMENT_ARM = 0.49; //Moment arm in meters
double const THRUST = 15;


void control(double* quaternions, double* omega, PWMServo pitchServo, PWMServo yawServo) {
    //state vector
    Matrix<6, 1> x = {quaternions[1], quaternions[2], quaternions[3], omega[0], omega[1], omega[2]}; 
    Matrix<3, 1> u = -K * x; //u is the control torque vector
    //u(0) is roll torque, u(1) is pitch torque, u(2) is yaw torque
    double input[2] = {u(1), u(2)}; //input is the control torque vector that we can actually affect
    moveServos(input, pitchServo, yawServo); //move servos to the calculated angles
}

void moveServos (double* torque, PWMServo pitchServo, PWMServo yawServo) {
    // Calculate the gimbal angles in radians based on the control torques
    double gimbal[2] = {torque[0] * MOMENT_OF_INERTIA / (MOMENT_ARM * THRUST), torque[1] * MOMENT_OF_INERTIA / (MOMENT_ARM * THRUST)};
    // Map the control torques to servo angles
    // Constrain the angles to gimbal servo limits
    gimbal[0] = constrain(gimbal[0], -GIMBAL_MAX, GIMBAL_MAX); //Constrain the gimbal angles to the servo limits
    gimbal[1] = constrain(gimbal[1], -GIMBAL_MAX, GIMBAL_MAX); //Constrain the gimbal angles to the servo limits
    double pitchAngle = SERVO_RATIO * RAD_TO_DEG * gimbal[0] - PITCH_SERVO_OFFFSET; //Pitch servo angle in degrees
    double yawAngle = SERVO_RATIO * RAD_TO_DEG * gimbal[1] - YAW_SERVO_OFFFSET; //Yaw servo angle in degrees
    
    
    Serial.print("Pitch Servo Angle: ");
    Serial.print(pitchAngle);
    Serial.print("Yaw Servo Angle: ");
    Serial.println(yawAngle);
    // Write the angles to the servos
    pitchServo.write(pitchAngle);
    yawServo.write(yawAngle);
   
}

