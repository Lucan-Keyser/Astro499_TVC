#include "../include/control.h"
#include "../include/config.h"
#include "../include/logging.h"
#include <BasicLinearAlgebra.h>
#include <PWMServo.h>
using namespace BLA;


Matrix<3, 6> K = {10.0000000000000,	0,	0,	1.01242283656583,	0,	0,
                    0,	10.0000000000000,	0,	0,	1.04880884817015,	0,
                    0,	0,	10.0000000000000,	0,	0,	1.04880884817015,};
// K is the gain matrix for the LQR controller

const int n = 10000;
const int m = 10;



void control(double* quaternions, double* omega, PWMServo pitchServo, PWMServo yawServo) {
    //state vector
    Matrix<6, 1> x = {quaternions[1], quaternions[2], quaternions[3], omega[0], omega[1], omega[2]}; 
    Matrix<3, 1> u = -K * x; //u is the control torque vector
    //u(0) is roll torque, u(1) is pitch torque, u(2) is yaw torque
    double torque[2] = {u(1), u(2)}; //input is the control torque vector that we can actually affect, roll torque is not used in this case
    double gimbal[2] = {torque[0] / (MOMENT_ARM * THRUST), torque[1] / (MOMENT_ARM * THRUST)}; //gimbal angles in radians, torque is the control torque vector divided by the thrust and moment arm

    // Map the control torques to servo angles
    // Constrain the angles to gimbal servo limits
    // Calculate the gimbal angles in radians based on the control torques
    gimbal[0] = constrain(gimbal[0], -MAX_GIMBAL_ANGLE, MAX_GIMBAL_ANGLE); //Constrain the gimbal angles to the servo limits
    gimbal[1] = constrain(gimbal[1], -MAX_GIMBAL_ANGLE, MAX_GIMBAL_ANGLE); //Constrain the gimbal angles to the servo limits
    moveServos(gimbal, pitchServo, yawServo); //move servos to the calculated angles
}

void moveServos (double* gimbal, PWMServo pitchServo, PWMServo yawServo) {
    //gimbal input in radians to servo output
    
    double pitchAngle = SERVO_RATIO * RAD_TO_DEG * gimbal[0] - SERVO_OFFSET_PITCH; //Pitch servo angle in degrees
    double yawAngle = SERVO_RATIO * RAD_TO_DEG * gimbal[1] - SERVO_OFFSET_YAW; //Yaw servo angle in degrees
    
    
    // Serial.print("Pitch Servo Angle: ");
    // Serial.print(pitchAngle);
    // Serial.print("Yaw Servo Angle: ");
    // Serial.println(yawAngle);
    // // Write the angles to the servos
    pitchServo.write(pitchAngle);
    yawServo.write(yawAngle);

    double servos[2] = {pitchAngle, yawAngle};
    logControlData(gimbal, servos);
   
}

void invertTest() {
    Matrix<m,n> A;
    Matrix<n,m> B;
    for (int i = 0; i < m; i++){
        for (int j = 0; j < n; j++) {
            A(i,j) = random(20);
        }
    }
    for (int i = 0; i < n; i++){
        for (int j = 0; j < m; j++) {
            B(i,j) = random(20);
        }
    }
    Matrix<m,m> C = A*B;
}

