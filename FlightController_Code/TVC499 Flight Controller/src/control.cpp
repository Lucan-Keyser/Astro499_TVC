#include "../include/control.h"
#include "../include/config.h"

#include <BasicLinearAlgebra.h>
#include <PWMServo.h>
using namespace BLA;



// K is the gain matrix for the LQR controller

bool Control::initialize() {
    // Initialize the control system
    return true;
}
void Control::control() {
    //state vector
    *quaternions = sensors.getQuaternions(); //quaternion vector
    *omega = sensors.getGyroRates(); //angular velocity vector

    Matrix<6, 1, double> x = {*quaternions[1], *quaternions[2], *quaternions[3], *omega[0], *omega[1], *omega[2]}; 
    Matrix<3, 1, double> u = -K * x; //u is the control torque vector
    //u(0) is roll torque, u(1) is pitch torque, u(2) is yaw torque
    torque[0] = u(1);
    torque[1] = u(2); //input is the control torque vector that we can actually affect, roll torque is not used in this case

    gimbalAngles[0] = torque[0] / (MOMENT_ARM * THRUST);
    gimbalAngles[1] = torque[1] / (MOMENT_ARM * THRUST); //gimbal angles in radians, torque is the control torque vector divided by the thrust and moment arm

    // Map the control torques to servo angles
    // Constrain the angles to gimbal servo limits
    // Calculate the gimbal angles in radians based on the control torques
    gimbalAngles[0] = constrain(gimbalAngles[0], -MAX_GIMBAL_ANGLE, MAX_GIMBAL_ANGLE); //Constrain the gimbal angles to the servo limits
    gimbalAngles[1] = constrain(gimbalAngles[1], -MAX_GIMBAL_ANGLE, MAX_GIMBAL_ANGLE); //Constrain the gimbal angles to the servo limits

    if (!controlActive) { //if the control is not active, set the gimbal angles to 0
        gimbalAngles[0] = 0.0;
        gimbalAngles[1] = 0.0;
    }
    actuators.moveServos(gimbalAngles); //move the servos to the gimbal angles
    
    
}

void Control::updateControlBoolean(bool setpoint) {
    controlActive = setpoint; //set the control active flag to the setpoint

}




