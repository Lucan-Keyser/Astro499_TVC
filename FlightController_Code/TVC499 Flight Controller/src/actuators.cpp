#include "../include/control.h"
#include "../include/config.h"
#include "../include/actuators.h"
#include <BasicLinearAlgebra.h>
#include <PWMServo.h>


bool Actuators::initialize() {
        // Initialize the control system
        pitchServo->attach(PITCH_SERVO_PIN);  // Attach pitch servo to pin J2
        yawServo->attach(YAW_SERVO_PIN);  // Attach yaw servo to pin J1
        double init[2] = {0.0, 0.0}; //initial angles in radians
        moveServos(init); //move servos to the initial angles
        return true;
}

void Actuators::moveServos(double* gimbalAngles) {
    gimbalAngles[0] = -gimbalAngles[0];
    //gimbal input in radians to servo output
    
    servoAngles[0] = SERVO_RATIO * RAD_TO_DEG * gimbalAngles[0] - SERVO_OFFSET_PITCH + 90; //Pitch servo angle in degrees
    servoAngles[1] = SERVO_RATIO * RAD_TO_DEG * gimbalAngles[1] - SERVO_OFFSET_YAW + 90; //Yaw servo angle in degrees
    
    
    // Serial.print("Pitch Servo Angle: ");
    // Serial.print(pitchAngle);
    // Serial.print("Yaw Servo Angle: ");
    // Serial.println(yawAngle);
    // // Write the angles to the servos

    pitchServo->write(servoAngles[0]);
    yawServo->write(servoAngles[1]);

}