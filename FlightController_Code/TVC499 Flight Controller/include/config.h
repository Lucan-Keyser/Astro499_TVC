#ifndef CONFIG_H
#define CONFIG_H

// Pyro Pins
#define PYRO1_FIRE 28
#define PYRO2_FIRE 29
#define PYRO1_CONT A11  // continuity pyro 1
#define PYRO2_CONT A12  // continuity pyro 2
#define CONTINUITY_THRESHOLD 2

// Flight Parameters
#define THRUST_TIME 3000         // Thrust duration in milliseconds
#define APOGEE_DT 400            // Time interval for apogee detection
#define DELTA_H 0.5              // Altitude threshold for apogee detection
#define SEPARATION_DURATION 4000 // Separation duration in milliseconds
#define MAX_TILT_ANGLE 60        // Maximum tilt angle in degrees before abort

// Control Parameters
#define SERVO_RATIO 3.3 //Ratio of gimbal to servo angle
#define SERVO_OFFSET_PITCH 0 //Pitch servo offset in degrees
#define SERVO_OFFSET_YAW 0 //Yaw servo offset in degrees
#define MAX_GIMBAL_ANGLE DEG_TO_RAD * 8
#define MOMENT_OF_INERTIA 0.1 //Moment of inertia in kg*m^2
#define MOMENT_ARM 0.49 //Moment arm in meters
#define THRUST 15
#define LAUNCH_ACCELERATION -15 //launch detection limit
#define ABORT_CRITERIA 60 //how far we can pitch/yaw before aborting
#define BURN_TIME 3250 //burn time in millis
#define FREE_FALL_ACCEL 4// criteria for free fall acceleration, m/s^2.
#define ALTIMETER_CALIBRATION_COUNT 100 // Number of samples for altimeter calibration
#define ALTIMETER_CALIBRATION_DELAY 10 // Delay between altimeter calibration samples in milliseconds
#define ALTITUDE_THRESHOLD 0.75 // Altitude threshold for apogee detection in meters
#define APOGEE_DETECTION_INTERVAL 1000 // Time interval for apogee detection in milliseconds
#define APOGEE_DETECTION_COUNT 10 // Number of samples for apogee detection
#define APOGEE_DETECTION_DELAY 10 // Delay between apogee detection samples in milliseconds






// LED Configuration
#define NUM_LEDS 2
#define LED_DATA_PIN 3

// Buzzer Pins
#define BUZZER_HIGH 4
#define BUZZER_LOW 5

// LoRa Pins
#define RFM95_CS 10
#define RFM95_RST 23
#define RFM95_INT 1
#define RF95_FREQ 915.0

// State Definitions
#define PAD_IDLE 0
#define COUNTDOWN 1
#define ASCENT 2
#define UNPOWERED_ASCENT 3
#define DESCENT 4
#define ABORT -1

// Other Constants
#define TELEMETRY_INTERVAL 50    // Send telemetry every 50ms



#endif // CONFIG_H