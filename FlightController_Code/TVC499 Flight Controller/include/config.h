#ifndef CONFIG_H
#define CONFIG_H

//                                     Pyro Config
//-----------------------------------------------------------------------------------
#define LAUNCH_PYRO_FIRE 28
#define SEP_PYRO_FIRE 29
#define LAUNCH_PYRO_CONT A11  // continuity pyro 1
#define SEP_PYRO_CONT A12  // continuity pyro 2
#define PYRO_DURATION 4000  // 4 seconds
#define CONTINUITY_THRESHOLD 2 // Threshold for continuity detection (in volts)
//-----------------------------------------------------------------------------------


// Flight Parameters
#define THRUST_TIME 3000         // Thrust duration in milliseconds
#define APOGEE_DT 400            // Time interval for apogee detection
#define DELTA_H 0.5              // Altitude threshold for apogee detection
#define MAX_TILT_ANGLE 60        // Maximum tilt angle in degrees before abort

// Control Parameters
#define SERVO_RATIO 3.3 //Ratio of gimbal to servo angle
#define SERVO_OFFSET_PITCH 6.5 //Pitch servo offset in degrees
#define SERVO_OFFSET_YAW -2.5 //Yaw servo offset in degrees
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
#define BNO_RESET_PIN 15 // Pin for BNO reset
#define PITCH_SERVO_PIN 8 // Pin for pitch servo J2
#define YAW_SERVO_PIN 9 // Pin for yaw servo J1
#define COUNTDOWN_TIME 3000 // Countdown time in milliseconds



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
#define CALIBRATE 1
#define COUNTDOWN 2
#define ASCENT 3
#define UNPOWERED_ASCENT 4
#define DESCENT 5
#define ABORT 6
#define GROUND_IDLE 7

// Other Constants
#define TELEMETRY_INTERVAL 200  // Send serial data every 200ms
#define REF_PRESSURE_HPA 0  // Default reference pressure in hPa
// #define SD_CONFIG 


//Ring Buffer Size
#define BUFFER_SIZE 1500  // 5 seconds at 1000Hz



#endif // CONFIG_H