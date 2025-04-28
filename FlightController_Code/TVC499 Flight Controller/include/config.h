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



// SENSOR Parameters
#define ALTIMETER_CALIBRATION_COUNT 100 // Number of samples for altimeter calibration
#define ALTIMETER_CALIBRATION_DELAY 10 // Delay between altimeter calibration samples in milliseconds
#define ORIENTATION_CALIBRATION_DELAY 30
#define BNO_RESET_PIN 15 // Pin for BNO reset
#define BNO_GYRO_SAMPLE_RATE 5000 // BNO085 sample rate in microseconds
#define BNO_ACCEL_SAMPLE_RATE 30000 //BNO85

// Control Parameters
#define SERVO_RATIO 3.41755 //Ratio of gimbal to servo angle
#define MOMENT_OF_INERTIA 0.0821 //Moment of inertia in kg*m^2
#define MOMENT_ARM 0.485 //Moment arm in meters
#define THRUST 15
#define K_PROPORTIONAL_GAIN 10.0
#define K_DERIVATIVE_GAIN 1.0332

//ACTUATOR PARAMETERS
#define SERVO_OFFSET_PITCH 6.5 //Pitch servo offset in degrees
#define SERVO_OFFSET_YAW -2.5 //Yaw servo offset in degrees
#define MAX_GIMBAL_ANGLE DEG_TO_RAD * 8
#define PITCH_SERVO_PIN 8 // Pin for pitch servo J2
#define YAW_SERVO_PIN 9 // Pin for yaw servo J1


//STATE PARAMETERS
#define LAUNCH_ACCELERATION -12 //launch detection limit
#define ABORT_CRITERIA 60 //how far we can pitch/yaw before aborting
#define BURN_TIME 3250 //burn time in millis
#define FREE_FALL_ACCEL 4// criteria for free fall acceleration, m/s^2.
#define ALTITUDE_THRESHOLD 0.75 // Altitude threshold for apogee detection in meters
#define APOGEE_DETECTION_INTERVAL 1000 // Time interval for apogee detection in milliseconds
#define APOGEE_DETECTION_COUNT 10 // Number of samples for apogee detection
#define APOGEE_DETECTION_DELAY 10 // Delay between apogee detection samples in milliseconds
#define EXPECTED_APOGEE_TIME 7500 //Expected apogee time in ms

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
#define RECOVERY 7

// Other Constants
#define TELEMETRY_INTERVAL 200  // Send serial data every 200ms
#define REF_PRESSURE_HPA 0  // Default reference pressure in hPa
#define SD_CONFIG SdioConfig(FIFO_SDIO)



//Ring Buffer Size
#define BUFFER_SIZE 2150  // 5 seconds at 1000Hz


//UNUSED PARAMETERS: 
#define THRUST_TIME 3000         // Thrust duration in milliseconds
#define APOGEE_DT 400            // Time interval for apogee detection
#define DELTA_H 0.5              // Altitude threshold for apogee detection
#define MAX_TILT_ANGLE 60        // Maximum tilt angle in degrees before abort
#define COUNTDOWN_TIME 3000 // Countdown time in millisecond


#endif // CONFIG_H