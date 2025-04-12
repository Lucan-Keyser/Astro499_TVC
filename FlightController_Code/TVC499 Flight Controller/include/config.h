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
#define SERVO_RATIO 3
#define SERVO_OFFSET_YAW -14
#define SERVO_OFFSET_PITCH -11
#define PID_P 1.5
#define PID_I 0.2
#define PID_D 1.73
#define MAX_SERVO_ANGLE 7

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
#define REF_PRESSURE_HPA 700     // Default reference pressure in hPa

#endif // CONFIG_H