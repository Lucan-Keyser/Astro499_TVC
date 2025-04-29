#ifndef MKS_SERVO_H
#define MKS_SERVO_H

#include <Arduino.h>

class MksServo {
public:
    // Constructor
    MksServo(uint8_t servoPin, uint8_t potPin);

    // Initialization with default parameters 428 1072
    void begin(unsigned long minPulse = 544, unsigned long maxPulse = 2250, 
               float totalRange = 100.0, unsigned long frequency = 50,
               float potRange = 10.0);

    // Calibration
    bool calibrate();
    
    // Position control
    void setAngle(float angle);
    void setPulseWidth(unsigned long pulseWidth);
    void setPercentage(float percentage);
    
    // Position feedback
    float getCurrentAngle();
    unsigned long getCurrentPulseWidth();
    
    // Speed testing
    float measureSpeed(float startAngle, float endAngle);
    void testSpeedRange(float startAngle, float endAngle);
    
    // Center position
    void centerServo();
    
    // Position calculation helpers
    unsigned long angleToPulse(float angle);
    float pulseToAngle(unsigned long pulse);
    
    // Getter methods
    float getTotalRange() const { return totalAngleRange; }
    unsigned long getMinPulse() const { return minPulseWidth; }
    unsigned long getMaxPulse() const { return maxPulseWidth; }
    bool isCalibrated() const { return calibrated; }

private:
    // Hardware pins
    uint8_t servoPin;
    uint8_t potPin;
    
    // Servo parameters
    unsigned long minPulseWidth;   // Minimum pulse width (μs)
    unsigned long maxPulseWidth;   // Maximum pulse width (μs)
    unsigned long pulseRange;      // Range of pulse width (μs)
    float totalAngleRange;         // Total angle range in degrees
    float potAngleRange;           // Potentiometer's mechanical range in degrees
    
    // PWM settings
    unsigned long frequency;       // PWM frequency in Hz
    unsigned long period;          // PWM period in μs
    
    // Potentiometer calibration values
    int minPotValue;               // Pot reading at min position
    int maxPotValue;               // Pot reading at max position
    
    // Current state
    unsigned long currentPulseWidth;
    bool calibrated;
    
    // Helper methods
    void setPWM(unsigned long pulseWidth);
    int readPotWithFiltering(int samples = 10);
};

#endif