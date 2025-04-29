#include "MksServo.h"

/**
 * Constructor
 */
MksServo::MksServo(uint8_t servoPin, uint8_t potPin) : 
    servoPin(servoPin), 
    potPin(potPin),
    minPulseWidth(544),
    maxPulseWidth(1072),
    totalAngleRange(100.0),
    potAngleRange(174.0),
    frequency(560),
    minPotValue(0),
    maxPotValue(1023),
    currentPulseWidth(0),
    calibrated(false)
{
    // Initialize calculated values
    pulseRange = maxPulseWidth - minPulseWidth;
    period = 1000000 / frequency;
}

/**
 * Initialize the servo
 */
void MksServo::begin(unsigned long minPulse, unsigned long maxPulse, 
                     float totalRange, unsigned long freq, float potRange) {
    // Set parameters
    minPulseWidth = minPulse;
    maxPulseWidth = maxPulse;
    pulseRange = maxPulseWidth - minPulseWidth;
    totalAngleRange = totalRange;
    potAngleRange = potRange;
    frequency = freq;
    period = 1000000 / frequency;
    
    // Initialize pins
    pinMode(servoPin, OUTPUT);
    pinMode(potPin, INPUT);
    
    // Configure PWM for maximum precision
    analogWriteResolution(16);  // 16-bit resolution for Teensy
    analogWriteFrequency(servoPin, frequency);
    
    // Move to center position
    centerServo();
}

/**
 * Calibrate the servo with the potentiometer
 */
bool MksServo::calibrate() {
    Serial.println("\n=== SERVO CALIBRATION ===");
    Serial.println("This will determine the relationship between servo and potentiometer");
    Serial.println("Note: Potentiometer range is set to 174 degrees");
    
    // Step 1: Move to minimum position
    Serial.println("\nMoving servo to minimum position...");
    setPWM(minPulseWidth);
    currentPulseWidth = minPulseWidth;
    delay(1500);
    
    Serial.println("Press any key when ready to record minimum position...");
    while (!Serial.available()) { delay(10); }
    while (Serial.available()) { Serial.read(); }
    
    minPotValue = readPotWithFiltering(20);
    Serial.print("Min position potentiometer reading: ");
    Serial.println(minPotValue);
    
    // Step 2: Move to maximum position
    Serial.println("\nMoving servo to maximum position...");
    setPWM(maxPulseWidth);
    currentPulseWidth = maxPulseWidth;
    delay(1500);
    
    Serial.println("Press any key when ready to record maximum position...");
    while (!Serial.available()) { delay(10); }
    while (Serial.available()) { Serial.read(); }
    
    maxPotValue = readPotWithFiltering(20);
    Serial.print("Max position potentiometer reading: ");
    Serial.println(maxPotValue);
    
    // Check if calibration is valid
    if (abs(maxPotValue - minPotValue) < 100) {
        Serial.println("\nCalibration FAILED: Potentiometer range too small.");
        Serial.println("Please check the connection and alignment.");
        return false;
    }
    
    calibrated = true;
    
    // Calculate the scaling factor between pot and servo angles
    float potToServoRatio = totalAngleRange / potAngleRange;
    Serial.print("Potentiometer to servo ratio: ");
    Serial.println(potToServoRatio, 4);
    
    // Move back to center
    centerServo();
    
    Serial.println("\nCalibration SUCCESSFUL!");
    Serial.print("Servo range: ");
    Serial.print(totalAngleRange);
    Serial.println(" degrees");
    Serial.print("Potentiometer range: ");
    Serial.print(potAngleRange);
    Serial.println(" degrees");
    return true;
}

/**
 * Set servo position by angle
 */
void MksServo::setAngle(float angle) {
    float constrainedAngle = constrain(angle, 0, totalAngleRange);
    currentPulseWidth = angleToPulse(constrainedAngle);
    setPWM(currentPulseWidth);
}

/**
 * Set servo position by pulse width
 */
void MksServo::setPulseWidth(unsigned long pulseWidth) {
    unsigned long constrainedPulse = constrain(pulseWidth, minPulseWidth, maxPulseWidth);
    currentPulseWidth = constrainedPulse;
    setPWM(currentPulseWidth);
}

/**
 * Set servo position by percentage of range
 */
void MksServo::setPercentage(float percentage) {
    float constrainedPercentage = constrain(percentage, 0, 100);
    currentPulseWidth = minPulseWidth + (pulseRange * constrainedPercentage / 100.0);
    setPWM(currentPulseWidth);
}

/**
 * Get current angle from potentiometer
 */
float MksServo::getCurrentAngle() {
    if (!calibrated) {
        return -1.0; // Indicate not calibrated
    }
    
    int potValue = readPotWithFiltering();
    
    // Map potentiometer value to an angle position within its physical range
    int potRange = abs(maxPotValue - minPotValue);
    float direction = (maxPotValue > minPotValue) ? 1.0 : -1.0;
    float potPositionPercent;
    
    if (direction > 0) {
        potPositionPercent = (float)(potValue - minPotValue) / potRange;
    } else {
        potPositionPercent = (float)(maxPotValue - potValue) / potRange;
    }
    
    // Convert potentiometer position to servo angle, accounting for different ranges
    float servoAngle = potPositionPercent * totalAngleRange;
    
    return servoAngle;
}

/**
 * Get current pulse width
 */
unsigned long MksServo::getCurrentPulseWidth() {
    return currentPulseWidth;
}

/**
 * Measure servo movement time using potentiometer feedback
 * This function precisely measures how long it takes for the servo to move
 * between two positions by monitoring the potentiometer readings
 */
float MksServo::measureSpeed(float startAngle, float endAngle) {
    if (!calibrated) {
        Serial.println("Error: Servo not calibrated. Run calibrate() first.");
        return 0.0;
    }
    
    // Constrain angles to valid range
    startAngle = constrain(startAngle, 0, totalAngleRange);
    endAngle = constrain(endAngle, 0, totalAngleRange);
    float angleDiff = abs(endAngle - startAngle);
    
    if (angleDiff < 7.0) {
        Serial.println("Warning: Angle difference too small for accurate measurement.");
        return 0.0;
    }
    
    // Move to start position and wait for it to settle
    setAngle(startAngle);
    delay(1000);
    
    // Record actual start position from potentiometer
    float actualStartAngle = getCurrentAngle();
    Serial.print("Starting position: ");
    Serial.print(actualStartAngle, 1);
    Serial.println(" degrees");
    
    // Define movement detection thresholds
    const float startThreshold = totalAngleRange * 0.01; // 1% movement to detect start
    const float endThreshold = totalAngleRange * 0.05;   // 2% of target to detect completion
    
    // Variables for timing
    unsigned long movementStartTime = 0;
    unsigned long movementEndTime = 0;
    bool movementStarted = false;
    bool movementComplete = false;
    
    // Variables for movement tracking
    float previousAngle = actualStartAngle;
    float currentAngle = actualStartAngle;
    float targetAngle = endAngle;
    
    // Start the movement
    Serial.println("Starting movement...");
    unsigned long commandTime = micros();
    setAngle(endAngle);
    
    // Monitor position until movement completes or timeout
    unsigned long timeout = 5000000; // 2 second timeout (in microseconds)
    unsigned long startTime = micros();
    
    while ((micros() - startTime) < timeout) {
        // Read current position
        currentAngle = getCurrentAngle();
        
        // Check if movement has started (position changed beyond noise)
        if (!movementStarted && abs(currentAngle - actualStartAngle) > startThreshold) {
            movementStarted = true;
            movementStartTime = micros();
            Serial.print("Movement detected at: ");
            Serial.print((movementStartTime - commandTime) / 1000.0, 3);
            Serial.println(" ms after command");
        }
        
        // Check if movement has completed (position close to target)
        if (movementStarted && !movementComplete && abs(currentAngle - targetAngle) < endThreshold) {
            movementComplete = true;
            movementEndTime = micros();
            Serial.print("Target reached at: ");
            Serial.print((movementEndTime - movementStartTime) / 1000.0, 3);
            Serial.println(" ms");
            break;
        }
        
        // Update previous angle
        previousAngle = currentAngle;
        
        // Small delay to prevent overwhelming the ADC
        delayMicroseconds(500);
    }
    
    // Check if we timed out
    if (!movementComplete) {
        Serial.println("Warning: Movement did not complete within timeout period.");
        return 0.0;
    }
    
    // Calculate final values
    float actualEndAngle = getCurrentAngle();
    float actualAngleDiff = abs(actualEndAngle - actualStartAngle);
    float movementTime = (movementEndTime - movementStartTime) / 1000000.0; // Convert to seconds
    float speed = actualAngleDiff / movementTime;
    
    // Print detailed results
    Serial.println("\n=== MOVEMENT ANALYSIS ===");
    Serial.print("Command: ");
    Serial.print(startAngle, 1);
    Serial.print("° to ");
    Serial.print(endAngle, 1);
    Serial.print("° (");
    Serial.print(angleDiff, 1);
    Serial.println("°)");
    
    Serial.print("Actual: ");
    Serial.print(actualStartAngle, 1);
    Serial.print("° to ");
    Serial.print(actualEndAngle, 1);
    Serial.print("° (");
    Serial.print(actualAngleDiff, 1);
    Serial.println("°)");
    
    Serial.print("Delay before movement: ");
    Serial.print((movementStartTime - commandTime) / 1000.0, 3);
    Serial.println(" ms");
    
    Serial.print("Movement time: ");
    Serial.print(movementTime * 1000, 3);
    Serial.println(" ms");
    
    Serial.print("Average speed: ");
    Serial.print(speed, 2);
    Serial.println(" degrees/second");
    
    // Compare with datasheet
    float specTime = 0.038; // seconds per 60 degrees at 6V
    float specSpeed = 60.0 / specTime;
    
    Serial.print("Datasheet speed: ~");
    Serial.print(specSpeed, 2);
    Serial.println(" degrees/second (at 6V)");
    
    // Return the average speed
    return speed;
}

/**
 * Test and report speed between two angles
 */
void MksServo::testSpeedRange(float startAngle, float endAngle) {
    Serial.print("\n=== TESTING SPEED FROM ");
    Serial.print(startAngle);
    Serial.print(" TO ");
    Serial.print(endAngle);
    Serial.println(" DEGREES ===");
    
    float speed = measureSpeed(startAngle, endAngle);
    
    if (speed > 0) {
        float angleDiff = abs(endAngle - startAngle);
        float movementTime = angleDiff / speed;
        
        // Compare with datasheet
        float specTime = 0.038; // seconds per 60 degrees at 6V
        float specSpeed = 60.0 / specTime;
        
        Serial.print("Datasheet speed: ~");
        Serial.print(specSpeed, 2);
        Serial.println(" degrees/second (at 6V)");
        Serial.print("Your servo is ");
        if (speed > specSpeed * 1.1) {
            Serial.print("faster than spec by ");
            Serial.print(((speed / specSpeed) - 1) * 100, 1);
            Serial.println("%");
        } else if (speed < specSpeed * 0.9) {
            Serial.print("slower than spec by ");
            Serial.print((1 - (speed / specSpeed)) * 100, 1);
            Serial.println("%");
        } else {
            Serial.println("running at approximately the specified speed");
        }
    }
}

/**
 * Move servo to center position
 */
void MksServo::centerServo() {
    unsigned long centerPulse = minPulseWidth + (pulseRange / 2);
    setPWM(centerPulse);
    currentPulseWidth = centerPulse;
}

/**
 * Convert angle to pulse width
 */
unsigned long MksServo::angleToPulse(float angle) {
    float constrainedAngle = constrain(angle, 0, totalAngleRange);
    return minPulseWidth + (constrainedAngle / totalAngleRange) * pulseRange;
}

/**
 * Convert pulse width to angle
 */
float MksServo::pulseToAngle(unsigned long pulse) {
    unsigned long constrainedPulse = constrain(pulse, minPulseWidth, maxPulseWidth);
    return (constrainedPulse - minPulseWidth) * totalAngleRange / pulseRange;
}

/**
 * Set PWM signal directly
 */
void MksServo::setPWM(unsigned long pulseWidth) {
    // Calculate duty cycle for 16-bit resolution
    uint32_t duty = (pulseWidth * 65535) / period;
    duty = constrain(duty, 0, 65535);
    
    // Apply PWM
    analogWrite(servoPin, duty);
}

/**
 * Read potentiometer with noise filtering
 */
int MksServo::readPotWithFiltering(int samples) {
    long total = 0;
    for (int i = 0; i < samples; i++) {
        total += analogRead(potPin);
        delay(1);
    }
    return total / samples;
}