/** hardware.cpp
* ===========================================================
* Name: Flight Controller Hardware Implementation
* Section: TVC499
* Project: Flight Controller
* Purpose: LED and pyro channel control
* ===========================================================
*/

#include "../include/hardware.h"
#include "../include/config.h"
#include <Arduino.h>

bool Hardware::initialize() {
    pinMode(BUZZER_HIGH, OUTPUT);
    pinMode(BUZZER_LOW, OUTPUT);

    pinMode(LAUNCH_PYRO_FIRE, OUTPUT);
    pinMode(SEP_PYRO_FIRE, OUTPUT);
    digitalWrite(LAUNCH_PYRO_FIRE, LOW); 
    digitalWrite(SEP_PYRO_FIRE, LOW);
    pinMode(LAUNCH_PYRO_CONT, INPUT); // Set pyro continuity pins as input
    pinMode(SEP_PYRO_CONT, INPUT); // Set pyro continuity pins as input
    music(); // Play alert tone on startup
    // Serial.println("Inside hardware");
    return true;
}

void Hardware::playAlertTone(double frequency,  double duration) {
    double period = 1000000 / frequency; // Period in microseconds
    double halfPeriod = period / 2; // Half period in microseconds
    double startTime = millis(); 
    while (millis() - startTime < duration) { 
        digitalWrite(BUZZER_HIGH, HIGH); // Set buzzer high
        digitalWrite(BUZZER_LOW, LOW); // set buzzer low
        delayMicroseconds(halfPeriod); // Wait for half period
        
        digitalWrite(BUZZER_HIGH, LOW);
        digitalWrite(BUZZER_LOW, HIGH);
        delayMicroseconds(halfPeriod);
    }
    
    // Turn off buzzer
    digitalWrite(BUZZER_HIGH, LOW);
    digitalWrite(BUZZER_LOW, LOW);
}

void Hardware::update() {
    // Update hardware state
    checkPyroContinuity(); // Check pyro continuity
    setPyros(); // Set pyros based on trigger flags
}


void Hardware::checkPyroContinuity() {
    pyroContinuity1 = false;
    pyroContinuity2 = false; 
    double pyro1Value = analogRead(LAUNCH_PYRO_CONT); // Read analog value from pyro 1 continuity pin
    double pyro2Value = analogRead(SEP_PYRO_CONT); // Read analog value from pyro 2 continuity pin
    // Serial.print("Pyro 1 Continuity: ");
    // Serial.println(pyro1Value);    
    // Serial.print("Pyro 2 Continuity: ");
    // Serial.println(pyro2Value); 
    if (pyro1Value > CONTINUITY_THRESHOLD * (3.3 / 1023)) { //   3.3/1023 is the conversion factor for 10 bit ADC on teensy (0-1023 to 0-3.3V)
        pyroContinuity1 = true;
    }
    if (pyro2Value > CONTINUITY_THRESHOLD * (3.3 / 1023)) { 
        pyroContinuity2 = true; 
    } 

}

void Hardware::setPyros() {
    
    if (launchTriggered) {
        if (launchTimer == 0) { // If this is the first time we are firing the pyro
            launchTimer = millis(); // Start the timer
        }
        if (millis() - launchTimer >= PYRO_DURATION) { // Check if the pyro duration has passed
            launchTriggered = false; // Reset launch trigger flag after duration
            digitalWrite(LAUNCH_PYRO_FIRE, LOW); // Turn off pyro
        } else {
            digitalWrite(LAUNCH_PYRO_FIRE, HIGH); // Fire pyro
        }
    } else {
        digitalWrite(LAUNCH_PYRO_FIRE, LOW); // Turn off pyro
    }
    

    if (separationTriggered) {
        if (separationTimer == 0) { // If this is the first time we are firing the pyro
            separationTimer = millis(); // Start the timer
        }
        if (millis() - separationTimer >= PYRO_DURATION) { // Check if the pyro duration has passed
            separationTriggered = false; // Reset separation trigger flag after duration
            digitalWrite(SEP_PYRO_FIRE, LOW); // Turn off pyro
        } else {
            digitalWrite(SEP_PYRO_FIRE, HIGH); // Fire pyro
        }
    } else {
        digitalWrite(SEP_PYRO_FIRE, LOW); // Turn off pyro
    }

}

void Hardware::music() {
    for (int i = 0; i < 4; i++) {
        playAlertTone(2000 + i * 500, 200); // Play tone with increasing frequency 2000, 2500, 3000, 3500 hz
        delay(400); 
    }
    
}

void Hardware::setLaunchBool(bool setpoint) {
    launchTriggered = setpoint; // Set launch trigger flag
}

void Hardware::setSeparationBool(bool setpoint) {
    separationTriggered = setpoint; // Set separation trigger flag
}

// void Hardware::triggerSeparation() {
    
//     digitalWrite(SEP_PYRO_FIRE, HIGH); // Fire pyro
    
//     unsigned long separationStartTime = millis(); // Start timer for separation

//     separationTriggered = true;

//     if (separationTriggered && (millis() - separationStartTime >= PYRO_DURATION)) {
//         digitalWrite(SEP_PYRO_FIRE, LOW);
//     }


    
// }

// void triggerLaunch(bool& launchTriggered) {

//     digitalWrite(LAUNCH_PYRO_FIRE, HIGH); // Fire pyro
    
//     unsigned long launchStartTime = millis(); // Start timer for launch

//     launchTriggered = true;
        
//     if (launchTriggered && (millis() - launchStartTime >= PYRO_DURATION)) {
//         digitalWrite(LAUNCH_PYRO_FIRE, LOW);
//     }

// }


