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
    music(); // Play alert tone on startup
    return true;
}

void Hardware::playAlertTone(int frequency, int duration) {
    unsigned long period = 1000000 / frequency; // Period in microseconds
    unsigned long halfPeriod = period / 2; // Half period in microseconds
    unsigned long startTime = millis(); 
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



void Hardware::checkPyroContinuity() {
    pyroContinuity1 = false;
    pyroContinuity2 = 0.0; 
    int pyro1Value = analogRead(LAUNCH_PYRO_CONT); // Read analog value from pyro 1 continuity pin
    int pyro2Value = analogRead(SEP_PYRO_CONT); // Read analog value from pyro 2 continuity pin

    if (pyro1Value > CONTINUITY_THRESHOLD * (3.3 / 1023)) { //   3.3/1023 is the conversion factor for 10 bit ADC on teensy (0-1023 to 0-3.3V)
        pyroContinuity1 = true;
    }
    if (pyro2Value > CONTINUITY_THRESHOLD * (3.3 / 1023)) { 
        pyroContinuity2 = true; 
    } 
}

void Hardware::setPyros() {

    if (launchTriggered) {
        digitalWrite(LAUNCH_PYRO_FIRE, HIGH); // Fire pyro
    } else {
        digitalWrite(LAUNCH_PYRO_FIRE, LOW); // Turn off pyro
    }

    if (separationTriggered) {
        digitalWrite(SEP_PYRO_FIRE, HIGH); // Fire pyro
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


