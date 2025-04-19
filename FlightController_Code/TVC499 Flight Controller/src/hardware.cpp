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

void playAlertTone(int frequency, int duration) {
    unsigned long period = 1000000 / frequency; // Period in microseconds
    unsigned long halfPeriod = period / 2; // Half period in microseconds
    unsigned long startTime = millis(); 
    //how does this work? the frequency is the number of times the buzzer turns on and off in a second.
    // the frequency is the number of cycles per second, so the period is the time it takes for one cycle of the wave.
    //buzzer is normally 2khz to 4khz, so the period is 0.5ms to 0.25ms.
    // The half period is half of that time, so the buzzer will be on for 0.25ms to 0.125ms and off for the same amount of time.
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

void initializeHardware(bool& separationTriggered) {
    // Initialize separation state
    separationTriggered = false;
    
    // Initialize buzzer pins
    pinMode(BUZZER_HIGH, OUTPUT);
    pinMode(BUZZER_LOW, OUTPUT);
    
    // Play startup tone
    //give a boot up tone of beep beep beep beep progressively getting higher in pitch
    for (int i = 0; i < 4; i++) {
        playAlertTone(2000 + i * 500, 200); // Play tone with increasing frequency 2000, 2500, 3000, 3500 hz
        delay(400); // Delay between tones
    }

    // Initialize Pyro Pins
    pinMode(PYRO1_FIRE, OUTPUT);
    pinMode(PYRO2_FIRE, OUTPUT);
    digitalWrite(PYRO1_FIRE, LOW);
    digitalWrite(PYRO2_FIRE, LOW);
    
    // Set initial LED state, not present on PCB
//     leds[0] = CRGB::Black;
//     leds[1] = CRGB::Black;
//     FastLED.show();
}

void checkPyroContinuity(double* continuity) {
 
    // Check continuity of pyro channels 
    int pyro1Value = analogRead(A11); // Read analog value from pyro 1 continuity pin
    int pyro2Value = analogRead(A12); // Read analog value from pyro 2 continuity pin

    // Check if both pyro channels have continuity
    if (pyro1Value > CONTINUITY_THRESHOLD * (3.3 / 1023)) {
        // Serial.println("Continuity detected on pyro channel 1.");
        continuity[0] = 1.0; // Set continuity to 1.0 for pyro channel 1
    } else if (pyro2Value > CONTINUITY_THRESHOLD * (3.3 / 1023)) {
        // Serial.println("Continuity detected on pyro channel 1.");
        continuity[1] = 1.0; // Set continuity to 0.0 for pyro channel 2
    } 
}

void triggerSeparation(bool* separationTriggered) {
    
    // Fire pyro channels
    digitalWrite(PYRO2_FIRE, HIGH);
    
    // Start separation timer
    unsigned long separationStartTime = millis();
    *separationTriggered = true;

    Serial.println("Separation triggered!");

    // Play alert tone for separation
    playAlertTone(3000, 4000); // Play tone at 3000 Hz for 500 ms

    if (*separationTriggered && (millis() - separationStartTime >= SEPARATION_DURATION)) {
        digitalWrite(PYRO2_FIRE, LOW);
        
        Serial.println("Separation sequence completed");
    }
    else {
        Serial.println("Separation sequence in progress");
    }

    
}

void triggerLaunch(bool* launchTriggered) {
    // Fire pyro channel 1
    digitalWrite(PYRO1_FIRE, HIGH);
    
    // Start launch timer
    unsigned long launchStartTime = millis();
    *launchTriggered = true;
    
    Serial.println("Launch triggered!");
    
    // Play alert tone for launch
    playAlertTone(4000, 4000); // Play tone at 4000 Hz for 500 ms
    
    if (*launchTriggered && (millis() - launchStartTime >= SEPARATION_DURATION)) {
        digitalWrite(PYRO1_FIRE, LOW);
        
        Serial.println("Launch sequence completed");
    }
    else {
        Serial.println("Launch sequence in progress");
    }
}

