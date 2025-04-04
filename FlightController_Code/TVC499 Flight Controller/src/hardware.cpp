/** hardware.cpp
* ===========================================================
* Name: Flight Controller Hardware Implementation
* Section: TVC499
* Project: Flight Controller
* Purpose: LED and pyro channel control
* ===========================================================
*/

#include "../include/hardware.h"

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

void initializeHardware(CRGB* leds, bool* separationTriggered, 
                       unsigned long* separationStartTime) {
    // Initialize separation state
    *separationTriggered = false;
    *separationStartTime = 0;
    
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
    
    // Set initial LED state
    leds[0] = CRGB::Black;
    leds[1] = CRGB::Black;
    FastLED.show();
}

bool checkPyroContinuity() {
    // Check continuity of pyro channels 
    pinMode(PYRO1_CONTINUITY, INPUT_PULLUP);
    pinMode(PYRO2_CONTINUITY, INPUT_PULLUP);
    
    bool continuity1 = digitalRead(PYRO1_CONTINUITY) == LOW;
    bool continuity2 = digitalRead(PYRO2_CONTINUITY) == LOW;
    
    if (continuity1) {
        // Play special tone for continuity detection on PYRO1
        for (int i = 0; i < 5; i++) {
            playAlertTone(2000, 500); // Play tone at 3000 Hz 
            delay(200); // Delay between tones
        }

        Serial.println("Continuity detected on PYRO1!");

        return true;
    }
    if (continuity2) {
        // Play special tone for continuity detection on PYRO2
        for (int i = 0; i < 5; i++) {
            playAlertTone(4000, 500); // Play tone at 4000 Hz 
            delay(200); // Delay between tones
        }
        Serial.println("Continuity detected on PYRO2!");
        return true;

    } else {
        Serial.println("No continuity detected.");

        return false;
    }
}

void triggerSeparation(bool* separationTriggered, unsigned long* separationStartTime) {
    
    // Fire pyro channels
    digitalWrite(PYRO1_FIRE, HIGH);
    digitalWrite(PYRO2_FIRE, HIGH);
    
    // Start separation timer
    *separationStartTime = millis();
    *separationTriggered = true;

    // Play alert tone for separation
    playAlertTone(3000, 4000); // Play tone at 3000 Hz for 500 ms

    if (*separationTriggered && 
        (millis() - *separationStartTime >= SEPARATION_DURATION)) {
        digitalWrite(PYRO1_FIRE, LOW);
        digitalWrite(PYRO2_FIRE, LOW);
        *separationTriggered = false;
        
        Serial.println("Separation sequence completed");
    }
    else {
        Serial.println("Separation sequence in progress");
    }

    
    Serial.println("Separation triggered!");
}

