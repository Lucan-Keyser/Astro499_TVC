/** hardware.h
* ===========================================================
* Name: Flight Controller Hardware Interface
* Section: TVC499
* Project: Flight Controller
* Purpose: LED and pyro channel control
* ===========================================================
*/

#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include <FastLED.h>

// Pyro channel pins
#define PYRO1_FIRE 28
#define PYRO2_FIRE 29
#define PYRO1_CONTINUITY 25
#define PYRO2_CONTINUITY 26
#define BUZZER_HIGH 4
#define BUZZER_LOW 5
#define SEPARATION_DURATION 4000  // 4 seconds

/**
 * @brief Play alert tone using the buzzer
 * @param frequency Frequency of the tone in Hz
 * @param duration Duration of the tone in milliseconds
 */
void playAlertTone(int frequency, int duration);

/**
 * @brief Initialize hardware components (LEDs, pyro channels, buzzer)
 * @param leds Pointer to LED array
 * @param separationTriggered Pointer to separation flag
 * @param launchTriggered Pointer to launch flag
 */
void initializeHardware(bool& separationTriggered, bool& launchTriggered);

/**
 * @brief Check pyro continuity and buzz if continuity is detected
 * @return True if continuity is detected, false otherwise
 */

void checkPyroContinuity(double* continuity);

/**
 * @brief Trigger separation sequence (fire pyro channel 2)
 * @param leds Pointer to LED array
 * @param separationTriggered Pointer to separation flag
 */
void triggerSeparation(bool& separationTriggered);


/**
 * @brief Trigger launch sequence (fire pyro channels 1)
 * @param launchTriggered Pointer to launch flag
 */
void triggerLaunch(bool& launchTriggered);

void countDownMusic();


#endif // HARDWARE_H