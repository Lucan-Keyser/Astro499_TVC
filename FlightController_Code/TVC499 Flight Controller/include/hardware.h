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


class Hardware {
    private:
        bool separationTriggered = false;
        bool launchTriggered = false;
        bool pyroContinuity1 = false; // Pyro 1 continuity 
        bool pyroContinuity2 = false; // Pyro 2 continuity
        double launchTimer = 0; // Timer for launch sequence
        double separationTimer = 0; // Timer for separation sequence


    public:
        Hardware() {}
       
        bool initialize();
        void playAlertTone(double frequency,  double duration);
        void checkPyroContinuity();
        void setPyros();
        void music();
        void setLaunchBool(bool setpoint);
        void setSeparationBool(bool setpoint);
        void update();

        bool getLaunchTriggered() { return launchTriggered; } // Getter for launch trigger
        bool getSeparationTriggered() { return separationTriggered; } // Getter for separation trigger
        bool getPyroContinuity1() { return pyroContinuity1; } // Getter for pyro 1 continuity
        bool getPyroContinuity2() { return pyroContinuity2; } // Getter for pyro 2 continuity


    };



#endif // HARDWARE_H