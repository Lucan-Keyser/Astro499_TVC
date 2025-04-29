#include <Arduino.h>
#include "MksServo.h"

// Pin definitions
const int SERVO_PIN = 9;  // PWM-capable pin on Teensy 4.1
const int POT_PIN = A0;   // Analog input for potentiometer feedback

// Create servo object
MksServo servo(SERVO_PIN, POT_PIN);

// Global variables
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 500;  // Update display every 500ms
bool runningDemo = false;
int demoStep = 0;
unsigned long demoStartTime = 0;

// Function prototypes
void processCommand(const String& command);
void runDemo();
void printStatus();
void printMenu();

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Wait up to 3 seconds for serial connection
  unsigned long startTime = millis();
  while (!Serial && millis() - startTime < 3000);
  
  Serial.println("\nMKS DS95i Servo Controller");
  Serial.println("=========================");
  
  // Initialize servo with calibrated values
  // 428-1072μs pulse range, 100° servo range, 560Hz frequency, 174° potentiometer range 428 980 
  servo.begin(544, 2400, 180.0, 50, 174.0);
  
  // Move to center position
  servo.centerServo();
  
  Serial.println("Servo initialized. Please calibrate with the 'C' command");
  printMenu();
}

void loop() {
  // Process any incoming serial commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    processCommand(input);
  }
  
  // If demo is running, update it
  if (runningDemo) {
    runDemo();
  }
  
  // Periodically display position if in monitoring mode
  if (millis() - lastDisplayTime > displayInterval) {
    lastDisplayTime = millis();
    
    // If calibrated, periodically show the current position
    if (servo.isCalibrated()) {
      printStatus();
    }
  }
}

void processCommand(const String& command) {
  char cmd = command.charAt(0);
  
  switch (cmd) {
    case 'C':
    case 'c':
      // Calibrate servo
      servo.calibrate();
      break;
      
    case 'A':
    case 'a':
      // Set angle
      {
        float angle = command.substring(1).toFloat();
        if (angle >= 0 && angle <= servo.getTotalRange()) {
          servo.setAngle(angle);
          Serial.print("Setting angle to: ");
          Serial.print(angle);
          Serial.println(" degrees");
        } else {
          Serial.print("Invalid angle. Range is 0 to ");
          Serial.println(servo.getTotalRange());
        }
      }
      break;
      
    case 'P':
    case 'p':
      // Set pulse width
      {
        unsigned long pulse = command.substring(1).toInt();
        if (pulse >= servo.getMinPulse() && pulse <= servo.getMaxPulse()) {
          servo.setPulseWidth(pulse);
          Serial.print("Setting pulse width to: ");
          Serial.print(pulse);
          Serial.println("μs");
        } else {
          Serial.print("Invalid pulse width. Range is ");
          Serial.print(servo.getMinPulse());
          Serial.print(" to ");
          Serial.print(servo.getMaxPulse());
          Serial.println("μs");
        }
      }
      break;
      
    case 'T':
    case 't':
      // Test speed from 0 to 60 degrees
      servo.testSpeedRange(0, 60);
      break;
      
    case 'F':
    case 'f':
      // Custom speed test
      {
        String params = command.substring(1);
        int commaPos = params.indexOf(',');
        
        if (commaPos > 0) {
          float startAngle = params.substring(0, commaPos).toFloat();
          float endAngle = params.substring(commaPos + 1).toFloat();
          
          if (startAngle >= 0 && startAngle <= servo.getTotalRange() && 
              endAngle >= 0 && endAngle <= servo.getTotalRange()) {
            servo.testSpeedRange(startAngle, endAngle);
          } else {
            Serial.print("Invalid angles. Range is 0 to ");
            Serial.println(servo.getTotalRange());
          }
        } else {
          Serial.println("Invalid format. Use 'Fx,y' where x and y are angles");
        }
      }
      break;
      
    case 'D':
    case 'd':
      // Run demo sequence
      Serial.println("Starting demo sequence...");
      runningDemo = true;
      demoStep = 0;
      demoStartTime = millis();
      break;
      
    case 'S':
    case 's':
      // Stop demo
      runningDemo = false;
      Serial.println("Demo stopped.");
      break;
      
    case 'M':
    case 'm':
      // Show current position
      printStatus();
      break;
      
    case 'H':
    case 'h':
    case '?':
      // Show menu
      printMenu();
      break;
      
    default:
      Serial.println("Unknown command. Press H for help.");
      break;
  }
}

void runDemo() {
  unsigned long elapsedTime = millis() - demoStartTime;
  
  // Stop demo after 30 seconds
  if (elapsedTime > 30000) {
    runningDemo = false;
    Serial.println("Demo complete.");
    return;
  }
  
  // Change positions every 3 seconds
  switch ((elapsedTime / 3000) % 6) {
    case 0:
      if (demoStep != 0) {
        Serial.println("Demo: Moving to minimum position (0°)");
        servo.setAngle(0);
        demoStep = 0;
      }
      break;
      
    case 1:
      if (demoStep != 1) {
        Serial.println("Demo: Moving to 25% position");
        servo.setPercentage(25);
        demoStep = 1;
      }
      break;
      
    case 2:
      if (demoStep != 2) {
        Serial.println("Demo: Moving to 50% position (center)");
        servo.centerServo();
        demoStep = 2;
      }
      break;
      
    case 3:
      if (demoStep != 3) {
        Serial.println("Demo: Moving to 75% position");
        servo.setPercentage(75);
        demoStep = 3;
      }
      break;
      
    case 4:
      if (demoStep != 4) {
        Serial.println("Demo: Moving to maximum position");
        servo.setAngle(servo.getTotalRange());
        demoStep = 4;
      }
      break;
      
    case 5:
      if (demoStep != 5) {
        Serial.println("Demo: Speed test from 0° to 60°");
        servo.testSpeedRange(0, 60);
        demoStep = 5;
      }
      break;
  }
}

void printStatus() {
  float angle = servo.getCurrentAngle();
  unsigned long pulse = servo.getCurrentPulseWidth();
  
  Serial.print("Position: ");
  Serial.print(angle, 1);
  Serial.print("° (");
  Serial.print(pulse);
  Serial.println("μs)");
}

void printMenu() {
  Serial.println("\nAvailable commands:");
  Serial.println("  C - Calibrate servo with potentiometer");
  Serial.println("  Axx - Set angle to xx degrees");
  Serial.println("  Pxxxx - Set pulse width to xxxx microseconds");
  Serial.println("  T - Test speed from 0 to 60 degrees");
  Serial.println("  Fx,y - Test speed from x to y degrees");
  Serial.println("  D - Run demo sequence");
  Serial.println("  S - Stop demo sequence");
  Serial.println("  M - Show current position");
  Serial.println("  H - Show this help menu");
}