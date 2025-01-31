#include <Orientation.h>
#include <Quaternion.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <utility/imumaths.h>
#include <PWMServo.h>
#include <FastLED.h>
#include <RH_RF95.h>

// Define Pyro Pins
#define PYRO1_FIRE 28
#define PYRO2_FIRE 29



// Separation Variables
bool separationTriggered = false;
unsigned long separationStartTime = 0;
const unsigned long separationDuration = 4000; // 4 second

// Initialize sensor objects
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP3XX bmp;
float REFPRESSURE_HPA = 700;

// Orientation data
Orientation ori;
EulerAngles oriMeasure;
double* quatAngles = new double[3];
double* gyroRates = new double[3];
double* localAngles = new double[3];
double* linearAccelerations = new double[3];
double* accelData = new double[3];
double* gravData = new double[3];
double* altData = new double[3];
double* accelAngles = new double[3];
double* intAngles = new double[3];
double* complAngles = new double[3];

// Time tracking
double dt = 0;
double lastTime = micros();
double flightStart = 0;
double flightTime = 0;
double thrustTime = 3000;
double apogeeDt = 400;
double deltaH = 0.5;
double apogeeLast = 0;
double previousAltitude = 0;

// State Variables
#define PAD_IDLE 0
#define COUNTDOWN 1
#define ASCENT 2
#define UNPOWERED_ASCENT 3
#define DESCENT 4
#define ABORT -1
int currentState = PAD_IDLE;

// Interface variables
double LEDTimer = millis();
String command = "";

// Servo setup
PWMServo yawServo;
PWMServo pitchServo;

// PWM Variables
#define RATIO 3
#define OFFSET2 -11
#define OFFSET1 -14
#define P 1.5
#define I 0.2
#define D 1.73
#define MAX_ANGLE 7
#define NUM_LEDS 2
#define DATA_PIN 3
#define buzz_high 4
#define buzz_low 5

// LED setup
CRGB leds[NUM_LEDS];

// RF95 Pins
#define RFM95_CS 10
#define RFM95_RST 23
#define RFM95_INT 1
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

double quat_last_angle1 = 0.0;
double quat_last_angle2 = 0.0;
double quatDelta1 = 0.0;
double quatDelta2 = 0.0;
double quatIntegral1 = 0;
double quatIntegral2 = 0;
double complIntegral1 = 0;
double complIntegral2 = 0;
double last_gyro1 = 0;
double last_gyro2 = 0;
double accel1 = 0;
double accel2 = 0;
double a1Des = 0;
double a2Des = 0;

unsigned long lastTelemetryTime = 0;  // Tracks the last time telemetry was sent
const unsigned long telemetryInterval = 50;  // Send telemetry every 50 ms


void setup(void) {
  Serial.begin(9600);
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);

  if (!bno.begin()) {
    Serial.println("No BNO055 detected!");
    while (1);
  }

  Wire1.begin();
  if (!bmp.begin_I2C(0x77, &Wire1)) {
    Serial.println("No BMP3 sensor detected!");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  reset();
  
  yawServo.attach(8);
  pitchServo.attach(9);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("RF95 LoRa init failed!");
    while (1);
  }

  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);

  pinMode(buzz_high, OUTPUT);
  pinMode(buzz_low, OUTPUT);
  tone(buzz_high, 500);
  delay(1500);
  noTone(buzz_high);
  digitalWrite(buzz_high, LOW);
  digitalWrite(buzz_low, LOW);

  // Initialize Pyro Pins
  pinMode(PYRO1_FIRE, OUTPUT);
  pinMode(PYRO2_FIRE, OUTPUT);
  digitalWrite(PYRO1_FIRE, LOW);
  digitalWrite(PYRO2_FIRE, LOW);
}

void loop(void) {
  checkForCommands();
  updateData();
  updateState();
  executeState();
  calculateComplPID();

  if (separationTriggered && (millis() - separationStartTime >= separationDuration)) {
    digitalWrite(PYRO1_FIRE, LOW);
    digitalWrite(PYRO2_FIRE, LOW);
    separationTriggered = false;
  }

}

// Check for incoming LoRa commands
void checkForCommands() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      buf[len] = '\0';  // Null-terminate buffer
      String receivedCommand = String((char*)buf);
      receivedCommand.trim();

      Serial.print("LoRa received: ");
      Serial.println(receivedCommand); // Debugging

      if (receivedCommand.equals("SEPARATE")) {
        Serial.println("LoRa command SEPARATE received!");
        triggerSeparation();
      } else {
        Serial.println("Received unknown command: " + receivedCommand);
      }
    }
  }
}



// Activate pyros
void triggerSeparation() {
  leds[0] = CRGB::Red;
  leds[1] = CRGB::Red;
  FastLED.show();
  digitalWrite(PYRO1_FIRE, HIGH);
  digitalWrite(PYRO2_FIRE, HIGH);
  separationStartTime = millis();
  separationTriggered = true;
  Serial.println("Separation triggered!");
}

void updateData() { //updates all data
  updateIMU(); //which way are we pointing?
  updateAltimeter(altData); //how high?
  calculateNums(); //do math
  readSerial(); //listen 
  
}

void sendData() {
  char message[64]; // Buffer for data

  // Extract orientation data 
  float yaw = quatAngles[0] *RAD_TO_DEG;  // Yaw from orientation
  float pitch = quatAngles[1] *RAD_TO_DEG; // Pitch from orientation
  float roll = quatAngles[2] *RAD_TO_DEG; // Roll from orientation

  // Altimeter data
  float altitude = altData[0];

  // Servo angles
  int yawServoAngle = yawServo.read();
  int pitchServoAngle = pitchServo.read();

  // Format the data into a string rounding to second deciaml place
  snprintf(message, sizeof(message),
           "YPR: %.2f,%.2f,%.2f; Alt: %.2f; Servo: %d,%d",
           yaw, pitch, roll, altitude, yawServoAngle, pitchServoAngle);

  // Send the message via LoRa
  rf95.send((uint8_t *)message, strlen(message));
  rf95.waitPacketSent();

  // Print the data to Serial for debugging
  // Serial.println("Sent: ");
  // Serial.println(message);
}


void calculateNums() { //will be used to implement control logic. 
  flightNumbers();
  quatDelta1 = (quatAngles[0]-quat_last_angle1)/dt;
  quatDelta2 = (quatAngles[1] - quat_last_angle2)/dt;
  //Serial.println(delta);
  quat_last_angle1 = quatAngles[0];
  quat_last_angle2 = quatAngles[1];
  quatIntegral1 = quatIntegral1+quatAngles[0]*dt;
  quatIntegral2 = quatIntegral2+quatAngles[1]*dt;

  accelAngles[0] = atan(accelData[1]/accelData[0]);
  accelAngles[1] = -atan(accelData[2]/accelData[0]);
  intAngles[2] = intAngles[2]+gyroRates[0]*dt;
  complAngles[0] = 0.98*(complAngles[0] - gyroRates[2]*dt)+0.02*accelAngles[0];
  complAngles[1] = 0.98*(complAngles[1] -gyroRates[1]*dt)+0.02*accelAngles[1];
  complAngles[2] = intAngles[2];

  complIntegral1 = complIntegral1 + complAngles[0]*dt;
  complIntegral2 = complIntegral2 + complAngles[1]*dt;

  accel1 = (gyroRates[0] - last_gyro1)/dt;
  accel2 = (gyroRates[1]-last_gyro2)/dt;
  last_gyro1 = gyroRates[0];
  last_gyro2 = gyroRates[1];


  // Serial.println(accel1*RAD_TO_DEG);
}

void flightNumbers() { //basic global variables
  flightTime = millis() - flightStart; //how long have we been flying?

}

void readSerial () { 
  if (Serial.available()) {
    command = Serial.readString();
    command.trim();
    Serial.print("Received command: ");
    Serial.println(command);  // Debugging: Show received command

    if (command.equals("SEPARATE")) {
      Serial.println("Triggering Separation!");
      triggerSeparation();
    }
  }
}


boolean detectApogee() { //have we apogeed?
  if ((millis()-apogeeLast)>apogeeDt) {//if its time to check apogee, lets check it!
    apogeeLast = millis(); //reset timer for future iterations

    if (previousAltitude>altData[0]+deltaH) { //have we descended below previous altitudes?
      return true; //apogee!
    }

    previousAltitude = altData[0]; //reset altitude to check in the future. 
    return false; //no apogee!

  } else { 
    return false; //no apogee!
  }
}

void updateState(){ //uses sensor data to keep track of state machine
  if (command == "LAUNCH" && currentState == PAD_IDLE){ //PAD_IDLE --> COUNTDOWN, if we send "LAUNCH" to the teensy

    if (millis() - lastTelemetryTime >= telemetryInterval) { // start sending orientation to GSE once launch sequence triggered
    sendData();
    lastTelemetryTime = millis();  // Reset the timer
    }

    currentState++; //get out of this state into the next one
    leds[0] = CRGB::Black;
    leds[1] = CRGB::Black;
    FastLED.show();
    reset();

  }else if (accelData[0] <= -11 && currentState == COUNTDOWN) { //COUNTDOWN --> ASCENT, if we are accelerating
    flightStart = millis(); //get that flight timer going
    currentState++; 

  } else if (flightTime >= thrustTime && currentState == ASCENT){ //changes states from ascent to unpowered after set time
    currentState++;

  } else if ((currentState == UNPOWERED_ASCENT || currentState == ASCENT) && detectApogee()) { //if we're ascending in general lets check for apogee
    currentState++;

  } else if (((abs(quatAngles[0] *RAD_TO_DEG) >= 60)|| (abs(quatAngles[1] * RAD_TO_DEG) >= 60)) && (currentState == ASCENT || currentState == UNPOWERED_ASCENT)) { //ABORT IF WE TILT TOO FAR

    currentState = ABORT;
    
  } 
}

void executeState(){ //runs code based on state of rocket //the LED blinks are placeholders
  if (currentState == PAD_IDLE){ //If at PAD_IDLE, turn off LED

    leds[0] = CRGB::Black;
    leds[1] = CRGB::Black;
    FastLED.show();

  } else if (currentState == COUNTDOWN) { //for countdown, blink LED

    blinkLED(1.5);

  } else if (currentState == ASCENT) { //blink faster for ascent

    blinkLED(.075);

  } else if (currentState == UNPOWERED_ASCENT) {

    blinkLED(.25);
  } else if (currentState == DESCENT ) {

    blinkLED(5);

  } else if (currentState == ABORT){

    abortProcedures();

  }


}

void abortProcedures() {

  blinkLED(1.5);

}

void blinkLED (double duration) { 
  //blinks the onboard LED at a certain duration. Duration is the time it takes for LED to turn on and off
  double currentTime = millis();

  if((currentTime - LEDTimer)/1000 <= duration/2) {
    leds[0] = CRGB::Black;
    leds[1] = CRGB::Black;
    FastLED.show();
  } else if ((currentTime - LEDTimer)/1000 <= duration) {
    leds[0] = CRGB::White;
    leds[1] = CRGB::White;
    FastLED.show();
  } else {
    LEDTimer = millis();
  }

}

void updateIMU() { //gathers IMU information

  sensors_event_t orientationData, angVelocityData, linearAccelData/*,magnetometerData*/, accelerometerData, gravityData; //set variables up to be called
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); //get sensor fusion orientation (affected by magnetic fields)
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE); //get gyro values (seems pretty stable)
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL); //get linear acceleration x, y, z
  // bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER); //magnetic field data
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER); //gravity data? haven't looked into it yet
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);


  //update everything and store in the arrays defined above. 
  returnData(&orientationData, localAngles); 
  returnData(&angVelocityData, gyroRates);
  returnData(&linearAccelData, linearAccelerations);
  returnData(&accelerometerData, accelData);
  returnData(&gravityData, gravData);
  

  //classic dt calculation! Divided by 1 million because units is microseconds
  dt = (micros()-lastTime)/1000000;

  quaternions(gyroRates, dt, quatAngles); //updates quaternion library with gryo rates, the library basically integrates 
  //then updates quaternion position

  lastTime = micros(); //reset our last time!
}


void updateAltimeter(double* alt) { //read altimeter
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  //fill in array with numbers
  alt[0] = bmp.readAltitude(REFPRESSURE_HPA); //altitude based on recorded reference pressure
  alt[1] = bmp.pressure;
  alt[2] = bmp.temperature;
}

void returnData(sensors_event_t* event, double* data) { 
  //broad function that will return whatever data is requested. 
  data[0] = -1000, data[1] = -1000 , data[2] = -1000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    data[0] = event->acceleration.x;
    data[1] = event->acceleration.y;
    data[2] = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    data[0] = event->orientation.x;
    data[1] = event->orientation.y;
    data[2] = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    data[0] = event->magnetic.x;
    data[1] = event->magnetic.y;
    data[2] = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    data[0] = event->gyro.x;
    data[1] = event->gyro.y;
    data[2] = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    data[0] = event->gyro.x;
    data[1] = event->gyro.y;
    data[2] = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    data[0] = event->acceleration.x;
    data[1] = event->acceleration.y;
    data[2] = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    data[0] = event->acceleration.x;
    data[1] = event->acceleration.y;
    data[2] = event->acceleration.z;
  }

}


void quaternions(double gyros[3], double dt, double* quat) { //updates quaternions

  ori.update(-gyros[2], -gyros[1], gyros[0], dt); //to match with coordinates of rocket, x is roll, 
  //z yaw (reversed for some reason idk why, it isnt like that on the diagram), y pitch (reversed)
  oriMeasure = ori.toEuler(); //convert quat to normal people vectors
  quat[0] = oriMeasure.yaw; //populate array
  quat[1] = oriMeasure.pitch;
  quat[2] = oriMeasure.roll;


}


void reset() { //reset in general
  resetSensors();
}
void resetSensors(){ //broad function to reset sensors when needed
  zeroIMU();
  zeroAltimeter();
}

void zeroAltimeter(){ //reset our baseline
  updateAltimeter(altData);
  REFPRESSURE_HPA = altData[1]/100; //reset baseline. 1/100 is conversion

}

void zeroIMU(){ //zeros IMU. May include more things later but for now just zeros quaternion vectors. 
  // accelAngles[0] = atan(accelData[1]/accelData[0]);
  // accelAngles[1] = atan(accelData[2]/accelData[0]);
  // double yaw = accelAngles[0];
  // double pitch = accelAngles[1];
  // double roll = 0;
  // double cr = cos(roll * 0.5);
  // double sr = sin(roll * 0.5);
  // double cp = cos(pitch * 0.5);
  // double sp = sin(pitch * 0.5);
  // double cy = cos(yaw * 0.5);
  // double sy = sin(yaw * 0.5);
  // double qw = cr * cp * cy + sr * sp * sy;
  // double qx = sr * cp * cy - cr * sp * sy;
  // double qy = cr * sp * cy + sr * cp * sy;
  // double qz = cr * cp * sy - sr * sp * cy;
  
  ori.zero();
  // ori.set(qw, qx, qy, qz);
}

void moveYawServo(double angle){
  if(angle>MAX_ANGLE){
    angle = MAX_ANGLE;
  }
  else if(angle<-MAX_ANGLE){
    angle = -MAX_ANGLE;
  }
  double command = (RATIO*angle)-OFFSET1+90;
  //if(math.fabs(command)>5)
  yawServo.write(command);
  //Serial.println(command);
}
void movePitchServo(double angle){
  if(angle>MAX_ANGLE){
    angle = MAX_ANGLE;
  }
  else if(angle<-MAX_ANGLE){
    angle = -MAX_ANGLE;
  }
  double command = (RATIO*angle)-OFFSET2+90;
  //if(math.fabs(command)>5)
  pitchServo.write(command);
  //Serial.println(command);
}
void calculateComplPID(){
  a1Des = ((complAngles[0] - 0)*P + (-gyroRates[2] - 0)*D+complIntegral1*I);
  a2Des = ((complAngles[1] - 0)*P + (-gyroRates[1] - 0)*D+complIntegral2*I); 
  double angle1 = (0.05/(0.05*0.30))*a1Des;
  double angle2 = (0.05/(0.05*0.30))*a2Des;
  // double angle2 = ((quatAngles[1]-0)*P+(delta2-0)*D+integral2*I);
  // Serial.println(-angle1);
  moveYawServo(-angle1);
  movePitchServo(-angle2);
}

void calculateQuatPID(){
  a1Des = ((quatAngles[0] - 0)*P + (quatDelta1 - 0)*D+quatIntegral1*I);
  a2Des = ((quatAngles[1] - 0)*P + (quatDelta2 - 0)*D+quatIntegral2*I); 
  double angle1 = (0.05/(0.05*0.30))*a1Des;
  double angle2 = (0.05/(0.05*0.30))*a2Des;
  // double angle2 = ((quatAngles[1]-0)*P+(delta2-0)*D+integral2*I);
  // Serial.println(-angle1);
  moveYawServo(-angle1);
  movePitchServo(-angle2);
}