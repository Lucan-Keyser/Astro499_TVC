#include <Orientation.h>
#include <Quaternion.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <utility/imumaths.h>

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Adafruit_BMP3XX bmp; //create instance of altimeter
float REFPRESSURE_HPA = 700; //arbitrary sea level pressure in hPA = 100 Pa). Will be reset on start up and before launch


Orientation ori; //create instance of quaternion class
EulerAngles oriMeasure; // same thing but with euler angles

//establish double pointers that will have the accelerometer data stored in them
double* quatAngles = new double[3];
double* gyroRates = new double[3];
double* localAngles = new double[3];
double* linearAccelerations = new double[3];
double* accelData = new double[3];
double* gravData = new double[3];
double* altData = new double[3]; //altitude, pressure (hPa), tempurature

//time tracking information
double dt = 0; //units microseconds
double lastTime = micros(); //units microseconds, used for dt
double flightStart = 0; //to record start time of launch
double flightTime = 0; //how long have we been flying?
double thrustTime = 3000; //in milliseconds that the motor runs for
double apogeeDt = 400; //0.4 seconds between altitude measurements
double deltaH = .5; //differnce in height to consider when we apogee
double apogeeLast = 0; //last time apogee was checked
double previousAltitude = 0; //previous altitude apogee was checked at


/* STATE VARIABLES*/
#define PAD_IDLE 0
#define COUNTDOWN 1
#define ASCENT 2
#define UNPOWERED_ASCENT 3
#define DESCENT 4
#define ABORT -1
int currentState = PAD_IDLE;

//random interface variables
int ledPin = 13;
double LEDTimer = millis(); //units milliseconds
String command = "";

//setup function, runs everything at startup
void setup(void) {
  Serial.begin(9600);
  while(!Serial);
  // Initialise the IMU 
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1); // will get hung up if no IMU detected
  }
  //initialize alt
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1); //will get hung up if alt not detected
  }

  //get first reading. This is always wrong, so we do it here. 
  if (! bmp.performReading()) { //get reading and let us know if we have error
    Serial.println("Failed to perform reading :(");
  } //THIS FIRST READING IS WRONG

  //set sampling rates
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X); //magic things I dont understand for altimeter
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  //LED and reset things
  pinMode(ledPin, OUTPUT);
  reset(); //reset things;
}

void loop(void) { //runs continiously, DO NOT ADD DELAYS IN HERE
  updateData(); //gather data
  updateState(); //checks to see if we've changed states
  executeState(); //acts upon state
  //Serial.println(quatAngles[0]*RAD_TO_DEG);
  //Serial.println(altData[0]);
}

void updateData() { //updates all data
  updateIMU(); //which way are we pointing?
  updateAltimeter(altData); //how high?
  calculateNums(); //do math
  readSerial(); //listen 
  
}

void calculateNums() { //will be used to implement control logic. 
  flightNumbers();
}

void flightNumbers() { //basic global variables
  flightTime = millis() - flightStart; //how long have we been flying?

}

void readSerial () { //listens ans stores variables
  if (Serial.available()) {
    command = Serial.readString();
    command.trim();
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
  if (command == "g" && currentState == PAD_IDLE){ //PAD_IDLE --> COUNTDOWN, if we send some message to the teensy

    currentState++; //get out of this state into the next one
    digitalWrite(ledPin, LOW);  //turn off LED
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
  if (currentState == PAD_IDLE){ //If at PAD_IDLE, turn on LED

    digitalWrite(ledPin, HIGH);

  } else if (currentState == COUNTDOWN) { //for countdown, blink LED

    blinkLED(1);

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
    digitalWrite(ledPin, LOW);
  } else if ((currentTime - LEDTimer)/1000 <= duration) {
    digitalWrite(ledPin, HIGH);
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
  ori.zero();
}