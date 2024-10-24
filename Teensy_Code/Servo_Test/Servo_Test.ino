// SERVO TEST!: https://docs.arduino.cc/learn/electronics/servo-motors/

//[LMK] servo variables
#include <PWMServo.h>
PWMServo myservo;
int pos = 0;

void setup() {
  // put your setup code here, to run once:
  
  //[LMK] attach servo
  myservo.attach(9, 1000, 200);

}

void loop() {
  // put your main code here, to run repeatedly:
  // for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15ms for the servo to reach the position
  // }

  myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //myservo.detach(9);
  delay(15); 

}

