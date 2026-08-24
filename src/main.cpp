#include <Arduino.h>
#include <Servo.h>


Servo servos[4];
int servoPins[]= {2,4,6,5};
int servoAngles[]={90,90,90,90};
int servoCount=4;

void setup() {
  for(int i=0; i<servoCount; i++) {
    servos[i].attach(servoPins[i]);    //her servoyu ilgili pine bağla
    servos[i].write(servoAngles[i]);

  }
  
}

void loop() {
  for (int i = 0; i < servoCount; i++) {
    servos[i].write(45);
  }
  delay(1000);

  for (int i = 0; i < servoCount; i++) {
    servos[i].write(135);
  }
  delay(1000);

}