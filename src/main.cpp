#include <Arduino.h>
#include <Servo.h>


Servo servos[4];


int servoPins[] = { 3, 5, 6, 9 };


int potPins[] = { A0, A1, A2, A3 };

int jointCount = 4;

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < jointCount; i++) {
    servos[i].attach(servoPins[i]);
  }
}

void loop() {
  for (int i = 0; i < jointCount; i++) {
    
    int potValue = analogRead(potPins[i]);

    
    int angle = map(potValue, 0, 1023, 0, 180);

    
    servos[i].write(angle);

    
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(angle);
    Serial.print("  ");
  }
  Serial.println();

  delay(15);
}