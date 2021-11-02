#include <Servo.h>
 
int potPin1 = A0;
int potPin2 = A1;
int brushlessPin1 = 9;
int brushlessPin2 = 10;
 
Servo esc1;
Servo esc2;
 
int speed1 = 0;
int speed2 = 0;
 
void setup() {
  Serial.begin(9600);
 
  esc1.attach(brushlessPin1);
  esc2.attach(brushlessPin2);
}
 
void loop() {
  speed1 = map( analogRead(potPin1), 0, 1023, 1000, 2000 );
  speed2 = map( analogRead(potPin2), 0, 1023, 1000, 2000 );
 
  esc1.writeMicroseconds(speed1);
  esc2.writeMicroseconds(speed2);
 
  Serial.print(speed1);
  Serial.print("\t");
  Serial.println(speed2);
 
  delay(100);
}
