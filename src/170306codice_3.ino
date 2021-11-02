#include <Wire.h>
#include <Servo.h>
 
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
 
int servoPin1 = 13;
int servoPin2 = 12;
int servoPin3 = 11;
int servoPin4 = 10;
int servoPin5 = 9;
 
uint8_t Value1 = 0;
uint8_t Value2 = 0;
uint8_t Value3 = 0;
uint8_t Value4 = 0;
uint8_t Value5 = 0;
 
uint8_t c = 0;
 
void setup() {
//  Serial.begin(9600);
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
 
  myservo1.attach(servoPin1);
  myservo2.attach(servoPin2);
  myservo3.attach(servoPin3);
  myservo4.attach(servoPin4);
  myservo5.attach(servoPin5);
}
 
void loop() {
}
 
void receiveEvent(int howMany) {
 
  while(c != 59) { if (Wire.available() >= 1) { c = Wire.read(); } }
  while(Wire.available() < 1){ }
  if (Wire.available() >= 1) { Value1 = Wire.read(); }
  while(Wire.available() < 1){ }
  if (Wire.available() >= 1) { Value2 = Wire.read(); }
  while(Wire.available() < 1){ }
  if (Wire.available() >= 1) { Value3 = Wire.read(); }
  while(Wire.available() < 1){ }
  if (Wire.available() >= 1) { Value4 = Wire.read(); }
  while(Wire.available() < 1){ }
  if (Wire.available() >= 1) { Value5 = Wire.read(); }
  while(c != 46) { if (Wire.available() >= 1) { c = Wire.read(); } }
 
  myservo1.write(Value1);
  myservo2.write(Value2);
  myservo3.write(Value3);
  myservo4.write(Value4);
  myservo5.write(Value5);
 
/*
  Serial.print(Value1);
  Serial.print("\t");
  Serial.print(Value2);
  Serial.print("\t");
  Serial.print(Value3);
  Serial.print("\t");
  Serial.print(Value4);
  Serial.print("\t");
  Serial.println(Value5);
*/
 
}
