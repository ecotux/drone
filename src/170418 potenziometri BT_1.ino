/*
Arduino Pins           Bluetooth Pins
RX (Pin 0)     ———->      TX
TX (Pin 1)      ———->      RX
*/
 
int potPin1 = A0;
int potPin2 = A1;
int speed1 = 0;
int speed2 = 0;
 
void setup() {
  Serial.begin(38400);
}
 
void loop() {
  speed1 = map( analogRead(potPin1), 0, 1023, 1000, 2000 );
  speed2 = map( analogRead(potPin2), 0, 1023, 1000, 2000 );
 
  Serial.print(speed1);
  Serial.print("\t");
  Serial.print(speed2);
  Serial.print("\n");
 
  delay(100);
}
