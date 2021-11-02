/*
Arduino Pins           Bluetooth Pins
RX (Pin 0)     ———->      TX
TX (Pin 1)      ———->      RX
*/
 
int speed1 = 0;
int speed2 = 0;
 
int inChar = 0;
String inString = "";
 
void setup() {
  Serial.begin(38400);
}
 
void loop() {
 
  while( Serial.available() > 0 ) {
    inChar = Serial.read();
    if( isDigit(inChar) ) {
      inString += (char)inChar;
    }
    if( inChar == '\t' ) {
      speed1 = inString.toInt();
      inString = "";
    }
    if( inChar == '\n' ) {
      speed2 = inString.toInt();
      inString = "";
    }
  }
 
  Serial.print(speed1);
  Serial.print("\t");
  Serial.print(speed2);
  Serial.print("\n");
 
  delay(100);
}
