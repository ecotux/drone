/*
Arduino Pins           Bluetooth Pins
RX (Pin 0)     ———>      TX
TX (Pin 1)     ———>      RX
*/
 
int speed1 = 0;
int speed2 = 0;
 
unsigned int nrPacket = 0;
int inChar = 0;
String inString = "";
int receiving = 0;
 
void setup() {
  Serial.begin(38400);
}
 
void loop() {
 
  while( Serial.available() > 0 ) {
    inChar = Serial.read();
    if( receiving == 0 ) {
      if( char(inChar) == 'a' ) {
        receiving = 1;
      }
    } else {
      if( isDigit(inChar) ) {
        inString += (char)inChar;
      }
      if( char(inChar) == ',' ) {
        nrPacket = inString.toInt();
        inString = "";
      }
      if( inChar == '\t' ) {
        speed1 = inString.toInt();
        inString = "";
      }
      if( inChar == '\n' ) {
        speed2 = inString.toInt();
        inString = "";
        receiving = 0;
      }
    }
  }
 
  Serial.print("r,");
  Serial.print(nrPacket);
  Serial.print(",");
  Serial.print(speed1);
  Serial.print(",");
  Serial.print(speed2);
  Serial.print("\n");
 
  delay(100);
}
