/*
Arduino Pins           Bluetooth Pins
RX (Pin 0)     ———>      TX
TX (Pin 1)     ———>      RX
*/
 
int potPin1 = A0;
int potPin2 = A1;
int speed1 = 0;
int speed2 = 0;
 
unsigned int nrPacket = 0;
int inChar = 0;
String inString = "s";
int receiving = 0;
 
void setup() {
  Serial.begin(38400);
}
 
void loop() {
  nrPacket = nrPacket + 1;
 
  speed1 = map( analogRead(potPin1), 0, 1023, 1000, 2000 );
  speed2 = map( analogRead(potPin2), 0, 1023, 1000, 2000 );
 
  Serial.print("a");
  Serial.print(nrPacket);
  Serial.print(",");
  Serial.print(speed1);
  Serial.print("\t");
  Serial.print(speed2);
  Serial.print("\n");
 
  while( Serial.available() <= 0 ) {
    Serial.println("initMaster");
  }
 
  while( Serial.available() > 0 ) {
    inChar = Serial.read();
    if( receiving == 0 ) {
      if( char(inChar) == 'r' ) {
        receiving = 1;
      }
    } else {
      if( inChar == '\n' ) {
        receiving = 0;
        break;
      } else {
        inString += (char)inChar;
      }
    }
  }
  Serial.println(inString);
  inString="s";
 
  delay(100);
}
