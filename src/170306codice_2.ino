#include <VirtualWire.h>
#include <Wire.h>
 
uint8_t su;
uint8_t giu;
uint8_t dx;
uint8_t sx;
uint8_t Value1;
uint8_t Value2;
uint8_t Value3;
uint8_t Value4;
uint8_t Value5;
 
const int motdxA = 10;
const int motdxB = 3;
const int motsxA = 5;
const int motsxB = 6;
 
uint8_t buf[9];
uint8_t buflen = 9;
 
uint8_t c = 0;
int i = 0;
 
void setup() {
  vw_set_ptt_inverted(true); // Required by the RF module
  vw_setup(2000); // bps connection speed
  vw_set_rx_pin(11); // Arduino pin to connect the receiver data pin
  vw_rx_start(); // Start the receiver
 
//  Serial.begin(9600);
  Wire.begin();
 
  pinMode(motdxA, OUTPUT);
  pinMode(motdxB, OUTPUT);
  pinMode(motsxA, OUTPUT);
  pinMode(motsxB, OUTPUT);
}
 
void loop() {
  if (vw_get_message(buf, &buflen)){
    su = buf[0];
    giu = buf[1];
    dx = buf[2];
    sx = buf[3];
    Value1 = buf[4];
    Value2 = buf[5];
    Value3 = buf[6];
    Value4 = buf[7];
    Value5 = buf[8];
 
    Wire.beginTransmission(8);
    Wire.write(';');
    for( i = 4; i < 9; i++ ) {
      Wire.write(buf[i]);
    }
    Wire.write('.');
    Wire.endTransmission();
  }
 
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
