#include <VirtualWire.h>
 
int bsu  = 4;
int bgiu = 8;
int bdx  = 13;
int bsx  = 2;
uint8_t pot1 = 0;
uint8_t pot2 = 1;
uint8_t pot3 = 2;
uint8_t pot4 = 3;
uint8_t pot5 = 4;
 
uint8_t su   = 0;
uint8_t giu  = 0;
uint8_t dx   = 0;
uint8_t sx   = 0;
uint8_t Value1 = 0;
uint8_t Value2 = 0;
uint8_t Value3 = 0;
uint8_t Value4 = 0;
uint8_t Value5 = 0;
 
uint8_t tras[9];
 
void setup () {
 
    Serial.begin(9600);
 
    vw_set_ptt_inverted(true);  // Required by the RF module
    vw_setup(2000);             // bps connection speed
    vw_set_tx_pin(12);           // Arduino pin to connect the transmit data pin
 
    pinMode(bsu,  INPUT);
    pinMode(bgiu, INPUT);
    pinMode(bdx,  INPUT);
    pinMode(bsx,  INPUT);
}
 
void loop() {
   su  = digitalRead(bsu);
   giu = digitalRead(bgiu);
   dx  = digitalRead(bdx);
   sx  = digitalRead(bsx);
 
   pot1 = analogRead(0);
   pot2 = analogRead(1);
   pot3 = analogRead(2);
   pot4 = analogRead(3);
   pot5 = analogRead(4);
 
//   Value1 = map(pot1, 0, 255, 0, 179);
//   Value2 = map(pot2, 0, 255, 0, 179);
//   Value3 = map(pot3, 0, 255, 0, 179);
//   Value4 = map(pot4, 0, 255, 0, 179);
//   Value5 = map(pot5, 0, 255, 0, 179);
 
   Value1 = map(pot1, 0, 255, 0, 179);
   Value2 = map(pot2, 0, 255, 60, 120);
   Value3 = map(pot3, 0, 255, 0, 179);
   Value4 = map(pot4, 0, 255, 0, 179);
   Value5 = map(pot5, 0, 255, 14, 134);
 
  Serial.print(Value1);
  Serial.print("\t");
  Serial.print(Value2);
  Serial.print("\t");
  Serial.print(Value3);
  Serial.print("\t");
  Serial.print(Value4);
  Serial.print("\t");
  Serial.println(Value5);
 
//   Serial.println(su);
//   Serial.println(giu);
//   Serial.println(dx);
//   Serial.println(sx);
//   Serial.println();
 
   tras[0] = su;
   tras[1] = giu;
   tras[2] = dx;
   tras[3] = sx;
   tras[4] = Value1;
   tras[5] = Value2;
   tras[6] = Value3;
   tras[7] = Value4;
   tras[8] = Value5;
   vw_send((uint8_t *)tras, 9);
   vw_wait_tx();
 
}
