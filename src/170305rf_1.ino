#include <VirtualWire.h>
 
int motorspeed = 0;
 
int potPin = A0;
int speed;
 
uint8_t controlArray[1];
 
void setup() {
 
  vw_set_ptt_inverted(true); 
  vw_setup(2000);
  vw_set_tx_pin(2);
 
}
 
void loop() {
 
  motorspeed = analogRead(A0);
  motorspeed = map(motorspeed, 0, 1023, 1000, 2000);
  motorspeed = speed;
 
  controlArray[0] = speed;
 
  vw_send((uint8_t *)controlArray, 1);
  vw_wait_tx();
 
}

