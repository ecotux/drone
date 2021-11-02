#include <VirtualWire.h>
#include <SoftwareServo.h>
 
SoftwareServo esc1;
SoftwareServo esc2;
SoftwareServo esc3;
SoftwareServo esc4;
 
int brushless1Pin = 11;
int brushless2Pin = 10;
int brushless3Pin = 9;
int brushless4Pin = 6;
int potPin = A0;
 
int speed = 0;
 
void setup() {
 
   esc1.attach( brushless1Pin );
   esc2.attach( brushless2Pin );
   esc3.attach( brushless3Pin );
   esc4.attach( brushless4Pin );
 
  vw_set_ptt_inverted(true); 
  vw_setup(2000);
  vw_set_rx_pin(3); 
  vw_rx_start();
 
}
 
void loop() {
  SoftwareServo::refresh();
 
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
 
  if (vw_get_message(buf, &buflen)) {
    speed = buf[0];
  }
 
  esc1.write( speed );
  esc2.write( speed );
  esc3.write( speed );
  esc4.write( speed );
 
}
