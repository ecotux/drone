#include <VirtualWire.h>
#include <Wire.h>
 
// *************** IMU
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
 
// Complementary Filter Data
const double k = 10.0; // MAGIC !!
double tmpCompAngleX = 0.0, tmpCompAngleY = 0.0;
double compAngleX, compAngleY; // Calculated angle using a complementary filter
 
// RAW value
double roll, pitch; // RAW roll & pitch from accelerometer
double gyroXrate, gyroYrate; // RAW gyro rates
double dt; // delta time (in seconds)
 
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
 
const int MPU_addr=0x68;  // I2C address of the MPU-6050
 
unsigned long oldTmpSerial = 0;
unsigned long tmpSerial = 0;
// *************** end IMU
 
// *************** Trasmissione comandi
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
// *************** End Trasmissione comandi
 
void setup() {
// *************** Init RF
  vw_set_ptt_inverted(true); // Required by the RF module
  vw_setup(2000); // bps connection speed
  vw_set_rx_pin(11); // Arduino pin to connect the receiver data pin
  vw_rx_start(); // Start the receiver
// *************** end RF
 
  Serial.begin(9600);
  Wire.begin();
 
// *************** Init IMU
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  accX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
 
  compAngleX = roll;
  compAngleY = pitch;
 
  timer = micros();
// *************** end IMU
 
// Motori DC
  pinMode(motdxA, OUTPUT);
  pinMode(motdxB, OUTPUT);
  pinMode(motsxA, OUTPUT);
  pinMode(motsxB, OUTPUT);
}
 
void loop() {
// *************** receive RF
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
// *************** end RF
 
// *************** start IMU
// Update all RAW values: accX, accY, accZ, tempRaw, gyroX, gyroY, gyroZ;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  accX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tempRaw=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  // RAW values updated
 
  dt = (double)(micros() - timer) / 1000000; // Calculate delta time (in seconds)
  timer = micros();
 
// Suppongo NO "RESTRICT_PITCH"
// ovvero:
// "roll" varia da -90 a +90 gradi: -90 < roll < 90
// "pitch" varia da -180 a +180 gradi: -180 < pitch <= 180
 
// RAW roll & pitch from accelerometer
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
// RAW gyro rates
  gyroXrate = gyroX / 131.0; // Convert to deg/s
  gyroYrate = gyroY / 131.0; // Convert to deg/s
 
// NON ben CAPITO
// proveniente da: https://github.com/TKJElectronics/KalmanFilter/blob/master/examples/MPU6050/MPU6050.ino
// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && compAngleY > 90) || (pitch > 90 && compAngleY < -90)) {
    compAngleY = pitch;
  }
  if (abs(compAngleY) > 90) {
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  }
 
// HERE IS THE FILTER: SECOND-ORDER COMPLEMENTARY FILTER
  // Calculate the parameter for the Complementary filter
  tmpCompAngleX += (roll - compAngleX) * k * k * dt;
  compAngleX += dt * ( gyroXrate + tmpCompAngleX + 2 * k * (roll - compAngleX));
  tmpCompAngleY += (pitch - compAngleY) * k * k * dt;
  compAngleY += dt * ( gyroYrate + tmpCompAngleY + 2 * k * (pitch - compAngleY));
  // END OF FILTER
// *************** end IMU
 
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
 
  tmpSerial = millis();
  if(tmpSerial - oldTmpSerial > 500 ) {
    oldTmpSerial = tmpSerial;
 
    Serial.print("roll: "); Serial.print("\t");
    Serial.print(compAngleX); Serial.print("\t");
 
    Serial.print("\t");
 
    Serial.print("pitch: "); Serial.print("\t");
    Serial.print(compAngleY); Serial.print("\t");
 
    Serial.print("\r\n");
  }
 
}
