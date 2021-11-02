/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
 
 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
 
 Contact information
 -------------------
 
 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */
 
/* Copyright (C) 2016 Gabriele Biucchi. All rights reserved.
 
 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
 
 Contact information
 -------------------
 
 Gabriele Biucchi, IPC Verri
 Web      :  http://www.ecotux.com
 e-mail   :  mrecotux@gmail.com
 */
 
// scarica
// https://raw.githubusercontent.com/TKJElectronics/KalmanFilter/master/Kalman.cpp
// https://raw.githubusercontent.com/TKJElectronics/KalmanFilter/master/Kalman.h
// in "sketchbook/libraries/Kalman/"
 
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
 
#include <Wire.h>
 
// IMU Data
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
 
// Kalman Filter Data
Kalman kalmanX;
Kalman kalmanY;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
 
// RAW value
double roll, pitch; // RAW roll & pitch from accelerometer
double gyroXrate, gyroYrate; // RAW gyro rates
double dt; // delta time (in seconds)
 
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
 
const int MPU_addr=0x68;  // I2C address of the MPU-6050
 
unsigned long oldTmpSerial = 0;
unsigned long tmpSerial = 0;
double temperature = 0.0;
 
void setup() {
 
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
 
  Serial.begin(9600);
 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  accX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
 
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
 
  timer = micros();
}
 
void loop() {
 
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
 
// NON ben capito
// proveniente da: https://github.com/TKJElectronics/KalmanFilter/blob/master/examples/MPU6050/MPU6050.ino
// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalAngleY = pitch;
  }
  if (abs(kalAngleY) > 90) {
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  }
 
// HERE IS THE FILTER: KALMAN FILTER
  // Calculate the angle using a Kalman filter
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  // END OF FILTER
 
 
// Print Data
 
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");
 
  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");
 
  Serial.print("\t");
#endif
 
#if 1 // Set to 1 to activate
 
tmpSerial = millis();
if(tmpSerial - oldTmpSerial > 500 ) {
  oldTmpSerial = tmpSerial;
 
  Serial.print("roll: "); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");
 
  Serial.print("\t");
 
  Serial.print("pitch: "); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
 
//  Serial.print("\t");
//  temperature = (double)tempRaw / 340.0 + 36.53;
//  Serial.print(temperature); Serial.print("\t");
 
  Serial.print("\r\n");
}
#endif
 
}
