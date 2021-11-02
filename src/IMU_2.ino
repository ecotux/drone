#include <Wire.h>
 
// IMU Data
const int MPU_addr = 0x68;    // I2C address of the MPU-6050
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
 
// RAW value
double roll, pitch;            // RAW roll & pitch from accelerometer
double gyroXrate, gyroYrate;   // RAW gyro rates
double temperature = 0.0;
 
// Complementary Filter Data
uint32_t timer;
double dt;                // delta time (in seconds)
const double k = 10.0;    // MAGIC !!
double tmpCompAngleX = 0.0;
double tmpCompAngleY = 0.0;
double compAngleX;        // Calculated angle using a complementary filter
double compAngleY;
 
// Writing to serial
unsigned long oldTmpSerial = 0;
unsigned long tmpSerial = 0;
 
 
void setup() {
 
  Serial.begin(9600);
 
// Initialize IMU
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
 
// Initialize Complementary Filter: compAngleX, compAngleY, timer
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);      // request a total of 14 registers
  accX = Wire.read() << 8 | Wire.read();    // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY = Wire.read() << 8 | Wire.read();    // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ = Wire.read() << 8 | Wire.read();    // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}
 
void loop() {
 
/*******
  RAW values
*******/
// Update all RAW values: accX, accY, accZ, tempRaw, gyroX, gyroY, gyroZ;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  accX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  tempRaw = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  // RAW values updated
//
// Suppongo NO "RESTRICT_PITCH"
// ovvero:
// "roll" varia da -90 a +90 gradi: -90 < roll < 90
// "pitch" varia da -180 a +180 gradi: -180 < pitch <= 180
//
// RAW roll & pitch from accelerometer
  roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
// RAW gyro rates
  gyroXrate = gyroX / 131.0; // Convert to deg/s
  gyroYrate = gyroY / 131.0; // Convert to deg/s
// RAW Temperature
  temperature = (double)tempRaw / 340.0 + 36.53;
 
/*******
  Second order Complementary Filter
*******/
  dt = (double)(micros() - timer) / 1000000; // Calculate delta time (in seconds)
  timer = micros();
// NON ben CAPITO, proveniente da: https://github.com/TKJElectronics/KalmanFilter/blob/master/examples/MPU6050/MPU6050.ino
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
 
/*******
  Print data
*******/
  tmpSerial = millis();
  if(tmpSerial - oldTmpSerial > 100 ) {
    oldTmpSerial = tmpSerial;
 
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");
 
  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");
#endif
 
    Serial.print("roll: "); Serial.print("\t");
    Serial.print(compAngleX); Serial.print("\t");
 
    Serial.print("pitch: "); Serial.print("\t");
    Serial.print(compAngleY); Serial.print("\t");
 
    Serial.print("temperature: "); Serial.print("\t");
    Serial.print(temperature); Serial.print("\t");
 
    Serial.print("\r\n");
  }
}
