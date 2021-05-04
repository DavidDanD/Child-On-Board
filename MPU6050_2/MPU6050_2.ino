#include<Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int16_t lastAcX, lastAcY, lastAcZ, lastGyX, lastGyY, lastGyZ;
int16_t disAccX, disAccY, disAccZ, disGyroX, disGyroY, disGyroZ;
int squareDistanceAcc, squareDistanceGyro;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22, 100000); // sda, scl
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.println("Setup complete");

}

void loop() {
  delay(250);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  disAccX = lastAcX-AcX;
  disAccY = lastAcY-AcY;
  disAccZ = lastAcZ-AcZ;
  disGyroX = lastGyX-GyX;
  disGyroY = lastGyY-GyY;
  disGyroZ = lastGyZ-GyZ;
  Serial.print("AccX: ");
  Serial.print(disAccX); Serial.print(" , ");
  Serial.print("GyroX: ");
  Serial.println(disGyroX);
  Serial.print("AccY: ");
  Serial.print(disAccY); Serial.print(" , ");
  Serial.print("GyroY: ");
  Serial.println(disGyroY);
  Serial.print("AccZ: ");
  Serial.print(disAccZ); Serial.print(" , ");
  Serial.print("GyroZ: ");
  Serial.println(disGyroZ);

  lastAcX = AcX;
  lastAcY = AcY;
  lastAcZ = AcZ;
  lastGyX = GyX;
  lastGyY = GyY;
  lastGyZ = GyZ;
  squareDistanceAcc = disAccX*disAccX + disAccY*disAccY + disAccZ*disAccZ;
  squareDistanceGyro =  disGyroX*disGyroX + disGyroY*disGyroY + disGyroZ*disGyroZ;
  
  Serial.print("Acc square distance: ");
  Serial.print(squareDistanceAcc); Serial.print(" , ");
  Serial.print("Gyro square distance: ");
  Serial.println(squareDistanceGyro);
  
  

}
