#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


// Create a sensor object
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float lastGyroX, lastGyroY, lastGyroZ;
float lastAccX, lastAccY, lastAccZ;
float temperature;

//Gyroscope sensor deviation
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

// Init MPU6050
void initMPU(){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void getreadings(){
  mpu.getEvent(&a, &g, &temp);

  lastAccX = accX;
  lastAccY = accY;
  lastAccZ = accZ;
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  
  lastGyroX = gyroX;
  lastGyroY = gyroY;
  lastGyroZ = gyroZ;
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
}

void setup(){
  Serial.begin(115200);
  initMPU();
}
 
void loop(){
  getreadings();
  Serial.print("lastAccX-accX: ");
  Serial.println(abs(lastAccX-accX));
  Serial.print("lastAccY-accY: ");
  Serial.println(abs(lastAccY-accY));
  Serial.print("lastAccZ-accZ: ");
  Serial.println(abs(lastAccZ-accZ));
  Serial.print("lastGyroX-gyroX: ");
  Serial.println(abs(lastGyroX-gyroX));
  Serial.print("lastGyroY-gyroY: ");
  Serial.println(abs(lastGyroY-gyroY));
  Serial.print("lastGyroZ-gyroZ: ");
  Serial.println(abs(lastGyroZ-gyroZ));
  delay(250);
}
