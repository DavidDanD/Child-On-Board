/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-mpu-6050-web-server/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "SPIFFS.h"

// Timer variables
unsigned long lastTime = 0;  
unsigned long lastTimeTemperature = 0;
unsigned long sampleDelay = 500;
unsigned long temperatureDelay = 10000;

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

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
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
  
//  float gyroX_temp = g.gyro.x;
//  if(abs(gyroX_temp) > gyroXerror)  {
//    gyroX += gyroX_temp;//50.00;
//  }
//  
//  float gyroY_temp = g.gyro.y;
//  if(abs(gyroY_temp) > gyroYerror) {
//    gyroY += gyroY_temp;//70.00;
//  }
//
//  float gyroZ_temp = g.gyro.z;
//  if(abs(gyroZ_temp) > gyroZerror) {
//    gyroZ += gyroZ_temp;//90.00;
//  }
}

String getTemperature(){
  mpu.getEvent(&a, &g, &temp);
  temperature = temp.temperature;
  return String(temperature);
}

void setup() {
  Serial.begin(115200);
  initSPIFFS();
  initMPU();
}

void loop() {
  if ((millis() - lastTime) > sampleDelay) {
    // Send Events to the Web Server with the Sensor Readings
    getreadings();

    Serial.println(abs(gyroX)); 
    if (abs(lastGyroX-gyroX)>1 || abs(lastGyroY-gyroY)>1 || abs(lastGyroZ-gyroZ)>1) {
      Serial.println("Gyroscope: ");
      Serial.println(abs(lastGyroX-gyroX));
      Serial.println(abs(lastGyroY-gyroY));
      Serial.println(abs(lastGyroZ-gyroZ));
    }

    Serial.println(abs(accX)); 
    if (abs(lastAccX-accX)>1 || abs(lastAccY-accY)>1 || abs(lastAccZ-accZ)>1) {
      Serial.println("Acceleration: ");
      Serial.println(abs(lastAccX-accX));
      Serial.println(abs(lastAccY-accY));
      Serial.println(abs(lastAccZ-accZ));
    }
    lastTime = millis();
  }
  if ((millis() - lastTimeTemperature) > temperatureDelay) {
    // Send Events to the Web Server with the Sensor Readings
    Serial.print("Temperature: ");
    Serial.println(getTemperature().c_str());
    lastTimeTemperature = millis();
  }
}
