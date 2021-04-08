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
#include "DHT.h"

//PINS
#define FSR_PIN 34
#define DHT_PIN 4
#define BUZZER_PIN 25
#define DHT_TYPE DHT22

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

//FSR
int fsrValue = 0;
int fsrThreshold = 500;

//Buzzer
int freq = 0;
int channel = 0;
int resolution = 8;

DHT dht(DHT_PIN, DHT_TYPE);

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

  if ((millis() - lastTimeTemperature) > temperatureDelay) {
//    Serial.print("Temperature: ");
//    Serial.println(getTemperature().c_str());
    lastTimeTemperature = millis();
  }
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
  dht.begin();
//  pinMode(BUZZER_PIN,OUTPUT);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(BUZZER_PIN, channel);
}

void loop() {
  if ((millis() - lastTime) > sampleDelay) {
    // Send Events to the Web Server with the Sensor Readings
    getreadings();
    fsrValue = analogRead(FSR_PIN);
    if (fsrValue>fsrThreshold) {
      Serial.println(fsrValue);
    }
//    Serial.println(abs(gyroX));
    if (abs(lastGyroX-gyroX)>1 || abs(lastGyroY-gyroY)>1 || abs(lastGyroZ-gyroZ)>1) {
      Serial.println("Gyroscope: ");
      Serial.println(abs(lastGyroX-gyroX));
      Serial.println(abs(lastGyroY-gyroY));
      Serial.println(abs(lastGyroZ-gyroZ));
    }

//    Serial.println(abs(accX)); 
    if (abs(lastAccX-accX)>1 || abs(lastAccY-accY)>1 || abs(lastAccZ-accZ)>1) {
      Serial.println("Acceleration: ");
      Serial.println(abs(lastAccX-accX));
      Serial.println(abs(lastAccY-accY));
      Serial.println(abs(lastAccZ-accZ));
    }
    lastTime = millis();
    
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
  
    // Check if temperature read failed.
    if (isnan(t) || isnan(h)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(t, h, false);
    Serial.print("Current temperature: ");
    Serial.println(t);
    Serial.print("Current humidity: ");
    Serial.println(h);
    Serial.print("Current heat index: ");
    Serial.println(hic);
    
    for(int i=0; i<5; i++){
//      tone(BUZZER_PIN, 500); // tone() is the main function to use with a buzzer, it takes 2 or 3 parameteres (buzzer pin, sound frequency, duration)
      ledcWriteTone(channel, 500);
      delay(100);
//      tone(BUZZER_PIN, 2000); // You can also use noTone() to stop the sound it takes 1 parametere which is the buzzer pin
      ledcWriteTone(channel, 2000);
      delay(100);
    }
//    noTone(BUZZER_PIN);
    ledcWriteTone(channel, 0);
    delay(1000);
  }
}
