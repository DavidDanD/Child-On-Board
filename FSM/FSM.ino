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
unsigned long lastTimeSMS = 0;
unsigned long engineIdleTime = 0;
unsigned long sampleDelay = 500;
unsigned long SMSDelay = 10*1000;
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

//DHT
float h,t,hic;

//SIM
String msg;


//Thresholds
#define weightThreshold 500
#define hicThreshold 24.5
#define IdleThresholdSMS 5*1000
#define IdleThresholdBuzz 10*1000
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


enum State_enum {RESET, START, SOFT_ALARM, ALARM, SMS, ENGINE_IDLE};
 
void state_machine_run();
void motors_stop();
void motors_forward();
void motors_right();
void motors_left();
uint8_t read_IR();
 
uint8_t state = RESET;

 
void sendMsg(String msg){
  Serial.print("msg sent: ");
  Serial.println(msg);
  lastTimeSMS = millis();
}

void softAlarm(){
  Serial.println("soft alarm");
}

void Alarm(){
  Serial.println("alarm");
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
  
  // Read humidity
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
  // Check if temperature read failed.
  
  if (isnan(t) || isnan(h)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  
  // Compute heat index in Celsius (isFahreheit = false)
  hic = dht.computeHeatIndex(t, h, false);
}

void setup(){
  Serial.begin(115200);
  initSPIFFS();
  initMPU();
  dht.begin();
//  pinMode(BUZZER_PIN,OUTPUT);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(BUZZER_PIN, channel);
}
 
void loop(){
  state_machine_run();
 
  delay(500);
}
 
void state_machine_run() 
{
  fsrValue = analogRead(FSR_PIN);
  switch(state)
  {
    case RESET:
      Serial.println("RESET");
      if(fsrValue >= weightThreshold){
        state = START;
      }
      break;
       
    case START:
      Serial.println("START");
      if(fsrValue < weightThreshold){
        state = RESET;
        break;
      }
      getreadings();
      if(abs(lastAccX-accX)<1 && abs(lastAccY-accY)<1 && abs(lastAccZ-accZ)<1){
        if(abs(lastGyroX-gyroX)<1 && abs(lastGyroY-gyroY)<1 && abs(lastGyroZ-gyroZ)<1){
          engineIdleTime = millis();
          state = ENGINE_IDLE;
          break;
        }
      }
      if(hic >= hicThreshold){
        sendMsg("High temprature");
        state = SOFT_ALARM;
      }
      break;

    case ENGINE_IDLE:
      Serial.println("ENGINE_IDLE");
      if(fsrValue < weightThreshold){
        state = RESET;
      }
      getreadings();
      if(abs(lastAccX-accX)>1 || abs(lastAccY-accY)>1 || abs(lastAccZ-accZ)>1){
        state = START;
        break;
      }
      if(abs(lastGyroX-gyroX)>1 || abs(lastGyroY-gyroY)>1 || abs(lastGyroZ-gyroZ)>1){
        state = START;
        break;
      }
      if(hic >= hicThreshold){
        sendMsg("High temprature");
        state = ALARM;
      }
      if((millis()-engineIdleTime) > IdleThresholdSMS){
        sendMsg("Child on board");
      }
      if((millis()-engineIdleTime) > IdleThresholdBuzz){
        sendMsg("Child on board");
        state = ALARM;
      }
      break;
      
    case SOFT_ALARM:
      Serial.println("SOFT_ALARM");
      softAlarm();
      if(fsrValue < weightThreshold){
        state = RESET;
      }
      else{
        state = START;
      }
      break;
 
    case ALARM:
      Serial.println("ALARM");
      Alarm();
      if(fsrValue < weightThreshold){
        state = RESET;
        break;
      }
      getreadings();
      if(abs(lastAccX-accX)>1 || abs(lastAccY-accY)>1 || abs(lastAccZ-accZ)>1){
        state = START;
        break;
      }
      if(abs(lastGyroX-gyroX)>1 || abs(lastGyroY-gyroY)>1 || abs(lastGyroZ-gyroZ)>1){
        state = START;
        break;
      }
      if((millis() - lastTimeSMS) > SMSDelay){
        sendMsg("Child on board");
      }
      break;

  }
}
