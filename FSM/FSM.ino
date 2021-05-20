#include <Arduino.h>
#include <pthread.h>
#include <Wire.h>
#include "DHT.h"
#include "FSM.h"

// Timer variables
unsigned long lastTime = 0;  
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeSMS = 0;
unsigned long engineIdleTime = 0;
unsigned long engineIdleSMSTime = 0;
unsigned long sampleDelay = 500;
unsigned long SMSDelay = 10*1000;
unsigned long temperatureDelay = 10000;

// Create a sensor object
//Adafruit_MPU6050 mpu;
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
int16_t disAccX, disAccY, disAccZ, disGyroX, disGyroY, disGyroZ;
int squareDistanceAcc, squareDistanceGyro;
//sensors_event_t a, g, temp;

int16_t gyroX, gyroY, gyroZ;
int16_t accX, accY, accZ;
int16_t lastGyroX, lastGyroY, lastGyroZ;
int16_t lastAccX, lastAccY, lastAccZ;
int16_t mpuTemperature;
int16_t temperature;

//Gyroscope sensor deviation
float gyroXThreshold = 0.4;
float gyroYThreshold = 0.4;
float gyroZThreshold = 0.4;
float accXThreshold = 0.4;
float accYThreshold = 0.4;
float accZThreshold = 0.4;
int accThreshold = 100000;
int gyroThreshold = 5000;

//FSR
int fsrValue = 0;
int fsrThreshold = 500;

//Buzzer
#define SOFT_ALARM_DELAY 5*1000
int freq = 0;
int channel = 0;
int resolution = 8;
pthread_t buzzerThread;
int* returnValue;
bool stopBuzzer = true;
int buzzerParams[] = {0,0,0};

//DHT
float h,t,hic;
DHT dht(DHT_PIN, DHT_TYPE);

//SIM
char *msg;
char ch = 176;

enum State_enum {RESET, START, SOFT_ALARM, ALARM, SMS, ENGINE_IDLE};
 
void state_machine_run();
 
uint8_t state = RESET;

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = "";


#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif
 
void sendMsg(String smsMessage){
  // To send an SMS, call modem.sendSMS(SMS_TARGET, smsMessage)
//  String smsMessage = "Hello from ESP32!";
  if(modem.sendSMS(SMS_TARGET, smsMessage)){
    SerialMon.println(smsMessage);
  }
  else{
    SerialMon.println("SMS failed to send");
  }
  lastTimeSMS = millis();
}

void *alarmThread(void* args){

  while(!stopBuzzer){
    ledcWriteTone(channel, buzzerParams[0]);
    delay(buzzerParams[2]);
    ledcWriteTone(channel, buzzerParams[1]);
    delay(buzzerParams[2]);
  }
  ledcWriteTone(channel, 0);
  pthread_exit(NULL);
  
}

void softAlarm(){
  buzzerParams[0] = 500;
  buzzerParams[1] = 0;
  buzzerParams[2] = 1000;
  stopBuzzer = false;
  pthread_create(&buzzerThread, NULL, alarmThread, NULL);
}

void Alarm(){
  if(!stopBuzzer){
    return;
  }
  buzzerParams[0] = 2000;
  buzzerParams[1] = 500;
  buzzerParams[2] = 300;
  stopBuzzer = false;
  pthread_create(&buzzerThread, NULL, alarmThread, NULL);
}


void calculateSquareDistance() {
  
  disAccX = lastAccX-accX;
  disAccY = lastAccY-accY;
  disAccZ = lastAccZ-accZ;
  disGyroX = lastGyroX-gyroX;
  disGyroY = lastGyroY-gyroY;
  disGyroZ = lastGyroZ-gyroZ;

  squareDistanceAcc = disAccX*disAccX + disAccY*disAccY + disAccZ*disAccZ;
  squareDistanceGyro =  disGyroX*disGyroX + disGyroY*disGyroY + disGyroZ*disGyroZ;
}

void calculateHeatIndex() {
  // Check if temperature read failed.
  
  if (isnan(t) || isnan(h)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  
  // Compute heat index in Celsius (isFahreheit = false)
  hic = dht.computeHeatIndex(t, h, false);
}

void getreadings(){
  
  lastAccX = accX;
  lastAccY = accY;
  lastAccZ = accZ;
  
  lastGyroX = gyroX;
  lastGyroY = gyroY;
  lastGyroZ = gyroZ;
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true); // request a total of 14 registers
  
  accX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  gyroX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  gyroY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  gyroZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  mpuTemperature = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  
  // Read humidity
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();

  calculateHeatIndex();
  calculateSquareDistance();
}

void setup(){
  Serial.begin(115200);
//  initMPU();
  Wire.begin(21, 22, 100000); // sda, scl
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.println("Setup complete");
//  init DHT
  dht.begin();
//  pinMode(BUZZER_PIN,OUTPUT);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(BUZZER_PIN, channel);
  
  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // use modem.init() if you don't need the complete restart

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }
}
 
void loop(){
  state_machine_run();
 
  delay(500);
}
 
void state_machine_run() 
{
  fsrValue = analogRead(FSR_PIN);
//  Serial.print("Fsr value: ");
//  Serial.println(fsrValue);
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
      Serial.print("temp: ");
      Serial.println(hic);
      Serial.print("square acc: ");
      Serial.println(squareDistanceAcc);
      Serial.print("square gyro: ");
      Serial.println(squareDistanceGyro);
      if(squareDistanceAcc<accThreshold){
        if(squareDistanceGyro<gyroThreshold){
          engineIdleTime = millis();
          state = ENGINE_IDLE;
          break;
        }
      }
      if(hic >= hicThreshold){
        asprintf(&msg, "High temperature: %.1f degrees Celsuis.", hic);
        sendMsg(msg);
        state = SOFT_ALARM;
      }
      break;

    case ENGINE_IDLE:
      Serial.println("ENGINE_IDLE");
      if(fsrValue < weightThreshold){
        state = RESET;
      }
      getreadings();
      Serial.print("temp: ");
      Serial.println(hic);
      Serial.print("square acc: ");
      Serial.println(squareDistanceAcc);
      Serial.print("square gyro: ");
      Serial.println(squareDistanceGyro);

      
//      if(abs(lastAccX-accX)>accXThreshold || abs(lastAccY-accY)>accYThreshold || abs(lastAccZ-accZ)>accZThreshold){
      if(squareDistanceAcc>accThreshold){
        state = START;
        break;
      }
//      if(abs(lastGyroX-gyroX)>gyroXThreshold || abs(lastGyroY-gyroY)>gyroYThreshold || abs(lastGyroZ-gyroZ)>gyroZThreshold){
      if(squareDistanceGyro>gyroThreshold){
        state = START;
        break;
      }
      if(hic >= hicThreshold){
        asprintf(&msg, "High temperature: %.1f degrees Celsuis.", hic);
        sendMsg(msg);
        state = ALARM;
      }
      if((millis()-engineIdleTime) > IdleThresholdSMS){
        if((millis()-engineIdleSMSTime) > IdleSMSDelay){
          sendMsg("Child on board");
          engineIdleSMSTime = millis();
        }
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
      delay(SOFT_ALARM_DELAY);
      stopBuzzer = true;
      pthread_join(buzzerThread, (void**)(&returnValue));
      break;
 
    case ALARM:
      Serial.println("ALARM");
      Alarm();
      if(fsrValue < weightThreshold){
        state = RESET;
        stopBuzzer = true;
        pthread_join(buzzerThread, (void**)(&returnValue));
        break;
      }
      getreadings();
//      if(abs(lastAccX-accX)>accXThreshold || abs(lastAccY-accY)>accYThreshold || abs(lastAccZ-accZ)>accZThreshold){
      if(squareDistanceAcc>accThreshold){
        state = START;
        stopBuzzer = true;
        pthread_join(buzzerThread, (void**)(&returnValue));
        break;
      }
//      if(abs(lastGyroX-gyroX)>gyroXThreshold || abs(lastGyroY-gyroY)>gyroYThreshold || abs(lastGyroZ-gyroZ)>gyroZThreshold){
      if(squareDistanceGyro>gyroThreshold){
        state = START;
        stopBuzzer = true;
        pthread_join(buzzerThread, (void**)(&returnValue));
        break;
      }
      if((millis() - lastTimeSMS) > SMSDelay){
        sendMsg("Child on board");
      }
      break;

  }
}
