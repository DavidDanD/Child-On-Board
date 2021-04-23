#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <pthread.h>
#include "DHT.h"

//PINS
#define FSR_PIN 34
#define DHT_PIN 15
#define BUZZER_PIN 25
#define DHT_TYPE DHT22

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
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float lastGyroX, lastGyroY, lastGyroZ;
float lastAccX, lastAccY, lastAccZ;
float temperature;

//Gyroscope sensor deviation
float gyroXThreshold = 0.4;
float gyroYThreshold = 0.4;
float gyroZThreshold = 0.4;
float accXThreshold = 0.4;
float accYThreshold = 0.4;
float accZThreshold = 0.4;

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

//SIM
char *msg;
char ch = 176;

//Thresholds
#define weightThreshold 500
#define hicThreshold 26
#define IdleThresholdSMS 5*1000
#define IdleSMSDelay 5*1000
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

enum State_enum {RESET, START, SOFT_ALARM, ALARM, SMS, ENGINE_IDLE};
 
void state_machine_run();
 
uint8_t state = RESET;

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = "";

// Your phone number to send SMS: + (plus sign) and country code, for Israel +972, followed by phone number
#define SMS_TARGET  "+972546591910"

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

#include <TinyGsmClient.h>

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT  Serial1

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

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
  initMPU();
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
      if(abs(lastAccX-accX)<accXThreshold && abs(lastAccY-accY)<accYThreshold && abs(lastAccZ-accZ)<accZThreshold){
        if(abs(lastGyroX-gyroX)<gyroXThreshold && abs(lastGyroY-gyroY)<gyroYThreshold && abs(lastGyroZ-gyroZ)<gyroZThreshold){
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
//      Serial.print("lastAccX-accX: ");
//      Serial.println(abs(lastAccX-accX));
//      Serial.print("lastAccY-accY: ");
//      Serial.println(abs(lastAccY-accY));
//      Serial.print("lastAccZ-accZ: ");
//      Serial.println(abs(lastAccZ-accZ));
//      Serial.print("lastGyroX-gyroX: ");
//      Serial.println(abs(lastGyroX-gyroX));
//      Serial.print("lastGyroY-gyroY: ");
//      Serial.println(abs(lastGyroY-gyroY));
//      Serial.print("lastGyroZ-gyroZ: ");
//      Serial.println(abs(lastGyroZ-gyroZ));
//      Serial.print("temp: ");
//      Serial.println(hic);
      if(abs(lastAccX-accX)>accXThreshold || abs(lastAccY-accY)>accYThreshold || abs(lastAccZ-accZ)>accZThreshold){
        state = START;
        break;
      }
      if(abs(lastGyroX-gyroX)>gyroXThreshold || abs(lastGyroY-gyroY)>gyroYThreshold || abs(lastGyroZ-gyroZ)>gyroZThreshold){
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
      if(abs(lastAccX-accX)>accXThreshold || abs(lastAccY-accY)>accYThreshold || abs(lastAccZ-accZ)>accZThreshold){
        state = START;
        stopBuzzer = true;
        pthread_join(buzzerThread, (void**)(&returnValue));
        break;
      }
      if(abs(lastGyroX-gyroX)>gyroXThreshold || abs(lastGyroY-gyroY)>gyroYThreshold || abs(lastGyroZ-gyroZ)>gyroZThreshold){
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
