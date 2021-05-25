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
int16_t disAccX, disAccY, disAccZ, disGyroX, disGyroY, disGyroZ;
int squareDistanceAcc, squareDistanceGyro;

int16_t gyroX, gyroY, gyroZ;
int16_t accX, accY, accZ;
int16_t lastGyroX, lastGyroY, lastGyroZ;
int16_t lastAccX, lastAccY, lastAccZ;
int16_t mpuTemperature;
int16_t temperature;

//Gyroscope sensor deviation
int accThreshold = 1000000;
int gyroThreshold = 50000;

//FSR
int fsrValue = 0;
int fsrThreshold = 500;

//Buzzer
int freq = 0;
int channel = 0;
int resolution = 8;
pthread_t buzzerThread;
int* returnValue;
bool stopBuzzer = true;
int buzzerParams[] = {0,0,0};

//DHT
int hicMinThreshold = 30;
int hicMaxThreshold = 35;
float h,t,hic;
DHT dht(DHT_PIN, DHT_TYPE);

//SIM
char *msg;
char ch = 176;
TinyGsmClient client(modem);
const char apn[]      = "uinternet"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = ""; // GPRS User
const char gprsPass[] = ""; // GPRS Password
const char simPIN[]   = "";

//WIFI
const char* ssid     = "David";
const char* password = "id29072018";

//SERVER
const char* serverName = "http://cdr.mcr.mybluehost.me/post-data.php";
String apiKeyValue = "tPmAT5Ab3j7F9";
const char server[] = "cdr.mcr.mybluehost.me"; // domain name: example.com, maker.ifttt.com, etc
const char resource[] = "/post-data.php";         // resource path, for example: /post-data.php
const int  port = 80;                             // server port number


enum State_enum {RESET, START, SOFT_ALARM, ALARM, SMS, ENGINE_OFF};
 
uint8_t state = RESET;


void setup(){
  Serial.begin(115200);
//  initMPU();
  Wire.begin(SDA_PIN, SCL_PIN, MPU6050_FREQ); // sda, scl, clock speed
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
//  init DHT
  dht.begin();
//  pinMode(BUZZER_PIN,OUTPUT);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(BUZZER_PIN, channel);
//  init WIFI
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  
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
  Serial.println("Setup complete");
}
 
void loop(){
  state_machine_run();
//  publish_data_to_cloud();
  publish_data_to_cloud_SIM();
  delay(500);
}

void state_machine_run() 
{
  fsrValue = analogRead(FSR_PIN);
  switch(state)
  {
    case RESET:
      Serial.println("RESET");
      if(fsrValue >= fsrThreshold){
        state = START;
      }
      break;
       
    case START:
      Serial.println("START");
      if(fsrValue < fsrThreshold){
        state = RESET;
        break;
      }
      
      getreadings();
      if(squareDistanceAcc<accThreshold){
        if(squareDistanceGyro<gyroThreshold){
          engineIdleTime = millis();
          state = ENGINE_OFF;
          break;
        }
      }
      
      if(hic >= hicMinThreshold){
        state = SOFT_ALARM;
      }
      
      Serial.println("START END");
      break;

    case ENGINE_OFF:
      Serial.println("ENGINE_OFF");

      // Check if there is child in the car
      if(fsrValue < fsrThreshold){
        state = RESET;
        break;
      }

      // Read gyroscope, accelerometer and thermometer values
      getreadings();
      printSensorsValues();

      // Check for car movement
      if(squareDistanceAcc>accThreshold){
        state = START;
        break;
      }
      if(squareDistanceGyro>gyroThreshold){
        state = START;
        break;
      }
      
      if(hic >= hicMinThreshold){
        asprintf(&msg, "High temperature: %.1f degrees Celsuis.", hic);
        sendMsg(msg);
        state = ALARM;
      }
      
      if((millis()-engineIdleTime) > IDLE_THRESHOLD_SMS){
        if((millis()-engineIdleSMSTime) > IDLE_DELAY_SMS){
          sendMsg("Child on board");
          engineIdleSMSTime = millis();
        }
      }
      if((millis()-engineIdleTime) > IDLE_THRESHOLD_BUZZER){
        sendMsg("Child on board");
        state = ALARM;
      }
      break;
      
    case SOFT_ALARM:
      Serial.println("SOFT_ALARM");
      asprintf(&msg, "High temperature: %.1f degrees Celsuis.", hic);
      sendMsg(msg);
      softAlarm();
      
      if(fsrValue < fsrThreshold){
        state = RESET;
        break;
      }
      else{
        state = START;
      }
      
      delay(SOFT_ALARM_DELAY);
      stopBuzzer = true;
      pthread_join(buzzerThread, (void**)(&returnValue));
      
      readDhtValues();
      calculateHeatIndex();
      if(hic >= hicMaxThreshold){
        asprintf(&msg, "Extremely High temperature: %.1f degrees Celsuis.", hic);
        sendMsg(msg);
        state = ALARM;
      }
      
      break;
 
    case ALARM:
      Serial.println("ALARM");
      alarm();
      if(fsrValue < fsrThreshold){
        state = RESET;
        stopBuzzer = true;
        pthread_join(buzzerThread, (void**)(&returnValue));
        break;
      }
      
      if ((millis() - lastTimeSMS) > SMSDelay) {
        sendMsg("Child on board");
      }
      
      getreadings();
      
      if (hic >= hicMaxThreshold){
        break;
      }
      
      if (squareDistanceAcc>accThreshold) {
        state = START;
        stopBuzzer = true;
        pthread_join(buzzerThread, (void**)(&returnValue));
        break;
      }
      if (squareDistanceGyro>gyroThreshold) {
        state = START;
        stopBuzzer = true;
        pthread_join(buzzerThread, (void**)(&returnValue));
        break;
      }
      break;

  }
}

void softAlarm(){
  buzzerParams[0] = 500;
  buzzerParams[1] = 0;
  buzzerParams[2] = 1000;
  stopBuzzer = false;
  pthread_create(&buzzerThread, NULL, alarmThread, NULL);
}

void alarm(){
  if(!stopBuzzer){
    return;
  }
  buzzerParams[0] = 2000;
  buzzerParams[1] = 500;
  buzzerParams[2] = 300;
  stopBuzzer = false;
  pthread_create(&buzzerThread, NULL, alarmThread, NULL);
}

void *alarmThread(void* args){
  int currentTone = 0;
  while(!stopBuzzer){
    ledcWriteTone(channel, buzzerParams[currentTone]);
    delay(buzzerParams[2]);
    currentTone = 1-currentTone;
    fsrValue = analogRead(FSR_PIN);
    if (fsrValue < fsrThreshold) {
      stopBuzzer = true;
    }
  }
  ledcWriteTone(channel, 0);
  pthread_exit(NULL);
}


void sendMsg(String smsMessage){
  // To send an SMS, call modem.sendSMS(SMS_TARGET, smsMessage)
  if(modem.sendSMS(SMS_TARGET, smsMessage)){
    SerialMon.println(smsMessage);
  }
  else{
    SerialMon.println("SMS failed to send");
  }
  lastTimeSMS = millis();
}

void getreadings(){
  
  lastAccX = accX;
  lastAccY = accY;
  lastAccZ = accZ;
  
  lastGyroX = gyroX;
  lastGyroY = gyroY;
  lastGyroZ = gyroZ;

  readMpuValues();
  readDhtValues();

  calculateHeatIndex();
  calculateSquareDistance();
}

void readMpuValues() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.requestFrom((uint16_t)MPU6050_ADDR, (uint8_t)14, true); // request a total of 14 registers
  
  accX = wireReadValue; // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY = wireReadValue; // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ = wireReadValue; // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  gyroX = wireReadValue; // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  gyroY = wireReadValue; // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  gyroZ = wireReadValue; // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  mpuTemperature = wireReadValue; // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
}

void readDhtValues() {
  // Read humidity
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
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

void printSensorsValues() {
    Serial.print("temp: ");
    Serial.println(hic);
    Serial.print("square acc: ");
    Serial.println(squareDistanceAcc);
    Serial.print("square gyro: ");
    Serial.println(squareDistanceGyro);
}

void publish_data_to_cloud()
{
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    String httpRequestData = "api_key=" + apiKeyValue + "&temperature=" + String(t)
                             + "&weight=" + String(fsrValue) + "&gyro=" + String(squareDistanceGyro)
                             + "&accelerometer=" + String(squareDistanceAcc) + "&state=" + String(state) + "";

    Serial.print("httpRequestData: ");
    Serial.println(httpRequestData);
    int httpResponseCode = http.POST(httpRequestData);
    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
}

void publish_data_to_cloud_SIM() {
//  SerialMon.print("Connecting to APN: ");
//  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
//    SerialMon.println(" fail");
  }
  else {
//    SerialMon.println(" OK");
//    
//    SerialMon.print("Connecting to ");
//    SerialMon.print(server);
    if (!client.connect(server, port)) {
//      SerialMon.println(" fail");
    }
    else {
//      SerialMon.println(" OK");
//      SerialMon.println("Performing HTTP POST request...");
      // Prepare your HTTP POST request data (Temperature in Celsius degrees)
      String httpRequestData = "api_key=" + apiKeyValue + "&temperature=" + String(t)
                             + "&weight=" + String(fsrValue) + "&gyro=" + String(squareDistanceGyro)
                             + "&accelerometer=" + String(squareDistanceAcc) + "&state=" + String(state) + "";
    
      client.print(String("POST ") + resource + " HTTP/1.1\r\n");
      client.print(String("Host: ") + server + "\r\n");
      client.println("Connection: close");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.println(httpRequestData.length());
      client.println();
      client.println(httpRequestData);

      unsigned long timeout = millis(); 
      while (client.connected() && millis() - timeout < 10000L) {
        // Print available data (HTTP response from server)
        while (client.available()) {
          char c = client.read();
          SerialMon.print(c);
          timeout = millis();
        }
      }
      client.stop();
      modem.gprsDisconnect();
    }
  }
}
