#include <Arduino.h>

#define FSR_PIN 34
int fsrValue = 0;
int fsrThreshold = 500;

void setup() {
  Serial.begin(115200);
  delay(500);
  // put your setup code here, to run once:

}

void loop() {
  fsrValue = analogRead(FSR_PIN);
  Serial.println(fsrValue);
  // put your main code here, to run repeatedly:

}
