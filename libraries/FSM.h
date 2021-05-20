//PINS
#define FSR_PIN 34
#define DHT_PIN 15
#define BUZZER_PIN 25
#define DHT_TYPE DHT22

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26

// Your phone number to send SMS: + (plus sign) and country code, for Israel +972, followed by phone number
#define SMS_TARGET  "+972546591910"

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

#include <TinyGsmClient.h>

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT  Serial1

//Thresholds
#define weightThreshold 500
#define hicThreshold 30
#define IdleThresholdSMS 5*1000
#define IdleSMSDelay 5*1000
#define IdleThresholdBuzz 10*1000