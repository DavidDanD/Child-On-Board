//FSR
#define FSR_PIN 34

// DHT
#define DHT_PIN 15
#define DHT_TYPE DHT22

// TTGO T-Call
// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb
#include <TinyGsmClient.h>
// Pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
// Thresholds
#define IDLE_THRESHOLD_SMS 	5*1000
#define IDLE_DELAY_SMS 		5*1000
// Your phone number to send SMS: + (plus sign) and country code, for Israel +972, followed by phone number
#define SMS_TARGET  "+972546591910"
// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT  Serial1

// MPU6050
#define SDA_PIN 		21
#define SCL_PIN 		22
#define MPU6050_FREQ 	100000
#define MPU6050_ADDR 	0x68

#define wireReadValue Wire.read() << 8 | Wire.read();

// BUZZER
#define BUZZER_PIN				25
#define SOFT_ALARM_DELAY		5*1000
#define IDLE_THRESHOLD_BUZZER 	10*1000


#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

void state_machine_run();
void softAlarm();
void alarm();
void *alarmThread(void* );
void sendMsg(String);
void getreadings();
void readMpuValues();
void calculateHeatIndex();
void calculateSquareDistance();