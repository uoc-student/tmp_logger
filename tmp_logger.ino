#include <DS3231.h>
#include <SD.h>
#include <Wire.h>
#include <time.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define INTERVAL        30 * 60 * 1000  // Interval for reading data in ms (current: every 30 minutes)
#define OFFSET          4.0             // There is a 4 degrees Celsius Offset
#define TMP_ADDR        0x76            // BME280 is connected at I2C address 0x76 (pin connected to GND)
#define SD_CS           7               // Chip Select, can be any GPIO pin
#define LED_BUILDIN     8               // Buildin blue led
#define DEBUG_MODE      false           // Set to true to disable deep sleep during development (prevents serial port issues)

Adafruit_BME280         bme;            // Create an instance of the BME280 (tmp sensor)
DS3231                  rtc;            // Create an instance of the DS3231 real time clock
bool                    h12 = false;    // 12 hour format
bool                    amPM = false;   // AM / PM display
bool                    CenturyBit;     // declare a variable to receive the Century bit for year overflow


void setup() {
  Serial.begin(115200); // Start serial communication
  Serial.println("******** Initializing setup() ********");
  
  pinMode(LED_BUILDIN, OUTPUT);
  Wire.begin(2, 3);  // SDA = GPIO 2, SCL = GPIO 3 

  // Set the RTC to 24-hour mode
  rtc.setClockMode(false);
  
  // Update Clock's Date and Time (see funtion).
  //inputClockDateTime(25, 5, 04, 1, 12, 19, 30);

  // Initialize the BME280 sensor
  if (!bme.begin(TMP_ADDR)) { // Check if the BME280 is connected at I2C address 0x76
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  } else {
    Serial.println("OK -> Temperature Sensor Initialized");
  }

  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    while (1);
  } else {
    Serial.println("OK -> Card Mount Initialized");
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    while (1);
  } else {
    Serial.println("OK -> SD Card Initialized.");
  }

  Serial.println("******** SETUP SUCCESSFULL ********");
}

void loop () {
  // Read temperature in Celsius from BME280
  float temp_C = bme.readTemperature() - OFFSET;
  // Convert Celsius to Fahrenheit
  float temp_F = (temp_C * 9.0 / 5.0) + 32.0;

  // Get current time from RTC
  uint8_t hour = rtc.getHour(h12, amPM); // (h12 clock, AMPM mode) 
  uint8_t minute = rtc.getMinute();
  uint8_t second = rtc.getSecond();
  uint8_t day = rtc.getDate();
  uint8_t month = rtc.getMonth(CenturyBit);
  uint16_t year = rtc.getYear() + 2000; // RTC year is in two digits (e.g., 25 for 2025)

  char logLine[64];
  sprintf(logLine, "%02d-%02d-%04d\t%02d:%02d:%02d\t%.1f C\t\t%.1f F\n",
          day, month, year, hour, minute, second, temp_C, temp_F);

  Serial.print(logLine);  // Print to Serial

  // Writing indication, if blinking don't remove sd.
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }

  // Append to log file
  File logFile = SD.open("/log.txt", FILE_APPEND);
  if (logFile) {
    logFile.print(logLine);
    logFile.close();
  } else {
    Serial.println("Failed to open log file for writing");
  }
  
  if (!DEBUG_MODE) {
  Serial.println("Entering deep sleep...");
  delay(100);  // Let the message flush
  esp_sleep_enable_timer_wakeup((uint64_t)INTERVAL * 1000);
  esp_deep_sleep_start();
  } else {
    Serial.println("DEBUG_MODE active — not sleeping.");
  }
}


void inputClockDateTime(byte year, byte month, byte day, byte dow, byte hh, byte mm, byte ss) {
  rtc.setYear(year);    // Year: 00-99
  rtc.setMonth(month);    // Month: 1-12
  rtc.setDate(day);    // Date: 1-31
  rtc.setDoW(dow);      // Day of Week: Tuesday (1=Sunday, 2=Monday, ..., 7=Saturday)
  rtc.setHour(hh);    // Hour: 0-23 
  rtc.setMinute(mm);  // Minute: 0-59
  rtc.setSecond(ss);   // Second: 0-59

  Serial.println("RTC time and date set.");
}
