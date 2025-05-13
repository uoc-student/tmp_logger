//#include <DS3231.h>
#include <SD.h>
#include <Wire.h>
#include <time.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RTClib.h>
#include "esp_sleep.h"

#define INTERVAL        30 * 60 * 1000  // Interval for reading data in ms (current: every 30 minutes)
#define OFFSET          4.0             // There is a 4 degrees Celsius Offset
#define TMP_ADDR        0x76            // BME280 is connected at I2C address 0x76 (pin connected to GND)
#define SD_CS           7               // Chip Select, can be any GPIO pin
#define DEBUG_MODE      false           // Set to true to disable deep sleep during development (prevents serial port issues)
#define SQW_PIN         1               // Square Wave Generator for the RTC clock, will trigger alarm to wake up esp32
#define DS3231_ADDR     0x68            // I²C address (to set the timer)
#define LED_BOARD       8
#define LOG_FILE        "/log.txt"      // Log file path on SD card

Adafruit_BME280         bme;            // Create an instance of the BME280 (tmp sensor)
RTC_DS3231              rtc;            // Create an instance of the DS3231 RTC (real time clock)
bool                    h12 = false;    // 12 hour format
bool                    amPM = false;   // AM / PM display
bool                    CenturyBit;     // declare a variable to receive the Century bit for year overflow


/*
 * ESP32C3 does not support RTC wakeup, 
 * it is using a GPIO LOW to wake up from deep sleep, 
 * this signal is comming from the external RTC :sunglasses :)
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/api-reference/system/sleep_modes.html#_CPPv433esp_deep_sleep_enable_gpio_wakeup8uint64_t33esp_deepsleep_gpio_wake_up_mode_t
 */

void setup() {
  Serial.begin(115200); // Start serial communication
  Serial.println("******** Initializing setup() ********");
  
  pinMode(LED_BOARD, OUTPUT);
  Wire.begin(2, 3);  // SDA = GPIO 2, SCL = GPIO 3 
  Serial.println("GPIO pins initialized");

  // Initialize the RTC and SWQ_PIN for wakeup call
  if (!rtc.begin()) {
  Serial.println("Couldn't find RTC");
  while (1);
  }
  
  pinMode(SQW_PIN, INPUT_PULLUP);  // Set pin as input with pull-up
  esp_deep_sleep_enable_gpio_wakeup(1ULL << SQW_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);

  if (rtc.lostPower()) {
  Serial.println("RTC lost power, setting the time!");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Set RTC to compile time
  } else {
    Serial.println("OK -> RTC initialized and time up to date!");
  }

  clearAlarmFlag();   // Clear any existing alarm
  setNextAlarm();     // Schedule next one
  Serial.println("OK -> RTC Alarm1 sheduled!");

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
  DateTime now = rtc.now();

  char logLine[64];
  sprintf(logLine, "%02d-%02d-%04d\t%02d:%02d:%02d\t%.1f C\t\t%.1f F\n",
          now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second(), temp_C, temp_F);

  Serial.print(logLine);  // Print to Serial

  // Writing indication, if blinking don't remove sd.
  digitalWrite(LED_BOARD, HIGH);
  delay(250);
  digitalWrite(LED_BOARD, LOW);

  // Append to log file
  File logFile = SD.open(LOG_FILE, FILE_APPEND);
  if (logFile) {
    logFile.print(logLine);
    logFile.close();
  } else {
    Serial.println("Failed to open log file for writing");
  }

  if (DEBUG_MODE) {
    Serial.println("DEBUG_MODE active — not sleeping.");
    delay(5000); // print every 5 seconds
  } else {
    Serial.println("Entering deep sleep...");
    delay(100);  // Let the message flush
    esp_deep_sleep_start();
  } 
}


/*
  Current Time Registers (starting at 0x00)
  Function Address Bits Description
  Seconds 0x00  BCD format (0–59) + A1M1 if used for Alarm
  Minutes 0x01  BCD format (0–59) + A1M2
  Hours 0x02  BCD format (0–23 or 1–12, + AM/PM flag)
  Day (DOW) 0x03  Day of week (1–7)
  Date  0x04  Day of month (1–31)
  Month 0x05  Month (1–12), bit 7 = Century bit
  Year  0x06  Year (00–99)
  
  Alarm 1 Registers (start at 0x07)
  Function  Address Bits / Notes
  Alarm1 Seconds  0x07  BCD seconds + A1M1 bit (bit 7)
  Alarm1 Minutes  0x08  BCD minutes + A1M2 bit
  Alarm1 Hours  0x09  BCD hours + A1M3 bit + 12/24 hr mode + AM/PM
  Alarm1 Day/Date 0x0A  BCD day/date + A1M4 bit + DY/DT bit (bit 6: 1 = day of week, 0 = date)
  
  Note: Bits 7 in each of the above control whether that time unit must match (A1Mx). 1 = ignore, 0 = match.

  Control and Status Registers
  Function  Address Bits Description
  Control Register  0x0E  Enables alarms, square wave, interrupts, etc.
  Status Register 0x0F  Alarm flags (A1F, A2F), Oscillator Stop Flag (OSF)
  
  🚨Bits to Clear / Set
  Control Register (0x0E):
  
  Bit 2 (INTCN) = 1 → Use interrupt (not square wave) on SQW pin
  Bit 0 (A1IE) = 1 → Enable Alarm 1 Interrupt
  Status Register (0x0F):
  Bit 0 (A1F) = 1 → Alarm 1 has triggered. You must clear it manually after wake.
  Write 0 to bit 0 to clear it after you read the alarm.
*/

void setNextAlarm() {
  DateTime now = rtc.now();

  int nextMinute = (now.minute() < 30) ? 30 : 0;
  int nextHour = (now.hour() + (now.minute() >= 30 ? 1 : 0)) % 24;

  // Convert to BCD format (required by the RTC registers)
  uint8_t seconds = 0;
  uint8_t minutes = ((nextMinute / 10) << 4) | (nextMinute % 10);
  uint8_t hours = ((nextHour / 10) << 4) | (nextHour % 10);
  uint8_t day = 0x80;  // Bit 7 = A1M4 = 1 (don’t match date)

  //1 = ignore, 0 = match.
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x07);  // Alarm 1 registers
  Wire.write(0x00);  // Seconds — match exactly (A1M1 = 0)
  Wire.write(minutes); // Minutes — match (A1M2 = 0)
  Wire.write(hours);   // Hours — match (A1M3 = 0)
  Wire.write(day);     // A1M4 = 1, match every day
  Wire.endTransmission();

  /* Enable Alarm 1 interrupt
     INTCN = 1 → enable interrupt output on SQW/INT pin
     A1IE = 1 → enable Alarm 1 interrupt
  */
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x0E);  // Read control register
  Wire.endTransmission();
  
  Wire.requestFrom(DS3231_ADDR, 1);  // request 1 byte 
  uint8_t ctrl = Wire.read();
  ctrl |= 0b00000101;  // INTCN = 1, A1IE = 1
  
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x0E);
  Wire.write(ctrl);
  Wire.endTransmission();

  Serial.printf("Alarm set for: %02d:%02d\n", nextHour, nextMinute);
}

void clearAlarmFlag() {
  // Clear Alarm 1 flag (A1F)
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x0F);  // Status register
  Wire.endTransmission(false);
  Wire.requestFrom(DS3231_ADDR, 1);
  uint8_t status = Wire.read();
  status &= ~0b00000001;  // Clear A1F

  // Write back the modified status
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x0F);
  Wire.write(status);
  Wire.endTransmission();
}
