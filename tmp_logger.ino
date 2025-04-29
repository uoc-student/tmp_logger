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
  //inputClockDateTime(25, 4, 15, 3, 15, 35, 30);

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

void loop() {
  Serial.println("Booo");
  delay(1000);
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
