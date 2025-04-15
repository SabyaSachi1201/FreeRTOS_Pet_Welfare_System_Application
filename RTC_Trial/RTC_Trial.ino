#include <Wire.h>
#include <RTClib.h>
#include <TimeLib.h> // Include the TimeLib library for time conversion

RTC_DS3231 rtc;

void setup() {
  Serial.begin(115200);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!");
    Serial.flush();
    while (1) {
      delay(10);
    }
  }

  // **Setting the RTC with a Unix timestamp (Uncomment and run only once or when needed)**
  // You would typically get this Unix timestamp from an NTP server or another source.
  //  time_t initialUnixTime = 1743998400; // Example: April 5, 2025 00:00:00 UTC
  //  rtc.adjust(DateTime(initialUnixTime));
  //  Serial.println("RTC time set with Unix timestamp!");
}

void loop() {
  DateTime now = rtc.now();

  // **Get the current time as a Unix timestamp**
  time_t currentUnixTime = now.unixtime();
  Serial.print("Current Unix Time: ");
  Serial.println(currentUnixTime);

  Serial.println();
  delay(1000);
}