/* Improved Code (Deep Copy via Queue) - Procedural Style (No RTOS) */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#include <ThingSpeak.h>
#include <Wire.h>
#include <RTClib.h>
#include <TimeLib.h>
#include <HX711.h>
#include <ESP32Servo.h>

// --- Configuration Constants ---
const char WIFI_SSID[] = "LAPTOP-9F4J3MNK 0381";
const char WIFI_PASSWORD[] = "r908#5K9";
const char THINGSPEAK_WRITE_API_KEY[] = "ZTUA7YU1VFM9AG6O";
const char THINGSPEAK_READ_API_KEY[] = "PH28HG0KLLQJLA5O";
const long THINGSPEAK_CHANNEL_ID = 2899351;
const int THINGSPEAK_READ_FIELD = 7;

// --- Pin Definitions ---
const int DHT_PIN = 4;
const int WATER_LEVEL_PIN = 34;
const int HX711_DT_PIN = 16;
const int HX711_SCK_PIN = 17;
const int SERVO_PIN = 13;

// --- Sensor Definitions ---
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
HX711 scale;
const float HX711_CALIBRATION_FACTOR = -7050;

// --- Timing Constants ---
const long UPLOAD_INTERVAL_MS = 15000;
const long SENSOR_READ_INTERVAL_MS = 5000;
const long WIFI_CONNECT_RETRY_DELAY_MS = 2000;
const int WIFI_MAX_CONNECT_RETRIES = 5;

// --- Global Variables ---
float longtude = 77.6096;
float latitude = 12.9544;
WiFiClient client;
RTC_DS3231 rtc;
Servo myservo;
long lastUpdateTime = 0;
long lastSensorReadTime = 0;
long lastReadFieldTime = 0;
bool wifiConnected = false;

// --- Data Structure ---
typedef struct {
  float temperature;
  float humidity;
  int waterLevelPercentage;
  float weightKgGlobal;
  long globalreadField;
} sensor_data_t;

// --- Function Prototypes ---
void connectWiFi();
sensor_data_t readSensors();
void uploadData(sensor_data_t data);
long readTargetTime();
void motor_actuation();

// --- Function Implementations ---

void connectWiFi() {
  Serial.println("Connecting to WiFi...");
  int retries = 0;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED && retries < WIFI_MAX_CONNECT_RETRIES) {
    delay(WIFI_CONNECT_RETRY_DELAY_MS);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    ThingSpeak.begin(client);
    wifiConnected = true;
  } else {
    Serial.println("\nFailed to connect to Wi-Fi after multiple retries!");
    wifiConnected = false;
  }
}

sensor_data_t readSensors() {
  sensor_data_t localSensorValues;

  // Read DHT11 sensor
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h) && !isnan(t)) {
    localSensorValues.temperature = t;
    localSensorValues.humidity = h;
  } else {
    Serial.println("Failed to read from DHT sensor!");
    localSensorValues.temperature = NAN;
    localSensorValues.humidity = NAN;
  }
  int rawWaterLevel = analogRead(WATER_LEVEL_PIN);
  localSensorValues.waterLevelPercentage = map(rawWaterLevel, 0, 4095, 0, 100);
  if (scale.is_ready()) {
    localSensorValues.weightKgGlobal = scale.get_units();
    Serial.print("Weight: ");
    Serial.print(localSensorValues.weightKgGlobal);
    Serial.println(" kg");
  } else {
    Serial.println("HX711 not ready");
    localSensorValues.weightKgGlobal = NAN;
  }
  Serial.printf("Temperature: %.2f *C, Humidity: %.2f %%, Water Level: %d%%\n",
                localSensorValues.temperature, localSensorValues.humidity, localSensorValues.waterLevelPercentage);
  return localSensorValues;
}

void uploadData(sensor_data_t data) {
  if (wifiConnected && !isnan(data.temperature) && !isnan(data.humidity) && data.waterLevelPercentage != -1) {
    Serial.println("--- Uploading Data to ThingSpeak ---");
    ThingSpeak.setField(1, data.temperature);
    ThingSpeak.setField(2, data.humidity);
    ThingSpeak.setField(3, longtude);
    ThingSpeak.setField(4, latitude);
    ThingSpeak.setField(5, data.waterLevelPercentage);
    ThingSpeak.setField(6, data.weightKgGlobal);

    int httpCode = ThingSpeak.writeFields(THINGSPEAK_CHANNEL_ID, THINGSPEAK_WRITE_API_KEY);
    if (httpCode == 200) {
      Serial.println("ThingSpeak channel update successful.");
    } else {
      Serial.printf("Problem updating ThingSpeak channel. HTTP error code: %d\n", httpCode);
    }
  } else {
    Serial.println("Wi-Fi not connected or invalid sensor data. Skipping upload.");
  }
}

long readTargetTime() {
  if (wifiConnected) {
    long fieldValue = ThingSpeak.readLongField(THINGSPEAK_CHANNEL_ID, THINGSPEAK_READ_FIELD, THINGSPEAK_READ_API_KEY);
    if (ThingSpeak.getLastReadStatus() == 200) {
      Serial.print("Read Value from ThingSpeak: ");
      Serial.println(fieldValue);
      return fieldValue;
    } else {
      Serial.printf("Error reading field from ThingSpeak: %d\n", ThingSpeak.getLastReadStatus());
    }
  } else {
    Serial.println("Wi-Fi not connected. Skipping ThingSpeak read.");
  }
  return 0; // Return 0 if unable to read
}

void motor_actuation() {
  Serial.println("Starting motor actuation...");

  for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {
    myservo.write(posDegrees);
    Serial.printf("Servo moving to %d degrees\n", posDegrees);
    delay(10);
  }
  delay(30);
  for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {
    myservo.write(posDegrees);
    Serial.printf("Servo moving to %d degrees\n", posDegrees);
    delay(10);
  }

  Serial.println("Motor actuation finished.");
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32 ThingSpeak Data Upload - Procedural (No RTOS)");

  // Initialize Hardware
  dht.begin();
  scale.begin(HX711_DT_PIN, HX711_SCK_PIN);
  scale.set_scale(HX711_CALIBRATION_FACTOR);
  scale.tare();
  Serial.println("HX711 initialized and tared.");
  myservo.attach(SERVO_PIN);
  Serial.printf("Servo attached to pin %d\n", SERVO_PIN);
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!");
    Serial.flush();
    while (1) {
      delay(10);
    }
  } else {
    Serial.println("RTC initialized.");
  }

  connectWiFi();
  lastUpdateTime = millis();
  lastSensorReadTime = millis();
  lastReadFieldTime = millis();
}

void loop() {
  long currentMillis = millis();

  if (currentMillis - lastSensorReadTime >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadTime = currentMillis;
    sensor_data_t currentData = readSensors();

    DateTime now = rtc.now();
    time_t currentUnixTime = now.unixtime();
    long targetTime = readTargetTime();

    if (abs(targetTime - currentUnixTime) <= 5 && targetTime != 0) {
      motor_actuation();
    }

    if (currentMillis - lastUpdateTime >= UPLOAD_INTERVAL_MS && wifiConnected) {
      lastUpdateTime = currentMillis;
      uploadData(currentData);
    }
  }

  delay(10); // Small delay to prevent busy-waiting
}