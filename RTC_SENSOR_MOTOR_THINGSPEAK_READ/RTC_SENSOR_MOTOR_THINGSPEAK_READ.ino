/* Using Semaphore was essential here because : 
Before Using Semaphore : 
My WifiTask was not getting suffecient time to Initialise, before the resource was requested and subsequently allocated by the UploadTask. This was causing a reboot as
watchdog was getting triggered again and again, most probably because of a racearound condition, i.e task keeps indefinitely waiting for resource to be allocated. 
The error message was something along the lines of Guru Meditation Error: Core 1 panic'ed (LoadProhibited). Exception was unhandled.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#include <ThingSpeak.h>
#include <Wire.h>
#include <RTClib.h>
#include <TimeLib.h> // Include the TimeLib library for time conversion
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// WiFi credentials
const char* ssid = "LAPTOP-9F4J3MNK 0381";         // Replace with your WiFi SSID
const char* password = "r908#5K9"; // Replace with your WiFi password

RTC_DS3231 rtc;

// ThingSpeak API credentials
// const char* thingspeak_host = "api.thingspeak.com";
const char* writeApiKey = "ZTUA7YU1VFM9AG6O"; // Replace with your ThingSpeak Write API Key
const char* readApiKey = "PH28HG0KLLQJLA5O"; // Replace with your ThingSpeak Read API Key
int fieldNumberToRead = 7; // The ThingSpeak field number you want to read
long channelId = 2899351;     // Replace with your ThingSpeak Channel ID
SemaphoreHandle_t wifiReadySemaphore;
QueueHandle_t targetTimeQueue;

// DHT11 sensor
#define DHTPIN 4     // Digital pin connected to the DHT11 sensor
#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);

// Water level sensor
#define WATER_LEVEL_PIN 34 // Analog pin connected to the water level sensor
WiFiClient client;

// Time constants for data upload (in milliseconds)
const long uploadIntervalMs = 15000; // Upload data every 15 seconds
const TickType_t uploadDelayTicks = pdMS_TO_TICKS(uploadIntervalMs);

// Task handles
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t uploadTaskHandle = NULL;
TaskHandle_t readFieldTaskHandle = NULL;

// Global variables for sensor data
float temperature = NAN;
float humidity = NAN;
int waterLevelPercentage = -1;
long globalreadField = 0;
// WiFi connection function for the WiFi task
void wifiTask(void *pvParameters) {
  Serial.println("WiFi Task started");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  ThingSpeak.begin(client);
  xSemaphoreGive(wifiReadySemaphore); // Signal that Wi-Fi is ready
  vTaskDelete(NULL); // Delete WiFi task after connecting
}

// Sensor reading function for the sensor task
void sensorTask(void *pvParameters) {
  Serial.println("Sensor Task started");
  dht.begin();
  for (;;) {
    // Read DHT11 sensor
    // float h = dht.readHumidity();
    // float t = dht.readTemperature();
    float h = 89.6;
    float t = 34.45;
    // Check if DHT readings are valid
    if (!isnan(h) && !isnan(t)) {
      temperature = t;
      humidity = h;
    } else {
      Serial.println("Failed to read from DHT sensor!");
    }

    // Read water level sensor (analog input)
    int rawWaterLevel = analogRead(WATER_LEVEL_PIN);
    //waterLevelPercentage = map(rawWaterLevel, 0, 4095, 0, 100); // Assuming 12-bit ADC
    waterLevelPercentage = 45;
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" *C, Humidity: ");
    Serial.print(humidity);
    Serial.print(" %, Water Level (%): ");
    Serial.print(waterLevelPercentage);
    Serial.println();

	//Implementing the RTC code part here
	 DateTime now = rtc.now();
  // **Get the current time as a Unix timestamp**
	time_t currentUnixTime = now.unixtime();
    Serial.println(currentUnixTime);
	  if (xQueueReceive(targetTimeQueue, &globalreadField, 0) == pdTRUE) 
	  {
      long timeDifference = abs(globalreadField - currentUnixTime);
		  Serial.print("Target Unix Time Received: ");
		  Serial.println(globalreadField);
		  if (timeDifference <= 5) 
		  {
			  Serial.println("Helloooooo (from SensorTask - Time Match)!");
			// Here you would send a message to the motor actuation task
		  }
	 }
    vTaskDelay(pdMS_TO_TICKS(2000)); // Read sensors every 2 seconds
  }
}

void readFieldTask(void *pvParameters)
{
	for(;;)
	{
		if (WiFi.status() == WL_CONNECTED) 
		{
			long fieldValue = ThingSpeak.readLongField(channelId, fieldNumberToRead,readApiKey);
			if (ThingSpeak.getLastReadStatus() == 200) 
			{
				Serial.print("Read Value: ");
				Serial.println(fieldValue);
				xQueueOverwrite(targetTimeQueue, &fieldValue); // Send to the queue		
			} 
			else 
			{
				Serial.print("Error reading field: ");
				Serial.println(ThingSpeak.getLastReadStatus());
			}
		}
		else
		{
			Serial.println("Error Connecting to Wi-Fi");
			
		}
		vTaskDelay(uploadDelayTicks);
	}
}
	

void uploadTask(void *pvParameters)
{
  Serial.println("Upload Task started");
  xSemaphoreTake(wifiReadySemaphore, portMAX_DELAY); // Wait for Wi-Fi to be ready
  //vTaskDelay(pdMS_TO_TICKS(100)); // Small delay after getting semaphore
  for(;;)
  {
        if (WiFi.status() == WL_CONNECTED && !isnan(temperature) && !isnan(humidity) && waterLevelPercentage != -1) 
		{
      ThingSpeak.setField(1, temperature);
			ThingSpeak.setField(2, humidity);
      ThingSpeak.setField(5,waterLevelPercentage);
		int httpCode = ThingSpeak.writeFields(channelId, writeApiKey);

		if (httpCode == 200) //HTTP success code (HTTP_OK) has value 200
    {
		Serial.println("Channel update successful.");
		} 
		else {
		Serial.println("Problem updating channel. HTTP error code " + httpCode);
		}
			vTaskDelay(uploadDelayTicks);
		}
  }
}

void setup() 
{
    Serial.begin(115200);
    Serial.println("ESP32 ThingSpeak Data Upload with FreeRTOS");
    if (!rtc.begin()) 
	{
		Serial.println("Couldn't find RTC!");
		Serial.flush();
		while (1) 
		{
		  delay(10);
		}
	}
    // Create the binary semaphore BEFORE creating tasks
    wifiReadySemaphore = xSemaphoreCreateBinary();
    if (wifiReadySemaphore == NULL) 
	{
		Serial.println("Error creating wifiReadySemaphore!");
		while(1); // Halt execution if semaphore creation fails
    }
	//  time_t initialUnixTime = 1743998400; // Example: April 5, 2025 00:00:00 UTC
	//  rtc.adjust(DateTime(initialUnixTime));
	//  Serial.println("RTC time set with Unix timestamp!");
	 // Create the queue for target time
	targetTimeQueue = xQueueCreate(1, sizeof(long));
	if (targetTimeQueue == NULL) 
	{
		Serial.println("Error creating targetTimeQueue!");
	}
    // Create tasks
    xTaskCreate(wifiTask, "WiFiTask", 2048, NULL, 5, &wifiTaskHandle);
    xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 4, &sensorTaskHandle);
    xTaskCreate(uploadTask, "UploadTask", 4096, NULL, 3, &uploadTaskHandle); // Increased stack size
	  xTaskCreate(readFieldTask, "ReadFieldTask", 4096, NULL, 3, &readFieldTaskHandle);

  // Tasks will now run independently, so no need for further code in setup
}

void loop() {
  // loop function is empty as tasks are running independently
}