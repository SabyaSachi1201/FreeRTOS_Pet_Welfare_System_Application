/* Using Semaphore was essential here because :
Before Using Semaphore :
My WifiTask was not getting suffecient time to Initialise, before the resource was requested and subsequently allocated by the UploadTask. This was causing a reboot as
watchdog was getting triggered again and again, most probably because of a racearound condition, i.e task keeps indefinitely waiting for resource to be allocated.
The error message was something along the lines of Guru Meditation Error: Core 1 panic'ed (LoadProhibited). Exception was unhandled.

Suggestion #1
Key takaways : DHT11 is causing restarts when i'm uncommenting the livevalues reading functions, we might have to address this by implementing a mutex.

Suggestion #2
Key takeaways : Another modification that is needed to be done is : Implementing queues to share sensor data between tasks (between SensorTask and uploadTask)

Suggestion #3
Key takeaways : For POC, let me just hardcode the values, for now.

Suggestion #4
Key takeaways : Implement a structure, which holds the values of the global variables that are to be updated, pass the structure pointer by using the Queue.
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
#include <freertos/semphr.h>
#include <HX711.h> // Include the HX711 library

// WiFi credentials
const char *ssid = "LAPTOP-9F4J3MNK 0381";         // Replace with your WiFi SSID
const char *password = "r908#5K9"; // Replace with your WiFi password

RTC_DS3231 rtc;

// ThingSpeak API credentials
const char *writeApiKey = "ZTUA7YU1VFM9AG6O"; // Replace with your ThingSpeak Write API Key
const char *readApiKey = "PH28HG0KLLQJLA5O";  // Replace with your ThingSpeak Read API Key
int fieldNumberToRead = 7;                       // The ThingSpeak field number you want to read
long channelId = 2899351;                        // Replace with your ThingSpeak Channel ID
SemaphoreHandle_t wifiReadySemaphore;
QueueHandle_t targetTimeQueue;
QueueHandle_t sensorDataQueue;
SemaphoreHandle_t sensorMutex;
SemaphoreHandle_t uploadMutex;

// DHT11 sensor
#define DHTPIN 4    // Digital pin connected to the DHT11 sensor
#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);

// Water level sensor
#define WATER_LEVEL_PIN 34 // Analog pin connected to the water level sensor

// Weight sensor (HX711)
#define DT_PIN 16 // Replace with your DT pin for HX711
#define SCK_PIN 17 // Replace with your SCK pin for HX711
HX711 scale;
float calibrationFactor = -7050; // Example value, NEEDS CALIBRATION

WiFiClient client;

// Time constants for data upload (in milliseconds)
const long uploadIntervalMs = 15000; // Upload data every 15 seconds
const TickType_t uploadDelayTicks = pdMS_TO_TICKS(uploadIntervalMs);

// Task handles
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t uploadTaskHandle = NULL;
TaskHandle_t readFieldTaskHandle = NULL;
//TaskHandle_t motorTaskHandle = NULL;
/*
// Global variables for sensor data
float temperature = NAN;
float humidity = NAN;
int waterLevelPercentage = -1;
float weightKgGlobal = NAN; // Global variable for weight
long globalreadField = 0;
*/

typedef struct 
{
// Global variables for sensor data
	float temperature;
	float humidity;
	int waterLevelPercentage;
	float weightKgGlobal; // Global variable for weight
	long globalreadField;
}sensor;

	
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
  vTaskDelete(NULL);                // Delete WiFi task after connecting
}

// Sensor reading function for the sensor task
void sensorTask(void *pvParameters) {
  sensor Sensorvalues; 
  sensor *s_ptr=NULL;
  s_ptr=&Sensorvalues;
  
  Serial.println("Sensor Task started");
  xSemaphoreTake(sensorMutex,portMAX_DELAY);
  for (;;) {
    // Read DHT11 sensor
     float h = dht.readHumidity();
     float t = dht.readTemperature();

    // Check if DHT readings are valid
    if (!isnan(h) && !isnan(t)) {
      Sensorvalues.temperature = t;
      Sensorvalues.humidity = h;
    } else {
      Serial.println("Failed to read from DHT sensor!");
    }

    // Read water level sensor (analog input)
    int rawWaterLevel = analogRead(WATER_LEVEL_PIN);
    Sensorvalues.waterLevelPercentage = map(rawWaterLevel, 0, 4095, 0, 100); // Assuming 12-bit ADC
    //waterLevelPercentage = 45;

    // Read weight sensor using the HX711 library
    if (scale.is_ready()) {
	  Sensorvalues.weightKgGlobal = scale.get_units(); // Get weight in calibrated units (e.g., kg)
      Serial.print("Weight: ");
      Serial.print(Sensorvalues.weightKgGlobal);
      Serial.println(" kg");
    } else {
      Serial.println("HX711 not ready");
    }
    Serial.print("Temperature: ");
    Serial.print(Sensorvalues.temperature);
    Serial.print(" *C, Humidity: ");
    Serial.print(Sensorvalues.humidity);
    Serial.print(" %, Water Level (%): ");
    Serial.print(Sensorvalues.waterLevelPercentage);
    Serial.println();
	  xSemaphoreGive(sensorMutex);

    // Implementing the RTC code part here
    DateTime now = rtc.now();
    // **Get the current time as a Unix timestamp**
    time_t currentUnixTime = now.unixtime();
    Serial.println(currentUnixTime);
    if (xQueueReceive(targetTimeQueue, &Sensorvalues.globalreadField, 0) == pdTRUE) 
	{
      long timeDifference = abs(Sensorvalues.globalreadField - currentUnixTime);
      Serial.print("Target Unix Time Received: ");
      Serial.println(Sensorvalues.globalreadField);
      if (timeDifference <= 10) 
	  {
        Serial.println("Helloooooo (from SensorTask - Time Match)!");
        //Suspend other tasks , only make the motorActuationTask run
        // Here you would send a message to the motor actuation task
      }
    }
	else
	{
		Serial.println("Time Queue Did not receive message");
	}
   xSemaphoreTake(uploadMutex,portMAX_DELAY);
	 xQueueOverwrite(sensorDataQueue,s_ptr); //Sending the pointer to the structure
    vTaskDelay(pdMS_TO_TICKS(5000)); // Read sensors every 2 seconds //changed from 2000
  }
}

void readFieldTask(void *pvParameters) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      long fieldValue = ThingSpeak.readLongField(channelId, fieldNumberToRead, readApiKey);
      if (ThingSpeak.getLastReadStatus() == 200) {
        Serial.print("Read Value: ");
        Serial.println(fieldValue);
        xQueueOverwrite(targetTimeQueue, &fieldValue); // Send to the queue
      } else {
        Serial.print("Error reading field: ");
        Serial.println(ThingSpeak.getLastReadStatus());
      }
    } else {
      Serial.println("Error Connecting to Wi-Fi");
    }
    vTaskDelay(uploadDelayTicks);
  }
}

void uploadTask(void *pvParameters)
 {
  Serial.println("Upload Task started");
  sensor *r_ptr=NULL;
  xSemaphoreTake(wifiReadySemaphore, portMAX_DELAY); // Wait for Wi-Fi to be ready
  xSemaphoreGive(uploadMutex);

  if (xQueueReceive(sensorDataQueue,&r_ptr,portMAX_DELAY) == pdTRUE && r_ptr!=NULL) 
  {
     
  // vTaskDelay(pdMS_TO_TICKS(100)); // Small delay after getting semaphore
	  for (;;) 
	  {
        if (WiFi.status() == WL_CONNECTED && !isnan(r_ptr->temperature) && !isnan(r_ptr->humidity) && r_ptr->waterLevelPercentage != -1) 
        {
          Serial.println("******************************Check:*******************************");  
          Serial.print(r_ptr->temperature);  
          ThingSpeak.setField(1,r_ptr->temperature);
          ThingSpeak.setField(2,r_ptr->humidity);
          ThingSpeak.setField(5,r_ptr->waterLevelPercentage);
          ThingSpeak.setField(6,r_ptr->weightKgGlobal); // Use the global weight variable

          int httpCode = ThingSpeak.writeFields(channelId, writeApiKey);

          if (httpCode == 200) // HTTP success code (HTTP_OK) has value 200
          {
            Serial.println("Channel update successful.");
          } else 
          {
            Serial.println("Problem updating channel. HTTP error code " + httpCode);
          }
        }
        //  vPortFree(r_ptr); // Free the dynamically allocated memory
        // r_ptr = NULL;     // Reset the pointer
	  }
  }
  else
  {
    Serial.println("Faulty Queue Communication");
  }
  vTaskDelay(uploadDelayTicks);

}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 ThingSpeak Data Upload with FreeRTOS");
  dht.begin();
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale(calibrationFactor);
  scale.tare(); // Tare the scale
  Serial.println("HX711 initialized and tared.");
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!");
    Serial.flush();
    while (1) {
      delay(10);
    }
  }
  // Create the binary semaphore BEFORE creating tasks
  wifiReadySemaphore = xSemaphoreCreateBinary();
  if (wifiReadySemaphore == NULL) {
    Serial.println("Error creating wifiReadySemaphore!");
    while (1)
      ; // Halt execution if semaphore creation fails
  }
  // Create the queue for target time
  targetTimeQueue = xQueueCreate(1, sizeof(long));
  if (targetTimeQueue == NULL) {
    Serial.println("Error creating targetTimeQueue!");
  }
  sensorDataQueue = xQueueCreate(1,sizeof(sensor*));
  if(sensorDataQueue==NULL)
  {
	  Serial.println("Error creating sensorDataQueue");
  }
  // Initialize HX711 scale
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale(calibrationFactor);
  scale.tare(); // Tare the scale
	
	sensorMutex = xSemaphoreCreateMutex();
  uploadMutex = xSemaphoreCreateMutex();
  // Create tasks
  xTaskCreate(wifiTask, "WiFiTask", 2048, NULL, 5, &wifiTaskHandle);
  xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 4, &sensorTaskHandle);
  xTaskCreate(uploadTask, "UploadTask", 4096, NULL, 3, &uploadTaskHandle); // Increased stack size
  xTaskCreate(readFieldTask, "ReadFieldTask", 4096, NULL, 3, &readFieldTaskHandle);
//xTaskCreate(motorActuationTask, "motorTask", 1024, NULL, 2, &motorTaskHandle);
  // Tasks will now run independently, so no need for further code in setup
}

void loop() {
  // loop function is empty as tasks are running independently
}