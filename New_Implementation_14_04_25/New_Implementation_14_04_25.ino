/* Using Semaphore was essential here because :
Before Using Semaphore :
My WifiTask was not getting suffecient time to Initialise, before the resource was requested and subsequently allocated by the UploadTask. This was causing a reboot as
watchdog was getting triggered again and again, most probably because of a racearound condition, i.e task keeps indefinitely waiting for resource to be allocated.
The error message was something along the lines of Guru Meditation Error: Core 1 panic'ed (LoadProhibited). Exception was unhandled.

Suggestion #1 (done)
Key takaways : DHT11 is causing restarts when i'm uncommenting the livevalues reading functions, we might have to address this by implementing a mutex.

Suggestion #2 (done)
Key takeaways : Another modification that is needed to be done is : Implementing queues to share sensor data between tasks (between SensorTask and uploadTask)

Suggestion #3 (will do for GPS)
Key takeaways : For POC, let me just hardcode the values, for now.

Suggestion #4 (done)
Key takeaways : Implement a structure, which holds the values of the global variables that are to be updated, pass the structure pointer by using the Queue.
/* Improved Code (Deep Copy via Queue) */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#include <ThingSpeak.h>
#include <Wire.h>
#include <RTClib.h>
#include <TimeLib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h> // Explicitly include queue header
#include <HX711.h>

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

// --- Sensor Definitions ---
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
HX711 scale;
const float HX711_CALIBRATION_FACTOR = -7050;

TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t uploadTaskHandle = NULL;
TaskHandle_t readFieldTaskHandle = NULL;
//TaskHandle_t motorTaskHandle = NULL;

// --- FreeRTOS Configuration ---
SemaphoreHandle_t wifiReadySemaphore;
QueueHandle_t targetTimeQueue;
QueueHandle_t sensorDataQueue;
SemaphoreHandle_t sensorMutex;
SemaphoreHandle_t uploadMutex;

// --- Task Configuration ---
const int WIFI_TASK_STACK_SIZE = 4096;
const int SENSOR_TASK_STACK_SIZE = 8192;
const int UPLOAD_TASK_STACK_SIZE = 8192;
const int READ_FIELD_TASK_STACK_SIZE = 4096;
const int WIFI_TASK_PRIORITY = 5;
const int SENSOR_TASK_PRIORITY = 4;
const int UPLOAD_TASK_PRIORITY = 3;
const int READ_FIELD_TASK_PRIORITY = 3;

// --- Timing Constants ---
const long UPLOAD_INTERVAL_MS = 15000;
const TickType_t UPLOAD_DELAY_TICKS = pdMS_TO_TICKS(UPLOAD_INTERVAL_MS);
const long SENSOR_READ_INTERVAL_MS = 5000;
const TickType_t SENSOR_READ_DELAY_TICKS = pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS);
const long WIFI_CONNECT_RETRY_DELAY_MS = 2000;
const int WIFI_MAX_CONNECT_RETRIES = 5;

WiFiClient client;
RTC_DS3231 rtc;

typedef struct {
    float temperature;
    float humidity;
    int waterLevelPercentage;
    float weightKgGlobal;
    long globalreadField;
} sensor_data_t;

// --- Task Functions ---

void wifiTask(void *pvParameters) {
    Serial.println("WiFi Task started");
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
        xSemaphoreGive(wifiReadySemaphore); // Signal that Wi-Fi is ready
    } else {
        Serial.println("\nFailed to connect to Wi-Fi after multiple retries!");
        // Consider what to do if Wi-Fi fails permanently (e.g., enter a low-power state)
    }
    vTaskDelete(NULL); // Delete WiFi task after attempting connection
}

void sensorTask(void *pvParameters) {
    Serial.println("Sensor Task started");
    sensor_data_t localSensorValues; // Local structure for reading

    for (;;) {
        xSemaphoreTake(sensorMutex, portMAX_DELAY);

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
        xSemaphoreGive(sensorMutex);

        // RTC and Time Matching (using local structure)
        DateTime now = rtc.now();
        time_t currentUnixTime = now.unixtime();
        Serial.print("Current Unix Time: ");
        Serial.println(currentUnixTime);
        if (xQueueReceive(targetTimeQueue, &(localSensorValues.globalreadField), 0) == pdTRUE) {
            long timeDifference = abs(localSensorValues.globalreadField - currentUnixTime);
            Serial.print("Target Unix Time Received: ");
            Serial.println(localSensorValues.globalreadField);
            if (timeDifference <= 10) {
                Serial.println("Time Match (from SensorTask)!");
                // Trigger motor task or other actions here
            }
        } else {
            Serial.println("Time Queue was empty.");
        }

        xSemaphoreTake(uploadMutex, portMAX_DELAY);
        Serial.print("***** Check ***** :     ");
        Serial.println(localSensorValues.temperature);
        xQueueOverwrite(sensorDataQueue, &localSensorValues); // Send the structure itself (deep copy)
        xSemaphoreGive(uploadMutex);

        vTaskDelay(SENSOR_READ_DELAY_TICKS);
    }
}

void readFieldTask(void *pvParameters) {
    Serial.println("Read Field Task started");
    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            long fieldValue = ThingSpeak.readLongField(THINGSPEAK_CHANNEL_ID, THINGSPEAK_READ_FIELD, THINGSPEAK_READ_API_KEY);
            if (ThingSpeak.getLastReadStatus() == 200) {
                Serial.print("Read Value from ThingSpeak: ");
                Serial.println(fieldValue);
                xQueueOverwrite(targetTimeQueue, &fieldValue);
            } else {
                Serial.printf("Error reading field from ThingSpeak: %d\n", ThingSpeak.getLastReadStatus());
            }
        } else {
            Serial.println("Wi-Fi not connected. Skipping ThingSpeak read.");
        }
        vTaskDelay(UPLOAD_DELAY_TICKS); // Use the same upload interval for reading as well
    }
}

void uploadTask(void *pvParameters) {
    Serial.println("Upload Task started");
    sensor_data_t receivedData; // Receive the structure directly
    xSemaphoreTake(wifiReadySemaphore, portMAX_DELAY);

    for (;;) {
        xSemaphoreTake(uploadMutex, portMAX_DELAY);
        if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
            if (WiFi.status() == WL_CONNECTED && !isnan(receivedData.temperature) && !isnan(receivedData.humidity) && receivedData.waterLevelPercentage != -1) {
                Serial.println("--- Uploading Data to ThingSpeak ---");
                ThingSpeak.setField(1, receivedData.temperature);
                ThingSpeak.setField(2, receivedData.humidity);
                ThingSpeak.setField(5, receivedData.waterLevelPercentage);
                ThingSpeak.setField(6, receivedData.weightKgGlobal);

                int httpCode = ThingSpeak.writeFields(THINGSPEAK_CHANNEL_ID, THINGSPEAK_WRITE_API_KEY);
                if (httpCode == 200) {
                    Serial.println("ThingSpeak channel update successful.");
                } else {
                    Serial.printf("Problem updating ThingSpeak channel. HTTP error code: %d\n", httpCode);
                }
            } else {
                Serial.println("Wi-Fi not connected or invalid sensor data. Skipping upload.");
            }
        } else {
            Serial.println("Error receiving sensor data from queue.");
        }
        xSemaphoreGive(uploadMutex);
        vTaskDelay(UPLOAD_DELAY_TICKS);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nESP32 ThingSpeak Data Upload with FreeRTOS (Deep Copy)");

    // Initialize Hardware
    dht.begin();
    scale.begin(HX711_DT_PIN, HX711_SCK_PIN);
    scale.set_scale(HX711_CALIBRATION_FACTOR);
    scale.tare();
    Serial.println("HX711 initialized and tared.");

    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC!");
        Serial.flush();
        while (1) {
            delay(10);
        }
    }

    // Create FreeRTOS Objects
    wifiReadySemaphore = xSemaphoreCreateBinary();
    if (wifiReadySemaphore == NULL) {
        Serial.println("Error creating wifiReadySemaphore!");
        while (1) yield();
    }

    targetTimeQueue = xQueueCreate(1, sizeof(long));
    if (targetTimeQueue == NULL) {
        Serial.println("Error creating targetTimeQueue!");
        while (1) yield();
    }

    sensorDataQueue = xQueueCreate(1, sizeof(sensor_data_t)); // Changed to size of the structure
    if (sensorDataQueue == NULL) {
        Serial.println("Error creating sensorDataQueue!");
        while (1) yield();
    }

    sensorMutex = xSemaphoreCreateMutex();
    if (sensorMutex == NULL) {
        Serial.println("Error creating sensorMutex!");
        while (1) yield();
    }

    uploadMutex = xSemaphoreCreateMutex();
    if (uploadMutex == NULL) {
        Serial.println("Error creating uploadMutex!");
        while (1) yield();
    }

    // Create Tasks
    xTaskCreate(wifiTask, "WiFiTask", WIFI_TASK_STACK_SIZE, NULL, WIFI_TASK_PRIORITY, &wifiTaskHandle);
    xTaskCreate(sensorTask, "SensorTask", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, &sensorTaskHandle);
    xTaskCreate(uploadTask, "UploadTask", UPLOAD_TASK_STACK_SIZE, NULL, UPLOAD_TASK_PRIORITY, &uploadTaskHandle);
    xTaskCreate(readFieldTask, "ReadFieldTask", READ_FIELD_TASK_STACK_SIZE, NULL, READ_FIELD_TASK_PRIORITY, &readFieldTaskHandle);
    
    // No further code needed in setup as tasks are running
}

void loop() {
    // Empty loop, FreeRTOS tasks are managing the execution
}