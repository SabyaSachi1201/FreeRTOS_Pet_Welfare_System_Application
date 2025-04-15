#include <WiFi.h>
#include <DHT.h>
#include <ThingSpeak.h>

// WiFi credentials
const char* ssid = "LAPTOP-9F4J3MNK 0381";
const char* password = "r908#5K9";

// ThingSpeak credentials
unsigned long channelID = 2891889;
const char* writeAPIKey = "3B626D0GVIQ8CQLY";

// DHT11 sensor pin
#define DHTPIN 4     // ESP32 GPIO pin connected to DHT11 data pin
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);
WiFiClient client;

void setup() {
  Serial.begin(115200);
  delay(10);

  // Connect to WiFi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
  Serial.print("WiFi status: ");
  Serial.println(WiFi.status()); // Add this line
}
  Serial.println("");
  Serial.println("WiFi connected");

  // Initialize ThingSpeak
  ThingSpeak.begin(client);

  // Initialize DHT sensor
  dht.begin();
}

void loop() {
  delay(2000); // Wait a few seconds between measurements.

  // Read temperature and humidity from DHT11
  // float humidity = dht.readHumidity();
  // float temperature = dht.readTemperature(); // Gets the temperature in Celsius
  float humidity = 90.76;
  float temperature = 21.57; // Gets the temperature in Celsius
  //float temperatureF = dht.readTemperature(true); // Gets the temperature in Fahrenheit

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

    // Print values to serial monitor
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");
  //Serial.print(temperatureF);
  //Serial.println(" *F");

  // Write values to ThingSpeak
  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, humidity);
  int httpCode = ThingSpeak.writeFields(channelID, writeAPIKey);

  if (httpCode == 200) {
    Serial.println("Channel update successful.");
  } 
  else {
    Serial.println("Problem updating channel. HTTP error code " + httpCode);
  }
}