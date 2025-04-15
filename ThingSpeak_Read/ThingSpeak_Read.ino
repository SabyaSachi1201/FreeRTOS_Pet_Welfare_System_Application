#include <WiFi.h>
#include <WiFiClient.h>
#include <ThingSpeak.h>
#include <HTTPClient.h> // Add this line

/// WiFi credentials
const char* ssid = "LAPTOP-9F4J3MNK 0381";         // Replace with your WiFi SSID
const char* password = "r908#5K9"; // Replace with your WiFi password


// ThingSpeak API credentials
long channelIdRead = 2899351; // Replace with your ThingSpeak Channel ID
const char* readApiKey = "PH28HG0KLLQJLA5O"; // Replace with your ThingSpeak Read API Key
int fieldNumberToRead = 7; // The ThingSpeak field number you want to read

WiFiClient client;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 ThingSpeak Field Reader");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  ThingSpeak.begin(client);
}

void loop() {
  Serial.print("Reading Field ");
  Serial.print(fieldNumberToRead);
  Serial.print(" from Channel ");
  Serial.print(channelIdRead);
  Serial.println("...");

  // Read the last value from the specified ThingSpeak field
  long fieldValue = ThingSpeak.readLongField(channelIdRead, fieldNumberToRead,readApiKey);

  if (ThingSpeak.getLastReadStatus() == 200) {
    Serial.print("Read Value: ");
    Serial.println(fieldValue);
    // You can now use the 'fieldValue' in your logic
  } else {
    Serial.print("Error reading field: ");
    Serial.println(ThingSpeak.getLastReadStatus());
  }

  delay(15000); // Read every 15 seconds (adjust as needed)
}