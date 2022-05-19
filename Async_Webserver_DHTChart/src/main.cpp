#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
//#include <ESPAsyncTCP.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

const char* ssid = "2anhem";
const char* password = "boo112904";

#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

AsyncWebServer server(80);

String readDHT11Temperature() {
  float t = dht.readTemperature();
  //t = 1.8 * t + 32; (F)
  if (isnan(t)) {    
    Serial.println("Failed to read from DHT11 sensor!");
    return "";
  }
  else {
    Serial.println(t);
    return String(t);
  }
}

String readDHT11Humidity() {
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("Failed to read from DHT11 sensor!");
    return "";
  }
  else {
    Serial.println(h);
    return String(h);
  }
}

void setup(){
  Serial.begin(115200);

  dht.begin();  
  // Initialize SPIFFS
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println(WiFi.localIP());

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readDHT11Temperature().c_str());
  });
  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readDHT11Humidity().c_str());
  });

  // Start server
  server.begin();
}
 
void loop(){
  
}