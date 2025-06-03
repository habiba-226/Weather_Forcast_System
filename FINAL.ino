#define BLYNK_TEMPLATE_ID "TMPL2wsmXkQHa"
#define BLYNK_TEMPLATE_NAME "Weather monitoring system"
#define BLYNK_AUTH_TOKEN "j3AJn-PXGcO-8eRPAi3ZcbeN-pxJV88M"

#define BLYNK_PRINT Serial

#include <Wire.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <PubSubClient.h>

// Pin definitions
#define LDR_DIGITAL 4
#define RAIN_PIN 36
#define DHT_PIN 5
#define DHT_TYPE DHT11

// Sensor objects
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// WiFi credentials
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Orange-q6sN";
char pass[] = "YcfKF77t";

const char* mqtt_server = "broker.hivemq.com";
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Global variables
BlynkTimer timer;
int rainPercent = 0;
bool lightStatus = false;

// DHT11 readings
float dht_temperature = 0.0;
float dht_humidity = 0.0;

// BMP180 readings
float bmp_temperature = 0.0;    // Temperature from BMP180
float pressure_hPa = 0.0;       // Current pressure
float altitude_m = 0.0;         // Altitude in meters
float seaLevelPressure = 0.0;   // Sea level adjusted pressure

// Weather trend tracking
float previousPressure = 0.0;
String weatherTrend = "Stable";

float tempThreshold = 10.0;
float rainThreshold = 10.0; 

// Your approximate altitude above sea level (adjust this for your location)
const float KNOWN_ALTITUDE = 74.0; // Alexandria, Egypt is about 74m above sea level

void setup() {
  Serial.begin(115200);

  Serial.println("Starting WiFi connection...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");

  Serial.println("Connecting to Blynk...");
  Blynk.config(auth, "blynk.cloud", 80);
  Blynk.connect();

  if (Blynk.connected()) {
    Serial.println("Connected to Blynk!");
  } else {
    Serial.println("Failed to connect to Blynk.");
  }

  dht.begin();
  pinMode(LDR_DIGITAL, INPUT);
  pinMode(RAIN_PIN, INPUT);
  analogReadResolution(12);

  // Initialize BMP180
  if (!bmp.begin()) {
    Serial.println("BMP180 not detected. Check wiring.");
  } else {
    Serial.println("BMP180 connected successfully.");
    // Get initial pressure reading for trend tracking
    sensors_event_t event;
    bmp.getEvent(&event);
    if (event.pressure) {
      previousPressure = event.pressure;
    }
  }

  Serial.println("=== Enhanced Weather Monitoring System ===");
  Serial.println("Available readings:");
  Serial.println("V0: DHT Temperature, V1: DHT Humidity, V2: Rain %");
  Serial.println("V3: Pressure, V4: Light Status, V5: BMP Temperature");
  Serial.println("V6: Altitude, V7: Sea Level Pressure, V8: Weather Trend");
  Serial.println("V9: Temperature Threshold, V10: Rain Threshold");

  mqttClient.setServer(mqtt_server, 1883); // Configure the MQTT server and port (1883 default MQTT port)
  mqttClient.setCallback(mqttCallback); // Set the callback function

  timer.setInterval(5000L, updateSensors); // every 5 seconds
  timer.setInterval(10000L, debugThresholds); // Debug thresholds every 10 seconds
}

void loop() {
  Blynk.run();
  timer.run();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected - reconnecting...");
    WiFi.begin(ssid, pass);
  }

  if (!mqttClient.connected()) {
    reconnectMQTT(); // Reconnect to MQTT broker if disconnected
  }

  mqttClient.loop(); // Handle incoming/outgoing MQTT messages
}

// Add this new function to debug threshold values
void debugThresholds() {
  Serial.println("=== THRESHOLD DEBUG ===");
  Serial.print("Current Temp Threshold: ");
  Serial.println(tempThreshold);
  Serial.print("Current Rain Threshold: ");
  Serial.println(rainThreshold);
  Serial.print("Blynk Connected: ");
  Serial.println(Blynk.connected() ? "YES" : "NO");
  Serial.println("========================");
}

// Improved MQTT reconnection function
void reconnectMQTT() {
  // Only try to reconnect if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - skipping MQTT reconnect");
    return;
  }

  static unsigned long lastReconnectAttempt = 0;
  unsigned long now = millis();
  
  // Only try to reconnect every 5 seconds
  if (now - lastReconnectAttempt > 5000) {
    lastReconnectAttempt = now;
    
    if (!mqttClient.connected()) {
      Serial.print("Attempting MQTT connection...");
      
      // Create a random client ID
      String clientId = "ESP32Client-";
      clientId += String(random(0xffff), HEX);
      
      if (mqttClient.connect(clientId.c_str())) {
        Serial.println("connected!");
        
        // Subscribe to threshold control topics
        mqttClient.subscribe("weather_station_001/commands");
        mqttClient.subscribe("weather_station_001/temp_threshold");
        mqttClient.subscribe("weather_station_001/rain_threshold");
        
        // Send a "hello" message
        mqttClient.publish("weather_station_001/status", "ESP32 Weather Station Online");
        
      } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" - will try again in 5 seconds");
      }
    }
  }
}

// MQTT Callback - Enhanced with threshold control
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT Message arrived [");
  Serial.print(topic); Serial.print("]: ");

  String message = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    message += (char)payload[i];
  }
  Serial.println();

  // Handle threshold updates via MQTT
  if (String(topic) == "weather_station_001/temp_threshold") {
    float newTempThreshold = message.toFloat();
    if (newTempThreshold > 0 && newTempThreshold <= 100) {
      tempThreshold = newTempThreshold;
      Serial.println("MQTT: Temperature threshold updated to " + String(tempThreshold) + "°C");
      // Update Blynk app as well
      if (Blynk.connected()) {
        Blynk.virtualWrite(V9, tempThreshold);
      }
    }
  }
  
  if (String(topic) == "weather_station_001/rain_threshold") {
    float newRainThreshold = message.toFloat();
    if (newRainThreshold >= 0 && newRainThreshold <= 100) {
      rainThreshold = newRainThreshold;
      Serial.println("MQTT: Rain threshold updated to " + String(rainThreshold) + "%");
      // Update Blynk app as well
      if (Blynk.connected()) {
        Blynk.virtualWrite(V10, rainThreshold);
      }
    }
  }
}

void updateSensors() {
  readDHTSensor();
  readRainSensor();
  readLightSensor();
  readBMP180Sensor();
  calculateWeatherTrend();

  // Send to Blynk if connected
  if (Blynk.connected()) {
    // Original sensors
    Blynk.virtualWrite(V0, dht_temperature);      // DHT Temperature
    Blynk.virtualWrite(V1, dht_humidity);         // DHT Humidity
    Blynk.virtualWrite(V2, rainPercent);          // Rain percentage
    Blynk.virtualWrite(V3, pressure_hPa);         // Pressure
    Blynk.virtualWrite(V4, lightStatus ? 1 : 0);  // Light status
    Blynk.virtualWrite(V5, bmp_temperature);      // BMP Temperature (more accurate)
    Blynk.virtualWrite(V6, altitude_m);           // Altitude
    Blynk.virtualWrite(V7, seaLevelPressure);     // Sea level pressure
    Blynk.virtualWrite(V8, weatherTrend);         // Weather trend
    
    Serial.println("All data sent to Blynk successfully");
  } else {
    Serial.println("Blynk not connected - attempting reconnection");
    Blynk.connect();
  }

  // ALWAYS try to publish to MQTT regardless of Blynk status
  if (mqttClient.connected()) {
    Serial.println("Publishing to MQTT...");
    
    // Publish each sensor reading
    mqttClient.publish("weather_station_001/temperature", String(dht_temperature).c_str());
    mqttClient.publish("weather_station_001/humidity", String(dht_humidity).c_str());
    mqttClient.publish("weather_station_001/rain", String(rainPercent).c_str());
    mqttClient.publish("weather_station_001/pressure", String(pressure_hPa).c_str());
    mqttClient.publish("weather_station_001/light", lightStatus ? "1" : "0");
    mqttClient.publish("weather_station_001/bmp_temp", String(bmp_temperature).c_str());
    mqttClient.publish("weather_station_001/altitude", String(altitude_m).c_str());
    mqttClient.publish("weather_station_001/sea_level", String(seaLevelPressure).c_str());
    mqttClient.publish("weather_station_001/trend", weatherTrend.c_str());

    // Publish current thresholds
    mqttClient.publish("weather_station_001/current_temp_threshold", String(tempThreshold).c_str());
    mqttClient.publish("weather_station_001/current_rain_threshold", String(rainThreshold).c_str());

    Serial.println("Successfully published all sensor data to MQTT");
  } else {
    Serial.println("MQTT not connected - attempting reconnection");
    reconnectMQTT();
  }
}

void readDHTSensor() {
  dht_temperature = dht.readTemperature();
  dht_humidity = dht.readHumidity();

  if (isnan(dht_temperature) || isnan(dht_humidity)) {
    Serial.println("Failed to read from DHT11 sensor!");
    return;
  }

  Serial.print("DHT11 - Temperature: ");
  Serial.print(dht_temperature);
  Serial.print("°C, Humidity: ");
  Serial.print(dht_humidity);
  Serial.println("%");

  // Check for temperature alert
  if (dht_temperature > tempThreshold) {
    // Send to Blynk
    if (Blynk.connected()) {
      Blynk.logEvent("temp_alert", "Temperature alert! Current: " + String(dht_temperature) + "°C");
    }
    
    // Send to MQTT
    if (mqttClient.connected()) {
      String alertMsg = "Temperature Alert: " + String(dht_temperature) + "°C (Threshold: " + String(tempThreshold) + "°C)";
      mqttClient.publish("weather_station_001/alerts", alertMsg.c_str());
      Serial.println("Temperature alert sent to MQTT: " + alertMsg);
    }
    
    Serial.println("Temperature alert triggered!");
  }
}

void readRainSensor() {
  int rainRaw = analogRead(RAIN_PIN);
  rainPercent = map(rainRaw, 0, 4095, 100, 0);
  rainPercent = constrain(rainPercent, 0, 100);

  // Check for rain alert
  if (rainPercent >= rainThreshold) {
    // Send to Blynk
    if (Blynk.connected()) {
      Blynk.logEvent("heavy_rain", "Heavy Rain Alert! Rain percentage is " + String(rainPercent) + "%");
    }
    
    // Send to MQTT
    if (mqttClient.connected()) {
      String alertMsg = "Heavy Rain Alert: " + String(rainPercent) + "% (Threshold: " + String(rainThreshold) + "%)";
      mqttClient.publish("weather_station_001/alerts", alertMsg.c_str());
      Serial.println("Rain alert sent to MQTT: " + alertMsg);
    }
    
    Serial.println("Rain alert triggered!");
  }

  Serial.print("Rain - Raw: ");
  Serial.print(rainRaw);
  Serial.print(", Percentage: ");
  Serial.print(rainPercent);
  Serial.println("%");
}

void readLightSensor() {
  bool rawLightReading = digitalRead(LDR_DIGITAL);
  lightStatus = !rawLightReading;

  Serial.print("Light - Digital: ");
  Serial.println(lightStatus ? "BRIGHT" : "DARK");
}

void readBMP180Sensor() {
  sensors_event_t event;
  bmp.getEvent(&event);
  
  if (event.pressure) {
    pressure_hPa = event.pressure;
    
    // Get temperature from BMP180
    float temp;
    bmp.getTemperature(&temp);
    bmp_temperature = temp;
    
    // Calculate altitude using current pressure
    altitude_m = bmp.pressureToAltitude(1013.25, pressure_hPa);
    
    // Calculate sea level pressure using known altitude
    seaLevelPressure = pressure_hPa / pow(1.0 - (KNOWN_ALTITUDE / 44330.0), 5.255);
    
    Serial.println("=== BMP180 Readings ===");
    Serial.print("Temperature: ");
    Serial.print(bmp_temperature);
    Serial.println("°C");
    
    Serial.print("Pressure: ");
    Serial.print(pressure_hPa);
    Serial.println(" hPa");
    
    Serial.print("Altitude: ");
    Serial.print(altitude_m);
    Serial.println(" meters");
    
    Serial.print("Sea Level Pressure: ");
    Serial.print(seaLevelPressure);
    Serial.println(" hPa");
    
  } else {
    Serial.println("Error reading from BMP180");
  }
}

void calculateWeatherTrend() {
  if (previousPressure > 0) {
    float pressureChange = pressure_hPa - previousPressure;
    
    if (pressureChange > 1.0) {
      weatherTrend = "Rising";  // High pressure coming = Good weather
    } else if (pressureChange < -1.0) {
      weatherTrend = "Falling"; // Low pressure coming = Bad weather
    } else {
      weatherTrend = "Stable";  // Pressure stable
    }
    
    Serial.print("Weather Trend: ");
    Serial.print(weatherTrend);
    Serial.print(" (Change: ");
    Serial.print(pressureChange);
    Serial.println(" hPa)");
  }
  
  // Update previous pressure for next comparison
  previousPressure = pressure_hPa;
}

BLYNK_CONNECTED() {
  Serial.println("Blynk Connected - Syncing virtual pins");
  Blynk.syncAll();

  // Send initial readings
  Blynk.virtualWrite(V0, dht_temperature);
  Blynk.virtualWrite(V1, dht_humidity);
  Blynk.virtualWrite(V2, rainPercent);
  Blynk.virtualWrite(V3, pressure_hPa);
  Blynk.virtualWrite(V4, lightStatus ? 1 : 0);
  Blynk.virtualWrite(V5, bmp_temperature);
  Blynk.virtualWrite(V6, altitude_m);
  Blynk.virtualWrite(V7, seaLevelPressure);
  Blynk.virtualWrite(V8, weatherTrend);
  
  // Send initial threshold values to app
  Blynk.virtualWrite(V9, tempThreshold);
  Blynk.virtualWrite(V10, rainThreshold);
  
  Serial.println("Initial threshold values sent to Blynk");
  Serial.print("Temp Threshold: ");
  Serial.println(tempThreshold);
  Serial.print("Rain Threshold: ");
  Serial.println(rainThreshold);
}

// Virtual pin handlers for debugging sensor readings
BLYNK_WRITE(V0) { Serial.print("V0 (DHT Temp) received: "); Serial.println(param.asFloat()); }
BLYNK_WRITE(V1) { Serial.print("V1 (DHT Humidity) received: "); Serial.println(param.asFloat()); }
BLYNK_WRITE(V2) { Serial.print("V2 (Rain) received: "); Serial.println(param.asInt()); }
BLYNK_WRITE(V3) { Serial.print("V3 (Pressure) received: "); Serial.println(param.asFloat()); }
BLYNK_WRITE(V4) { Serial.print("V4 (Light) received: "); Serial.println(param.asInt()); }
BLYNK_WRITE(V5) { Serial.print("V5 (BMP Temp) received: "); Serial.println(param.asFloat()); }
BLYNK_WRITE(V6) { Serial.print("V6 (Altitude) received: "); Serial.println(param.asFloat()); }
BLYNK_WRITE(V7) { Serial.print("V7 (Sea Level Pressure) received: "); Serial.println(param.asFloat()); }
BLYNK_WRITE(V8) { Serial.print("V8 (Weather Trend) received: "); Serial.println(param.asString()); }

// Temperature threshold handler - V9
BLYNK_WRITE(V9) {
  float newTempThreshold = param.asFloat();
  Serial.println("=== V9 TEMPERATURE THRESHOLD UPDATE ===");
  Serial.print("Received value: ");
  Serial.println(newTempThreshold);
  Serial.print("Previous threshold: ");
  Serial.println(tempThreshold);
  
  // Validate the input
  if (newTempThreshold >= 0 && newTempThreshold <= 100) {
    tempThreshold = newTempThreshold;
    
    Serial.print("New threshold set to: ");
    Serial.println(tempThreshold);
    
    // Also publish to MQTT for consistency
    if (mqttClient.connected()) {
      mqttClient.publish("weather_station_001/current_temp_threshold", String(tempThreshold).c_str());
    }
  } else {
    Serial.println("Invalid temperature threshold value received!");
  }
}

// Rain threshold handler - V10
BLYNK_WRITE(V10) {
  float newRainThreshold = param.asFloat();
  Serial.println("=== V10 RAIN THRESHOLD UPDATE ===");
  Serial.print("Received value: ");
  Serial.println(newRainThreshold);
  Serial.print("Previous threshold: ");
  Serial.println(rainThreshold);
  
  // Validate the input
  if (newRainThreshold >= 0 && newRainThreshold <= 100) {
    rainThreshold = newRainThreshold;
    
    Serial.print("New threshold set to: ");
    Serial.println(rainThreshold);
    
    // Also publish to MQTT for consistency
    if (mqttClient.connected()) {
      mqttClient.publish("weather_station_001/current_rain_threshold", String(rainThreshold).c_str());
    }
  } else {
    Serial.println("Invalid rain threshold value received!");
  }
}