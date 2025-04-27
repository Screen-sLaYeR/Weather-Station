#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// WiFi credentials
#define WIFI_SSID "Sweet Home"
#define WIFI_PASSWORD "9619702210"

// Firebase Realtime Database URL
const char* databaseURL = "http://weather-station-7f947-default-rtdb.firebaseio.com/sensor.json";

// Sensor Pins
#define DHTPIN 23
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Anemometer Pin
#define ANEMOMETER_PIN 4

// Wind Vane Hall Sensor Pins
#define WIND_HALL_1 27
#define WIND_HALL_2 26
#define WIND_HALL_3 25
#define WIND_HALL_4 33

// Global Variables
volatile int anemometerTicks = 0;
unsigned long lastAnemometerCheck = 0;
float windSpeed_mps = 0.0;

Adafruit_BMP280 bmp;

// ISR for anemometer pulse counting
void IRAM_ATTR anemometerISR() {
  anemometerTicks++;
}

void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  // Initialize sensors
  dht.begin();
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found!");
    while (1);
  }

  // Setup Anemometer pin
  pinMode(ANEMOMETER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), anemometerISR, FALLING);

  // Setup Wind vane hall sensor pins
  pinMode(WIND_HALL_1, INPUT_PULLUP);
  pinMode(WIND_HALL_2, INPUT_PULLUP);
  pinMode(WIND_HALL_3, INPUT_PULLUP);
  pinMode(WIND_HALL_4, INPUT_PULLUP);
}

void loop() {
  unsigned long currentMillis = millis();

  // Check wind speed every 5 seconds
  if (currentMillis - lastAnemometerCheck >= 5000) {
    noInterrupts();
    int tickCount = anemometerTicks;
    anemometerTicks = 0;
    interrupts();

    // Assuming 1 tick per rotation
    float rotationsPerSecond = tickCount / 5.0;
    float circumference_cm = 2 * PI * (9.42 / (2 * sin(PI / 3))); // 3 arms = 120 degrees apart
    float linearSpeed_cmps = rotationsPerSecond * circumference_cm;
    windSpeed_mps = linearSpeed_cmps / 100.0; // Convert cm/s to m/s

    lastAnemometerCheck = currentMillis;
  }

  // Read DHT22
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Read BMP280
  float pressure = bmp.readPressure() / 100.0F; // hPa

  // Read Wind Vane Direction
  int windDirHall1 = digitalRead(WIND_HALL_1);
  int windDirHall2 = digitalRead(WIND_HALL_2);
  int windDirHall3 = digitalRead(WIND_HALL_3);
  int windDirHall4 = digitalRead(WIND_HALL_4);

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Prepare JSON Payload
  String payload = "{";
  payload += "\"temperature\":" + String(temperature) + ",";
  payload += "\"humidity\":" + String(humidity) + ",";
  payload += "\"pressure\":" + String(pressure) + ",";
  payload += "\"wind_speed\":" + String(windSpeed_mps);
  payload += "}";

  Serial.println("Sending payload: " + payload);

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(databaseURL);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.PUT(payload); // You can also use .POST(payload)

    if (httpResponseCode > 0) {
      Serial.printf("HTTP Response code: %d\n", httpResponseCode);
    } else {
      Serial.printf("Error code: %d\n", httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("WiFi disconnected!");
  }

  delay(5000); // Send every 5 seconds
}
