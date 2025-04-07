#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>

#define IR_SENSOR_PIN 25
const float ARM_DISTANCE_CM = 9.42;
const int ARM_COUNT = 3;
const int ROTATION_SAMPLES = 5;  // Averaging over 5 full rotations
const unsigned long DEBOUNCE_MICROS = 10000; // 10 ms debounce

volatile unsigned long lastTriggerMicros = 0;
volatile int tickCount = 0;
volatile bool newRotation = false;

unsigned long rotationTimestamps[ROTATION_SAMPLES];
int rotationIndex = 0;
bool bufferFilled = false;

// Sensor setup
#define DHTPIN 4 // GPIO pin connected to DHT22 (use appropriate ESP32 pin)
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp;

// WiFi credentials
const char* ssid = "Sweet Home";
const char* password = "9619702210";

// Your server endpoint
const char* serverURL = "https://weather-station-1b75f-default-rtdb.firebaseio.com/sensor.json";

void IRAM_ATTR handleIRTrigger() {
  unsigned long currentMicros = micros();
  if (currentMicros - lastTriggerMicros > DEBOUNCE_MICROS) {
    tickCount++;

    if (tickCount >= ARM_COUNT) {
      tickCount = 0;
      rotationTimestamps[rotationIndex] = currentMicros;
      rotationIndex = (rotationIndex + 1) % ROTATION_SAMPLES;
      if (rotationIndex == 0) bufferFilled = true;
      newRotation = true;
    }

    lastTriggerMicros = currentMicros;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");

  pinMode(IR_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), handleIRTrigger, FALLING);

  dht.begin();
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found!");
    while (1);
  }
}

void loop() {
  float windSpeed_mps = 0.0;
  static unsigned long lastUpdate = 0;

  if (newRotation) {
    newRotation = false;

    float totalTime = 0.0;
    int count = bufferFilled ? ROTATION_SAMPLES : rotationIndex;

    for (int i = 1; i < count; i++) {
      int indexA = (rotationIndex + ROTATION_SAMPLES - i - 1) % ROTATION_SAMPLES;
      int indexB = (rotationIndex + ROTATION_SAMPLES - i) % ROTATION_SAMPLES;
      totalTime += (rotationTimestamps[indexB] - rotationTimestamps[indexA]);
    }

    if (count > 1) {
      float avgRotationTimeSec = (totalTime / (count - 1)) / 1000000.0;
      float rps = 1.0 / avgRotationTimeSec;

      float radius_cm = ARM_DISTANCE_CM / (2 * sin(PI / ARM_COUNT));
      float circumference_cm = 2 * PI * radius_cm;
      float linearSpeed_cmps = rps * circumference_cm;
      windSpeed_mps = linearSpeed_cmps / 100.0;
    }
  }

  if (millis() - lastUpdate >= 5000) {
    lastUpdate = millis();

    float pressure = bmp.readPressure() / 100.0F; // Convert to hPa
    float humidity = dht.readHumidity();
    float temperatureDHT = dht.readTemperature();

    if (isnan(humidity) || isnan(temperatureDHT)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    Serial.printf("Temp (DHT): %.2f°C, Pressure: %.2f hPa, Humidity: %.2f%%, Wind Speed: %.2f m/s\n",
                  temperatureDHT, pressure, humidity, windSpeed_mps);

    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(serverURL);
      http.addHeader("Content-Type", "application/json");

      String payload = String("{") +
                       "\"temperature_dht\":" + temperatureDHT + "," +
                       "\"pressure\":" + pressure + "," +
                       "\"humidity\":" + humidity + "," +
                       "\"wind_speed\":" + windSpeed_mps +
                       "}";

      int httpResponseCode = http.PUT(payload);
      Serial.printf("HTTP Response Code: %d\n", httpResponseCode);
      http.end();
    } else {
      Serial.println("WiFi not connected!");
    }
  }
}
