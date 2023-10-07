#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "DHT.h"
#include <BH1750.h>
#include <Adafruit_SI1145.h>

// Pin configurations for different sensors
#define DHTPIN 13     // Digital pin connected to the DHT sensor
#define BH1750_SDA 26  // SDA pin for BH1750 sensor
#define BH1750_SCL 27  // SCL pin for BH1750 sensor
#define UV_SDA 18      // SDA pin for UV sensor
#define UV_SCL 19      // SCL pin for UV sensor
#define LDR_PIN 5     // Pin connected to the LDR sensor
#define SOIL_MOISTURE_PIN 4  // Pin connected to the soil moisture sensor

// Uncomment whatever type you're using!
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;
Adafruit_SI1145 uv;

int _moisture, sensor_analog;
#define sensor_pin SOIL_MOISTURE_PIN  /* Soil moisture sensor O/P pin */

#define SEALEVELPRESSURE_HPA (1013.25)  // Define SEALEVELPRESSURE_HPA globally

void setup() {
  Serial.begin(9600);

  // Initialize the sensors
  dht.begin();
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);  // BH1750 sensor in high-resolution mode
  uv.begin();
}

void readDHT22() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  Serial.print(F("DHT22 - Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.println(F(" °C"));
}

void readBH1750() {
  float lux = lightMeter.readLightLevel();
  Serial.print(F("BH1750 - Light: "));
  Serial.print(lux);
  Serial.println(F(" lx"));
}

void readUVSensor() {
  Serial.print(F("UV Sensor - Visible: "));
  Serial.print(uv.readVisible());
  Serial.print(F(" IR: "));
  Serial.print(uv.readIR());
  Serial.print(F(" UV Index: "));
  Serial.println(uv.readUV());
}

void readLDRSensor() {
  int val = analogRead(LDR_PIN);
  Serial.print(F("LDR: "));
  Serial.println(val);
}

void readSoilMoistureSensor() {
  sensor_analog = analogRead(sensor_pin);
  _moisture = (100 - ((sensor_analog / 4095.00) * 100));
  Serial.print(F("Soil Moisture - Moisture: "));
  Serial.print(_moisture);
  Serial.println(F("%"));
}

void loop() {
  // Read DHT22 sensor
  Wire.begin();  // Initialize I2C with default pins for DHT22 sensor
  readDHT22();
  delay(1000);  // Delay between sensor readings

  // Read BH1750 sensor
  Wire.begin(BH1750_SDA, BH1750_SCL);  // Initialize I2C with BH1750 sensor pins
  readBH1750();
  delay(1000);  // Delay between sensor readings

  // Read UV sensor
  Wire.begin(UV_SDA, UV_SCL);  // Initialize I2C with UV sensor pins
  readUVSensor();
  delay(1000);  // Delay between sensor readings

  // Read LDR sensor
  readLDRSensor();
  delay(1000);  // Delay between sensor readings

  // Read Soil Moisture sensor
  readSoilMoistureSensor();
  delay(1000);  // Delay between sensor readings
}
