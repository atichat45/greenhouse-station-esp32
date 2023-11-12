#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DHT.h>
#include <BH1750.h>
#include <Adafruit_SI1145.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID "OMAKASE_2.4G"
#define WIFI_PASSWORD "0936540643"

// Digital Ocean MQTT Mosquitto Broker
#define MQTT_HOST IPAddress(139, 59, 243, 38)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

#define MQTT_USERNAME "ESP01"
#define MQTT_PASSWORD "zx123456"

// Test MQTT Topic
#define MQTT_SUB_TEST "test"
#define MQTT_PUB_TEST "test"

//MQTT Topic
//PUB
#define MQTT_PUB_MOI "SoilMoisture"
#define MQTT_PUB_HUM "Humidity"
#define MQTT_PUB_TEMP "Temperature"
#define MQTT_PUB_LR "LightLevel"
#define MQTT_PUB_UV "UVIndex"
//SUB
#define MQTT_SUB_L1 "Light1"
#define MQTT_SUB_L2 "Light2"
#define MQTT_SUB_L3 "Light3"
#define MQTT_SUB_L4 "Light4"
#define MQTT_SUB_P1 "Pump1"
#define MQTT_SUB_P2 "Pump2"
#define MQTT_SUB_S1 "Servo1"

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

//init Global variable
float h=0.00,t=0.00,lux=0.00;
int _moisture=0, val=0;

int sensor_analog;
#define sensor_pin SOIL_MOISTURE_PIN  /* Soil moisture sensor O/P pin */

#define SEALEVELPRESSURE_HPA (1013.25)  // Define SEALEVELPRESSURE_HPA globally

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 5000;         // Interval at which to publish sensor readings

int i = 0;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

// Add more topics that want your ESP to be subscribed to
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  
  // ESP subscribed to test topic
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_SUB_TEST, 0);
  Serial.println("Subscribing at QoS 0");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// You can modify this function to handle what happens when you receive a certain message in a specific topic
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;
  for (int i = 0; i < len; i++) {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  // Check if the MQTT message was received on topic test
  if (strcmp(topic, MQTT_SUB_TEST) == 0) {
    Serial.println("TRUE");
  }
 
  Serial.println("Publish received.");
  Serial.print("  message: ");
  Serial.println(messageTemp);
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void readDHT22() {
  h = dht.readHumidity();
  t = dht.readTemperature();
  // if (isnan(h) || isnan(t)) {
  //   Serial.println(F("Failed to read from DHT sensor!"));
  //   return;
  // }

  // Serial.print(F("DHT22 - Humidity: "));
  // Serial.print(h);
  // Serial.print(F("%  Temperature: "));
  // Serial.print(t);
  // Serial.println(F(" °C"));
}

void readBH1750() {
  lux = lightMeter.readLightLevel();
  // Serial.print(F("BH1750 - Light: "));
  // Serial.print(lux);
  // Serial.println(F(" lx"));
}

void readUVSensor() {
  // Serial.print(F("UV Sensor - Visible: "));
  // Serial.print(uv.readVisible());
  // Serial.print(F(" IR: "));
  // Serial.print(uv.readIR());
  // Serial.print(F(" UV Index: "));
  // Serial.println(uv.readUV());
}

void readLDRSensor() {
  val = analogRead(LDR_PIN);
  // Serial.print(F("LDR: "));
  // Serial.println(val);
}

void readSoilMoistureSensor() {
  sensor_analog = analogRead(sensor_pin);
  _moisture = (100 - ((sensor_analog / 4095.00) * 100));
  // Serial.print(F("Soil Moisture - Moisture: "));
  // Serial.print(_moisture);
  // Serial.println(F("%"));
}

void setup() {
  Serial.begin(9600);
  Serial.println();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  // Initialize the sensors
  dht.begin();
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);  // BH1750 sensor in high-resolution mode
  uv.begin();

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  //mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  connectToWifi();
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

  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 5 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    
    //String testString = "Hello, world! #" + String(i);
    //String testString = "Temp" + String(t) + " Hum" + String(h);
    //Message to PUB
    String Temp = String(t); //Temperature
    String Hum = String(h); //Humidity
    String lux1 = String(lux); //LightLevel
    String val1 = String(val); //UVIndex
    String _moisture1 = String(_moisture); //SoilMoisture

    // Publish an MQTT message on topic test
    //uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEST, 1, true, String(testString).c_str());
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(Temp).c_str()); 
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(Hum).c_str()); 
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_LR, 1, true, String(lux1).c_str());
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_UV, 1, true, String(val1).c_str()); 
    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_MOI, 1, true, String(_moisture1).c_str());                            
    //Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEST, packetIdPub1);
    //Serial.printf(" Message: %.2f \n", testString);
    i++;
  }

}