#include <ModbusMaster.h> // from: https://github.com/4-20ma/ModbusMaster
#include <WiFi.h>
#include <PubSubClient.h>
#include <credentials.h>      // Contains: ssid, password, mqttServer, etc.
#include <ArduinoJson.h>

/* ======= WiFi & MQTT Definitions ======= */
const char* mqtt_server      = mqttServer;
const char* mqtt_client_id   = "ESP32_Sensors";  // MQTT client id
const char* mqtt_topic       = "homeassistant/sensor/esp32_npk/state";
const char* availability_topic = "homeassistant/sensor/esp32_npk/status";

/* ======= Soil Sensor MQTT Definitions ======= */
const char* mqtt_topic1         = "homeassistant/sensor/esp32_soil1/state";
const char* availability_topic1 = "homeassistant/sensor/esp32_soil1/status";

const char* mqtt_topic2         = "homeassistant/sensor/esp32_soil2/state";
const char* availability_topic2 = "homeassistant/sensor/esp32_soil2/status";

/* ======= WiFi & MQTT Clients ======= */
WiFiClient espClient;
PubSubClient client(espClient);

/* ======= RS485 Pin Definitions ======= */
#define RX2_PIN     16   // RS485 RX (to module’s RO)
#define TX2_PIN     17   // RS485 TX (to module’s DI)
#define RE_DE_PIN   4    // RS485 enable (tied to RE & DE)

/* ======= Modbus Objects ======= */
// Soil Sensor 1 (node1) – Modbus address 1
ModbusMaster node1;  
// Soil Sensor 2 (node2) – Modbus address 2
ModbusMaster node2;  
// NPK Sensor (node) – Modbus address 3
ModbusMaster node;  

/* ======= RS485 Transmission Control ======= */
void preTransmission() {
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(10);
}

void postTransmission() {
  digitalWrite(RE_DE_PIN, LOW);
  delayMicroseconds(10);
}

/* ======= WiFi Setup ======= */
void setup_wifi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

/* ======= MQTT Publish Helper ======= */
void publish_mqtt(const char* topic, JsonDocument &doc) {
  String payload;
  serializeJson(doc, payload);
  client.publish(topic, payload.c_str(), true);
}

/* ======= Home Assistant Discovery for NPK Sensor ======= */
void publish_discovery_npk() {
  StaticJsonDocument<256> doc;
  
  // Device metadata
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"] = "esp32_npk_sensor";
  device["name"] = "Soil Sensor NPK";
  device["manufacturer"] = "sensorsiot";
  device["model"] = "Soil Sensor NPK";
  device["sw_version"] = "1.0";
  
  doc["availability_topic"] = availability_topic;
  doc["state_topic"] = mqtt_topic;
  
  // --- Humidity ---
  doc["name"] = "Soil Humidity";
  doc["unit_of_measurement"] = "%";
  doc["value_template"] = "{{ value_json.humidity }}";
  doc["unique_id"] = "esp32_npk_humidity";
  publish_mqtt("homeassistant/sensor/esp32_npk_humidity/config", doc);
  
  // --- Temperature ---
  doc["name"] = "Soil Temperature";
  doc["unit_of_measurement"] = "°C";
  doc["value_template"] = "{{ value_json.temperature }}";
  doc["unique_id"] = "esp32_npk_temperature";
  publish_mqtt("homeassistant/sensor/esp32_npk_temperature/config", doc);
  
  // --- Conductivity ---
  doc["name"] = "Soil Conductivity";
  doc["unit_of_measurement"] = "uS/cm";
  doc["value_template"] = "{{ value_json.conductivity }}";
  doc["unique_id"] = "esp32_npk_conductivity";
  publish_mqtt("homeassistant/sensor/esp32_npk_conductivity/config", doc);
  
  // --- pH ---
  doc["name"] = "Soil pH";
  doc["unit_of_measurement"] = "pH";
  doc["value_template"] = "{{ value_json.ph }}";
  doc["unique_id"] = "esp32_npk_ph";
  publish_mqtt("homeassistant/sensor/esp32_npk_ph/config", doc);
  
  // --- Nitrogen ---
  doc["name"] = "Soil Nitrogen";
  doc["unit_of_measurement"] = "mg/kg";
  doc["value_template"] = "{{ value_json.nitrogen }}";
  doc["unique_id"] = "esp32_npk_nitrogen";
  publish_mqtt("homeassistant/sensor/esp32_npk_nitrogen/config", doc);
  
  // --- Phosphorus ---
  doc["name"] = "Soil Phosphorus";
  doc["unit_of_measurement"] = "mg/kg";
  doc["value_template"] = "{{ value_json.phosphorus }}";
  doc["unique_id"] = "esp32_npk_phosphorus";
  publish_mqtt("homeassistant/sensor/esp32_npk_phosphorus/config", doc);
  
  // --- Potassium ---
  doc["name"] = "Soil Potassium";
  doc["unit_of_measurement"] = "mg/kg";
  doc["value_template"] = "{{ value_json.potassium }}";
  doc["unique_id"] = "esp32_npk_potassium";
  publish_mqtt("homeassistant/sensor/esp32_npk_potassium/config", doc);
}

/* ======= Home Assistant Discovery for a Soil Sensor ======= */
void publish_discovery_soil(const char* topic, const char* avail_topic, const char* unique_id, const char* sensor_name) {
  StaticJsonDocument<256> doc;
  
  // Device metadata
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"] = unique_id;
  device["name"] = sensor_name;
  device["manufacturer"] = "sensorsiot";
  device["model"] = "Soil Sensor Standard";
  device["sw_version"] = "1.0";
  
  doc["availability_topic"] = avail_topic;
  doc["state_topic"] = topic;
  
  // --- Humidity ---
  doc["name"] = String(sensor_name) + " Humidity";
  doc["device_class"] = "humidity";
  doc["state_class"] = "measurement";
  doc["unit_of_measurement"] = "%";
  doc["value_template"] = "{{ value_json.humidity }}";
  doc["unique_id"] = String(unique_id) + "_humidity";
  publish_mqtt((String("homeassistant/sensor/") + unique_id + "_humidity/config").c_str(), doc);
  
  // --- Temperature ---
  doc["name"] = String(sensor_name) + " Temperature";
  doc["device_class"] = "temperature";
  doc["state_class"] = "measurement";
  doc["unit_of_measurement"] = "°C";
  doc["value_template"] = "{{ value_json.temperature }}";
  doc["unique_id"] = String(unique_id) + "_temperature";
  publish_mqtt((String("homeassistant/sensor/") + unique_id + "_temperature/config").c_str(), doc);
}

/* ======= MQTT Reconnection ======= */
void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_client_id, availability_topic, 1, true, "offline")) {
      Serial.println(" Connected!");
      // Publish availability for all sensors
      client.publish(availability_topic, "online", true);
      client.publish(availability_topic1, "online", true);
      client.publish(availability_topic2, "online", true);
      // Publish Home Assistant discovery messages
      publish_discovery_npk();
      publish_discovery_soil(mqtt_topic1, availability_topic1, "esp32_soil1", "Soil Sensor 1");
      publish_discovery_soil(mqtt_topic2, availability_topic2, "esp32_soil2", "Soil Sensor 2");
    } else {
      Serial.print(" Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

/* ======= Read the NPK Sensor (7 registers) ======= */
void read_npk_sensor() {
  // Read registers starting at 0x0000 (7 registers)
  uint8_t result = node.readHoldingRegisters(0x0000, 7);
  if (result == node.ku8MBSuccess) {
    float humidity     = node.getResponseBuffer(0) * 0.1;
    float temperature  = (int16_t)node.getResponseBuffer(1) * 0.1;
    float conductivity = node.getResponseBuffer(2);
    float ph           = node.getResponseBuffer(3) * 0.1;
    float nitrogen     = node.getResponseBuffer(4);
    float phosphorus   = node.getResponseBuffer(5);
    float potassium    = node.getResponseBuffer(6);
    
    StaticJsonDocument<128> sensorData;
    sensorData["humidity"]     = humidity;
    sensorData["temperature"]  = temperature;
    sensorData["conductivity"] = conductivity;
    sensorData["ph"]           = ph;
    sensorData["nitrogen"]     = nitrogen;
    sensorData["phosphorus"]   = phosphorus;
    sensorData["potassium"]    = potassium;
    
    publish_mqtt(mqtt_topic, sensorData);
    
    Serial.printf("NPK Sensor - Humidity: %.1f %%RH\n", humidity);
    Serial.printf("NPK Sensor - Temperature: %.1f °C\n", temperature);
    Serial.printf("NPK Sensor - Conductivity: %.1f uS/cm\n", conductivity);
    Serial.printf("NPK Sensor - pH: %.1f\n", ph);
    Serial.printf("NPK Sensor - Nitrogen: %.0f mg/kg\n", nitrogen);
    Serial.printf("NPK Sensor - Phosphorus: %.0f mg/kg\n", phosphorus);
    Serial.printf("NPK Sensor - Potassium: %.0f mg/kg\n", potassium);
    Serial.println("------------------------");
  } else {
    Serial.printf("NPK Sensor - Modbus Error: 0x%X\n", result);
  }
}

/* ======= Read a Soil Sensor (3 registers) ======= */
void read_soil_sensor(ModbusMaster &sensor, const char* topic, const char* sensor_name) {
  // Read registers starting at 0x0000 (3 registers: humidity & temperature)
  uint8_t result = sensor.readHoldingRegisters(0x0000, 3);
  if (result == sensor.ku8MBSuccess) {
    float humidity    = sensor.getResponseBuffer(0) * 0.1;
    float temperature = (int16_t)sensor.getResponseBuffer(1) * 0.1;
    
    StaticJsonDocument<128> sensorData;
    sensorData["humidity"]    = humidity;
    sensorData["temperature"] = temperature;
    
    publish_mqtt(topic, sensorData);
    
    Serial.printf("%s - Humidity: %.1f %%RH\n", sensor_name, humidity);
    Serial.printf("%s - Temperature: %.1f °C\n", sensor_name, temperature);
    Serial.println("------------------------");
  } else {
    Serial.printf("%s - Modbus Error: 0x%X\n", sensor_name, result);
  }
}

/* ======= Setup ======= */
void setup() {
  Serial.begin(115200);
  // Initialize RS485 Serial2 (4800 baud, 8N1)
  Serial2.begin(4800, SERIAL_8N1, RX2_PIN, TX2_PIN);
  
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW);
  
  // Initialize Modbus nodes with unique slave addresses:
  
  // --- Soil Sensor 1 (node1) at address 1 ---
  node1.begin(1, Serial2);
  node1.preTransmission(preTransmission);
  node1.postTransmission(postTransmission);
  
  // --- Soil Sensor 2 (node2) at address 2 ---
  node2.begin(2, Serial2);
  node2.preTransmission(preTransmission);
  node2.postTransmission(postTransmission);
  
  // --- NPK Sensor (node) at address 3 ---
  node.begin(3, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

/* ======= Main Loop ======= */
void loop() {
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();
  
  // Read the NPK sensor (7 registers)
  read_npk_sensor();
  delay(1000);
  
  // Read Soil Sensor 1 (humidity and temperature)
  read_soil_sensor(node1, mqtt_topic1, "ESP32_Soil1");
  delay(1000);
  
  // Read Soil Sensor 2 (humidity and temperature)
  read_soil_sensor(node2, mqtt_topic2, "ESP32_Soil2");
  delay(2000);
}
