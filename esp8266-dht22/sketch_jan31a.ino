#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <DHT.h>

#define DHTPIN 5       // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22  // DHT 22 (AM2302)

#define MIN_FIELD_WIDTH 4  //number of digits of float value including -ve sign
#define PRECISION 2        //number of digits after decimal
#define STR_SIZE (MIN_FIELD_WIDTH + PRECISION + 1)

#define MQTT_HOST IPAddress(192, 168, 45, 1)

// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "gaurav-pc"
#define MQTT_PORT 1883
// Temperature MQTT Topics
#define MQTT_PUB_TEMP "temp"
#define MQTT_PUB_HUMID "humidity"

#define WIFI_SSID "HumidityServer"
#define WIFI_PASSWORD "password@123"

unsigned long previousMillis = 0;  // will store last time DHT was updated
const long interval = 5000;        // Updates DHT readings every 10 seconds

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

DHT dht(DHTPIN, DHTTYPE);

/***************************************************************************************/
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}
/***************************************************************************************/

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.print("Connected to Wi-Fi: ");
  Serial.println(WiFi.localIP());
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach();  // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

/***************************************************************************************/

void setup() {
  dht.begin();
  Serial.begin(115200);
  Serial.println("-------------------------------------------");

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  connectToWifi();
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    double newT = dht.readTemperature();
    double newH = dht.readHumidity();

    if (isnan(newT)) {
      Serial.println("Failed to read Temperature from DHT sensor!");
    } else {
      uint16_t packetIdTempPub = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(newT).c_str());
      Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP, packetIdTempPub);
      Serial.printf("Message: %.2f \n", newT);
    }

    if (isnan(newH)) {
      Serial.println("Failed to read Humdity from DHT sensor!");
    } else {
      uint16_t packetIdHumidPub = mqttClient.publish(MQTT_PUB_HUMID, 1, true, String(newH).c_str());
      Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_HUMID, packetIdHumidPub);
      Serial.printf("Message: %.2f \n", newH);
    }
  }
}