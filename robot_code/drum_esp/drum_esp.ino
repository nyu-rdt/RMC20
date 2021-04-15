#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WIFI_SSID "Team_16"
#define WIFI_PASS "lunabots"
#define WIFI_CHANNEL 1

#define SERVER_ADDR "192.168.1.10"
#define SERVER_PORT 1883

#define TOPIC_NAME_IN "robotCmds/limbs"
#define MQTT_RECONNECT_TIMEOUT 200
#define MQTT_READ_TIMEOUT 50

#define CMD_RECV_TIMEOUT 1000
#define CMD_SEND_TIMEOUT 500
#define NEUTRAL_VAL 100

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER_ADDR, SERVER_PORT, "drumTopic", "");
Adafruit_MQTT_Subscribe inTopic = Adafruit_MQTT_Subscribe(&mqtt, TOPIC_NAME_IN);

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  while(WiFi.status() != WL_CONNECTED)
    delay(500),Serial.println("Connecting");
  mqtt.subscribe(&inTopic);
}

void loop() {
  checkConnection();
  scanAndWrite();
}
// Constantly check the connection and reconnect if it is dropped
void checkConnection(){
  if (mqtt.connected()) return;

  int8_t connectionStatus;
  connectionStatus = mqtt.connect();
  
  while (connectionStatus != 0){
    mqtt.disconnect();
    delay(MQTT_RECONNECT_TIMEOUT);
    connectionStatus = mqtt.connect();
  }
}
void scanAndWrite(){
  Adafruit_MQTT_Subscribe* subPtr;
  //Serial.println("Waiting for command");
  while ((subPtr = mqtt.readSubscription(MQTT_READ_TIMEOUT))){
    if (subPtr == &inTopic){
      Serial.println("Received data");
      // Read and translate the last incoming string of bytes
      // Incoming bytes currently come in the following format:
      //      door linAct arm drum
      char* command = (char*) inTopic.lastread; 
      char drum = command[3];
      Serial.write((int)drum);
    }
  }
}
