#define SECRET_SSID  "Team_16"
#define SECRET_PASS  "lunabots"
#define SERVER_ADDR "192.168.1.2"
#define SERVER_PORT 1883
#define TOPIC_NAME_IN "robotCmds/Lidar_Data"
#define MQTT_RECONNECT_TIMEOUT 200
#define MQTT_READ_TIMEOUT 50
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#define WIFI_CHANNEL 1
#define MQTT_RECONNECT_TIMEOUT 200
#define MQTT_READ_TIMEOUT 50
// Create MQTT client on the ESP
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER_ADDR, SERVER_PORT, "robotCmds/Lidar_Data", "");
Adafruit_MQTT_Publish inTopic = Adafruit_MQTT_Publish(&mqtt, TOPIC_NAME_IN);
//Adafruit_MQTT_Subscribe inTopic = Adafruit_MQTT_Subscribe(&mqtt, TOPIC_NAME_IN);
// Lidar Pins
const int lidar_pins[4] = { 12, 11, 10, 9};
