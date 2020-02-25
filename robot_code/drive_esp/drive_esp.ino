/*
 * Changelog
 * Changed functions to have better names
 * Imlpemented proper char* casting in scanForCmd()
 */

/* NYU RDT
 * Drive Control code
 * 7    - bit string will be sent from the topic that we the esp is subscribed to
FOR DRIVE 
7bits
0 = mode
  0- forwards/backwards
  1- rotating
  2- arm movement 
1-3 = speed
  0- forwards/backwards
  1- percentage
  2- percentage
4-7 = degrees
  0-neg/pos
  1-1st dig
  2-2nd dig
  3-3th dig
 */

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// Connection information like Wifi credentials and server address
#define WIFI_SSID "Team_16"
#define WIFI_PASS "lunabots"
#define WIFI_CHANNEL 1
#define SERVER_ADDR "192.168.1.2"
#define SERVER_PORT 1883

#define TOPIC_NAME_IN "robotCmds/drive"
#define MQTT_RECONNECT_TIMEOUT 200
#define MQTT_READ_TIMEOUT 50

// Create MQTT client on the ESP
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER_ADDR, SERVER_PORT, "driveTopic", "");
Adafruit_MQTT_Subscribe inTopic = Adafruit_MQTT_Subscribe(&mqtt, TOPIC_NAME_IN);


void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);

  //  Wait until ESP is connected to wifi to proceed
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
  mqtt.subscribe(&inTopic);
}


void loop() {
  checkConnection();
  scanForCmd();
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


// Scan for any incoming data on any topic. The `subPtr` object will
// have to be compared in a switch case to determine the topic name.
void scanForCmd(){
  Adafruit_MQTT_Subscribe* subPtr;
  
  while ((subPtr = mqtt.readSubscription(MQTT_READ_TIMEOUT))){
    if (subPtr == &inTopic){
      char* command = (char*) inTopic.lastread; 
      Serial.write(command);
    }
  }
}
