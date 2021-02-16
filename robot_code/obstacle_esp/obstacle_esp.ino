/*
 * obstacle_esp.ino
 * 
 * Code for the radio in the locomotion subsystem, particularly for a NodeMCU with an ESP8266 
 * module. Reads commands from the MQTT network and relays them to the locomotion subsystem
 * controller. Also provides ping functionality so the state controller can check that its
 * connection to the subsystem is established.
 * 
 * See README for descriptions on the format of the command messages.
 * 
 * TODO:
 * - Implement handlers for special drive modes (>252)
 */

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WIFI_SSID "Team_15"
#define WIFI_PASS "lunabots"
#define WIFI_CHANNEL 1

#define SERVER_ADDR "192.168.43.201"
#define SERVER_PORT 1883

#define TOPIC_NAME_OUT "robotState/obstacleData" //Only publishing
#define MQTT_RECONNECT_TIMEOUT 200
#define MQTT_READ_TIMEOUT 50

#define CMD_TIMEOUT 1000
#define NEUTRAL_VAL 100

// Create MQTT client on the ESP
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER_ADDR, SERVER_PORT, "obstacleTopic", "");
Adafruit_MQTT_Publish out_topic = Adafruit_MQTT_Publish(&mqtt, TOPIC_NAME_OUT);

void checkConnection();
void send_lidar_vals(char[] all_bytes);
char get_the_byte();

unsigned long timeLastRecv;   // Last time in milliseconds command was received
bool cmdRecv;                 // Has a command been received this cycle

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);

  timeLastRecv = 0;
  cmdRecv = false;

  //  Wait until ESP is connected to wifi to proceed
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
}

void loop() {
  if (millis() > timeLastRecv + CMD_TIMEOUT) cmdRecv = false;

  checkConnection();
  //since we're not subscribing, we only need to publish lidar values

  if (Serial.available() > 0) {
    int inByte = Serial.read();
    Serial.println("received");
    Serial.println(inByte);
    //is there a synchro byte? ya it should be 0 according to dan i dont think
    //we coded it yet
    if (inByte == 0 ){ //synchro inByte
      char all_the_bytes[4];
      int i;
      for(i = 0; i < 4; i++)
      {
        all_the_bytes[i] = get_the_byte();
      }
      //forward(inByte2, inByte3);
      send_lidar_vals(all_the_bytes);
    }
  }
}

char get_the_byte()
{
  while(!(Serial.available()))
  {}
  return Serial.read();
}

void send_lidar_vals(char[] all_bytes){
  int i; 
  for(i = 0; i < 4; i++){
    out_topic.publish(all_bytes[i]);
  }
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
