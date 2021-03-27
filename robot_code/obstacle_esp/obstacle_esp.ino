/*
 * obstacle_esp.ino
 *
 * Code for the radio in the obstacles subsystem, particularly for a NodeMCU with an ESP8266
 * module. Reads 4 LIDAR bytes from the obstacle subsystem and send it to the server through MQTT
 * Expect synchronization byte of 0 from the obstacle subsystem prior to the 4 LIDAR values
 *
 * See README for descriptions on the format of the command messages.
 *
 * TODO:
 * - Sending synchronization byte of 0 from the obstacle subsystem
 */

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WIFI_SSID "Team_16"
#define WIFI_PASS "lunabots"
#define WIFI_CHANNEL 1

#define SERVER_ADDR "192.168.1.10"
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


void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);

  //  Wait until ESP is connected to wifi to proceed
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
		Serial.println("connecting to wifi");
	}
	Serial.println("connected to wifi");

}

void loop() {
  establish_connection();
  read_and_publish();
}

// Constantly check the connection and reconnect if it is dropped
void establish_connection(){
  if (mqtt.connected()) return;

  int8_t connectionStatus;
  connectionStatus = mqtt.connect();

  while (connectionStatus != 0){
		Serial.println("connecting to mqtt");
		mqtt.disconnect();
    delay(MQTT_RECONNECT_TIMEOUT);
    connectionStatus = mqtt.connect();
  }
}

void read_and_publish()
{
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    Serial.println("received");
    Serial.println(inByte);
    //is there a synchro byte? ya it should be 0 according to dan i dont think
    //we coded it yet
    if (inByte == 0 ){ //synchro inByte
      char all_the_bytes[4];
      for(int i = 0; i < 4; i++)
      {
        all_the_bytes[i] = get_the_byte();
      }
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

void send_lidar_vals(char all_bytes[]){
  for(int i = 0; i < 4; i++){
    out_topic.publish(all_bytes[i]);
  }
}
