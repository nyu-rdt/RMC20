#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WIFI_SSID "Team_16"
#define WIFI_PASS "lunabots"
#define WIFI_CHANNEL 1

#define SERVER_ADDR "192.168.1.10"
#define SERVER_PORT 1883

#define TOPIC_NAME_IN "robotCmds/limbs"
#define TOPIC_NAME_OUT "robotState/drivePing"
#define MQTT_RECONNECT_TIMEOUT 200
#define MQTT_READ_TIMEOUT 50

#define CMD_RECV_TIMEOUT 1000
#define CMD_SEND_TIMEOUT 500

#define NEUTRAL_VAL_DOOR 0
#define NEUTRAL_VAL_LIN 0
#define NEUTRAL_VAL_ARM 0
#define NEUTRAL_VAL_DRUM 0

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER_ADDR, SERVER_PORT, "drumTopic", "");
Adafruit_MQTT_Subscribe inTopic = Adafruit_MQTT_Subscribe(&mqtt, TOPIC_NAME_IN);
Adafruit_MQTT_Publish pingTopic = Adafruit_MQTT_Publish(&mqtt, TOPIC_NAME_OUT);

bool cmdRecv = false;
unsigned long timeLastRecv = 0;

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  while(WiFi.status() != WL_CONNECTED)
    delay(500);
	mqtt.subscribe(&inTopic);
}

void loop() {
	if (millis() > timeLastRecv + CMD_RECV_TIMEOUT) {
		cmdRecv = false;
	}
	checkConnection();
  scanAndWrite();

	if (!cmdRecv) {
		Serial.write(255);
		Serial.write(NEUTRAL_VAL_DOOR);
		Serial.write(NEUTRAL_VAL_LIN);
		Serial.write(NEUTRAL_VAL_ARM);
		Serial.write(NEUTRAL_VAL_DRUM);
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

void scanAndWrite(){
  Adafruit_MQTT_Subscribe* subPtr;
  //Serial.println("Waiting for command");
  while ((subPtr = mqtt.readSubscription(MQTT_READ_TIMEOUT))){
    if (subPtr == &inTopic){
			cmdRecv = true;
			timeLastRecv = millis();
			// Read and translate the last incoming string of bytes
			// Incoming bytes currently come in the following format:
			//      door linAct arm drum
			char* cmd = (char*) inTopic.lastread;
			char door = cmd[0];
			char linAct = cmd[1];
			char arm = cmd[2];
			char drum = cmd[3];
			if (door == 252 || linAct == 252 || arm == 252 || drum == 252) {
				pingTopic.publish(1);
			}
			else if (door == 255 || linAct == 255 || arm == 255 || drum == 255) {
				door = NEUTRAL_VAL_DOOR;
				linAct = NEUTRAL_VAL_LIN;
				arm = NEUTRAL_VAL_ARM;
				drum = NEUTRAL_VAL_DRUM;
				Serial.write(255);
				Serial.write(door);
				Serial.write(linAct);
				Serial.write(arm);
				Serial.write(drum);

			}
			else {
				Serial.write(255);
				Serial.write(door);
				Serial.write(linAct);
				Serial.write(arm);
				Serial.write(drum);
			}
		}
  }
}
