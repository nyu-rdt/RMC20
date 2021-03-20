/*
 * drive_esp.ino
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

#define TOPIC_NAME_IN "robotCmds/drive"
#define TOPIC_NAME_OUT "robotState/drivePing"
#define MQTT_RECONNECT_TIMEOUT 200
#define MQTT_READ_TIMEOUT 50

#define CMD_RECV_TIMEOUT 1000
#define CMD_SEND_TIMEOUT 500
#define NEUTRAL_VAL 100

// Create MQTT client on the ESP
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER_ADDR, SERVER_PORT, "driveTopic", "");
Adafruit_MQTT_Subscribe inTopic = Adafruit_MQTT_Subscribe(&mqtt, TOPIC_NAME_IN);
Adafruit_MQTT_Publish pingTopic = Adafruit_MQTT_Publish(&mqtt, TOPIC_NAME_OUT);

unsigned long timeLastRecv;   // Last time in milliseconds command was received
bool cmdRecv;                 // Has a command been received this cycle
long lastAckTime;

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);

  timeLastRecv = 0;
  cmdRecv = false;

  //  Wait until ESP is connected to wifi to proceed
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
  mqtt.subscribe(&inTopic);
}


void loop() {
  // If no command received for CMD_TIMEOUT ms, start writing zero values to the motors at 
  // the end of the cycle
  if (millis() > timeLastRecv + CMD_RECV_TIMEOUT) cmdRecv = false;

  checkConnection();
  scanForCmd();

  // Write zero values
  if (!cmdRecv) {
    Serial.write(255);
    Serial.write(NEUTRAL_VAL);
    Serial.write(NEUTRAL_VAL);
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

// read() only "gets rid of" 1 byte, so flush gets rid of all of them
void serialFlush(){
  while(Serial.available() > 0) {
    Serial.read();
  }
}
    

// Scan for any incoming data on any topic. The `subPtr` object will have to be compared in
// a switch case to determine the topic name.
void scanForCmd(){
  Adafruit_MQTT_Subscribe* subPtr;
  
  while ((subPtr = mqtt.readSubscription(MQTT_READ_TIMEOUT))){
    if (subPtr == &inTopic){
      cmdRecv = true;
      timeLastRecv = millis();

      // Read and translate the last incoming string of bytes
      // Incoming bytes currently come in the following format:
      //      0x00  0x00  robotSpeed  offsetNum
      char* command = (char*) inTopic.lastread; 
      char offsetNum = command[2];
      char robotSpeed = command[3];
      
      //ESTOP. Speed limit of 0 mph.
      if(offsetNum == 255)
        robotSpeed = 100;
      // Check if last message was 'ping' byte; if it was, forward
      // response to ping channel
      if (offsetNum == 252) {
        pingTopic.publish(1);
      }
      // If the message is not the ping packet, process the offset and the robot speed and 
      // send the command to the controller. The header byte 255 is used for synchronizing 
      // Tx and Rx.
      else {
        while(!(Serial.available()) && (millis() - lastAckTime < CMD_SEND_TIMEOUT)) {
          Serial.write(255); // synchro byte
          Serial.write(robotSpeed);
          Serial.write(offsetNum);
        }
        if (Serial.available()) {
          serialFlush();
          lastAckTime = millis();
        }
        else {
          Serial.write(255); // synchro byte
          Serial.write(100); // 100 is neutral, so basically estop
          Serial.write(100);
        }
      }
    }
  }
}
