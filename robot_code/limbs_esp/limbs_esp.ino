/*
 * limbs_esp.ino
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

#define TOPIC_NAME_IN "robotCmds/limbs"
#define TOPIC_NAME_OUT "robotCmds/limbsPing"
#define MQTT_RECONNECT_TIMEOUT 200
#define MQTT_READ_TIMEOUT 50

#define CMD_TIMEOUT 1000
#define NEUTRAL_VAL 100

char get_ping_bit(char byte_in);
bool get_door_state(char byte_in);
char get_lin_acts_speed(char byte_in);


// Create MQTT client on the ESP
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER_ADDR, SERVER_PORT, "driveTopic", "");
Adafruit_MQTT_Subscribe inTopic = Adafruit_MQTT_Subscribe(&mqtt, TOPIC_NAME_IN);
Adafruit_MQTT_Publish pingTopic = Adafruit_MQTT_Publish(&mqtt, TOPIC_NAME_OUT);

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
  
  mqtt.subscribe(&inTopic);
}


void loop() {
  // If no command received for CMD_TIMEOUT ms, start writing zero values to the motors at 
  // the end of the cycle
  if (millis() > timeLastRecv + CMD_TIMEOUT) cmdRecv = false;

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
      char limb_cmds = command[0];
      if(get_ping_bit(limb_cmds))
      {
        pingTopic.publish(1);
      }
      else
      {
        Serial.write(255);
        Serial.write(limb_cmds);
        //Serial.write(get_door_state(limb_cmds));
        //Serial.write(get_lin_acts_speed(limb_cmds));
        //Serial.write(get_arm_speed(limb_cmds));
        //Serial.write(get_drum_speed(limb_cmds));
        //
      }
    }
  }
}

//For EE on the controller side. You're welcome bro
//bit masking section
//the first bit is 7th bit, second is 6th,... last one is 0th.
//get 7th bit
//10000000 - 256
bool get_ping_bit(char byte_in)
{
  return ((1<<7) & byte_in) >> 7;
}
//get 6th bit
//11010101 & 01000000 = 01000000
//01000000 - 128
bool get_door_state(char byte_in)
{
  return ((1 << 6) & byte_in) >> 6;
}
//get 4th and 5th bit
//00110000 - 48
char get_lin_acts_speed(char byte_in)
{
  return ((3 << 4) & byte_in) >> 4;
}
//get 3rd and 2nd byte

//00001100 - 12
char get_arm_speed(char byte_in)
{
  return (byte_in & (3) << 2) >> 2;
}
//11010101 get last 2 - 01
//10110111 & 00000011
//00000011 - 3
char get_drum_speed(char byte_in)
{
  return byte_in & 3;
}
