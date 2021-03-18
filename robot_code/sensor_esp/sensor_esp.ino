#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WIFI_SSID "Team_15"
#define WIFI_PASS "lunabots"
#define WIFI_CHANNEL 1

#define SERVER_ADDR "192.168.43.201"
#define SERVER_PORT 1883

#define TOPIC_NAME_OUT "robotState/sensorData" //Only publishing
#define MQTT_RECONNECT_TIMEOUT 200
#define MQTT_READ_TIMEOUT 50

#define CMD_TIMEOUT 1000
#define NEUTRAL_VAL 100

// Create MQTT client on the ESP
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER_ADDR, SERVER_PORT, TOPIC_NAME_OUT, ""); //not sure about the topic name
Adafruit_MQTT_Publish out_topic = Adafruit_MQTT_Publish(&mqtt, TOPIC_NAME_OUT);


void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);

  //  Wait until ESP is connected to wifi to proceed
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}


void loop() {
  establish_connection();
  read_and_publish();
}


void establish_connection(){
  if (mqtt.connected()) return;

  int8_t connectionStatus;
  connectionStatus = mqtt.connect();
  
  while (connectionStatus != 0){
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

    if (inByte == 0){ //synchro inByte
      char all_bytes[4];
      for(int i = 0; i < 4; i++)
      {
        all_bytes[i] = get_byte();
      }
      send_sensor_vals(all_bytes);
    }
  }
}


char get_byte()
{
  while(!(Serial.available()))
  {}
  return Serial.read();
}


void send_sensor_vals(char[] all_bytes){
  for(int i = 0; i < 4; i++){
    out_topic.publish(all_bytes[i]);
  }
}

// Order of bytes:
// 2 bytes for potentiometer
// 1 byte - lin - act 
// 1 byte:
//   1 bit - drum contact
//   2nd bit - if the drum is turning or not?
// lsb - drum contact 