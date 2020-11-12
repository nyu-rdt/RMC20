#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <SPI.h>
#include <WiFi101.h>

// Wifi and MQTT credentials
#define WIFI_SSID  "Team_16"
#define WIFI_PASS  "lunabots"
#define SERVER_ADDR "192.168.1.2"
#define SERVER_PORT 1883
#define TOPIC_NAME_IN "robotCmds/obstacleData"
#define MQTT_RECONNECT_TIMEOUT 200
#define MQTT_READ_TIMEOUT 50

char publisher();
void readSerial();
char recieved[4];
void checkConnection();

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER_ADDR, SERVER_PORT, TOPIC_NAME_IN, "");
Adafruit_MQTT_Publish inTopic = Adafruit_MQTT_Publish(&mqtt, TOPIC_NAME_IN);

// the WiFi radio’s status
int status = 1;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  Serial1.begin(9600);

  mqtt.connect();
  // wait for serial port to connect. Needed for native USB port only
  while (!Serial) { ; }

  // Feather M0 configuration
  WiFi.setPins(8,7,4,2); 
  

  // check for the presence of the antenna
  if (WiFi.status() == WL_NO_SHIELD) {
     while (true);
  }
  
  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(1000);
  }

}

void loop(){
  readSerial();
  publisher();
}

//reads data sent to us by the feather
void readSerial(){
  //check connection to feather
  if (Serial1.available()){
    int i=0;
    delay(50)
    //reading characters from feather
    //puts into global variable 
    while(Serial1.available() && i<4){
      recieved[i++] = Serial1.read();
    }
  }
}

// constantly check the connection and reconnect if it is dropped
void establishConnection(){
  if (mqtt.connected()) {
    return;
  }
  int8_t connectionStatus;
  connectionStatus = mqtt.connect();
  
  while (connectionStatus != 0){
    mqtt.disconnect();
    delay(MQTT_RECONNECT_TIMEOUT);
    connectionStatus = mqtt.connect();
  }
}

//sends all of the recieved data to the nuc in our topic 
//via MQTT 
char publisher() {
  if (mqtt.connected()){
    int len = 4;
    //char publishable [len];
    //data.toCharArray(publishable, len);
    inTopic.publish(recieved);
  }
}
