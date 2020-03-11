#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <SPI.h>
#include <WiFi101.h>

// Wifi and MQTT credentials
#define WIFI_SSID  "Team_16"
#define WIFI_PASS  "lunabots"
#define SERVER_ADDR "192.168.1.2"
#define SERVER_PORT 1883
#define TOPIC_NAME_IN "robotCmds/lidarData"
#define MQTT_RECONNECT_TIMEOUT 200
#define MQTT_READ_TIMEOUT 50

const int LIDAR_PINS[4] = { 12, 11, 10, 9 };

void checkConnection();
void lidarLoop();
double readLidar(int);
char publisher(String&);

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER_ADDR, SERVER_PORT, TOPIC_NAME_IN, "");
Adafruit_MQTT_Publish inTopic = Adafruit_MQTT_Publish(&mqtt, TOPIC_NAME_IN);

// the WiFi radio’s status
int status = 1;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

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

  // connect all LIDARs to appropriate pins
  for (byte i = 0; i < 4; i++){
    pinMode(LIDAR_PINS[i], INPUT_PULLUP);
  }
}

void loop() {
  //checkConnection();
  lidarLoop();
}

// measures each lidar pin using the readLidar function and stores the measurement in dists_initial
void lidarLoop() {
  String data;
  float distsInitial [4];
  
  for (int i = 0; i < 4; i++){
    double val = readLidar(LIDAR_PINS[i]);
    distsInitial[i] = val;
    delayMicroseconds(65);
    Serial.print("Port:" +String(LIDAR_PINS[i])+":"+ String(distsInitial[i]) +",    ");
    data += String(LIDAR_PINS[i]);
  }
  publisher(data);

  // waits 100 microseconds before taking a new reading
  delay(100);
}

// LIDAR pins can only be read by varying the voltage to the pin and measuring the duration one at a time
// due to limitations with the arduino and the pulseIn function
double readLidar (int i){
  digitalWrite(i,HIGH);
  delayMicroseconds(2);
  digitalWrite(i,LOW);
  
  double duration = pulseIn(i, HIGH);
  
  // this converts the duration variable into meters
  return (duration * 171.5)/1000000;
}

// constantly check the connection and reconnect if it is dropped
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

char publisher(String& data) {
  int len = data.length() +1;
  char publishable [len];
  data.toCharArray(publishable, len);
  inTopic.publish(publishable);
}
