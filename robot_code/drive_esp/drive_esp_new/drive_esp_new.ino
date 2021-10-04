#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/*********************** ints to send to Arduino Uno *******************************/
int backward = 0; //int data
int forward = 1;
int stopMotor = 2;
int turnRight = 3;
int turnLeft = 4;

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "Team_16"
#define WLAN_PASS       "lunabots"

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;

// Set up MQTT client with nuc address and topic name
Adafruit_MQTT_Client mqtt(&client, "192.168.1.10", 1883, "motorsTopic", ""); 

/****************************** Feeds ***************************************/

// Setup a feed called 'inTopic' for subscribing to changes.
Adafruit_MQTT_Subscribe inTopic(&mqtt, "robotCmds/motors");

/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println(F("Adafruit MQTT demo"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&inTopic);
}

uint32_t x=0;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &inTopic) {
      Serial.print(F("Got: ")); //F means format -maybe Justin
      Serial.println((char *)inTopic.lastread); //Dereferencing

      // Read and transmit bytes
      // Incoming bytes currently come in the following format:
      // byte
      Serial.print("Byte message ");
      Serial.println(*inTopic.lastread);
      char* command = (char*) inTopic.lastread;
      
      //Grabbing the motor values from the byte array
      //TODO: THIS WORKS CAUSE STRUCT.PACK
      char motorData = command[0];

      // Splitting data into left and right halves
      char leftData = motorData >> 4;
      char rightData = motorData & 15; // 15 == 4'b1111

      Serial.println(motorData);

      Serial.print("leftData ");
      Serial.println((int)leftData);
      Serial.print("rightData ");
      Serial.println((int)rightData);

      //TODO: Byte stuff to remove the sign thing 
      // Motor powers 
      int rightPower = 1500;
      int leftPower = 1500; 
    
      // TODO: Determine proper direction
      // Determining power of the left motors
      int powerData = leftData & 7; // 7 == 4'b0111
      int leftDirection = leftData >> 3;
      int rightDirection = rightData >> 3;

      if(powerData == 0){
        Serial.write(stopMotor);
      }
      else if(leftDirection == 1 && rightDirection == 1){
        Serial.write(forward);
      }
      else if(leftDirection == 1 && rightDirection == 0){
        Serial.write(turnRight);
      }
      else if(leftDirection == 0 && rightDirection == 1){
        Serial.write(turnLeft);
      }
      else{
        Serial.write(backward);
      }
    }
  }

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  //We need this cause we're only subscribing so we'd lose the connection
  //otherwise
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
