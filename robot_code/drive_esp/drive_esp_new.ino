/*
 * Code for the radio in the locomotion subsystem, particularly for a NodeMCU with an ESP8266 
 * module. Reads commands from the MQTT network and relays them to the locomotion subsystem
 * controller.
*/

//Calling specific Wifi routine to connect to the network

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// WiFi & Server Constants
#define WIFI_SSID "Team_16"
#define WIFI_PASS "lunabots"
#define WIFI_CHANNEL 1
#define SERVER_ADDR "192.168.1.10"
#define SERVER_PORT 1883
#define TOPIC_NAME_IN "robotCmds/motors"
//#define TOPIC_NAME_OUT "robotState/motorPing"
#define MQTT_READ_TIMEOUT 5000 //temporary value

// Create MQTT client on the ESP
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER_ADDR, SERVER_PORT, "motorsTopic", "");
Adafruit_MQTT_Subscribe inTopic(&mqtt, TOPIC_NAME_IN);
//Adafruit_MQTT_Publish pingTopic = Adafruit_MQTT_Publish(&mqtt, TOPIC_NAME_OUT);

//Motor pins
int motor1Pin = 1;
int motor2Pin = 2;
int motor3Pin = 3;
int motor4Pin = 4;

void setup()
{
  Serial.begin(115200);
  delay(10);
  
  Serial.println(F("MQTT Testing wooo!"));

  //Setting pinModes for the motor Pins
//  pinMode(motor1Pin, OUTPUT);
//  pinMode(motor2Pin, OUTPUT);
//  pinMode(motor3Pin, OUTPUT);
//  pinMode(motor4Pin, OUTPUT);

  //MQTT STUFF
  //Connecting to Wifi network
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting");

  //Checks if the connection was successful or not
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  //Prints IP address assigned to ESP module
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  //Setting up MQTT to subscribe
  mqtt.subscribe(&inTopic);
}

void loop() {
  //rdt_data = bytes([motor1, motor2, motor3, motor4])
  analogWrite(motor1Pin, 0); //temporary set output to 0 & want motorpower 
}

// Scan for any incoming data on any topic. The `subPtr` object will have to be compared in
// a switch case to determine the topic name.
void subscribe_to_motor_vals(){
  //Creating a string name to the feed name
  
  //Creating MQTT subscription object
  // Adafruit_MQTT_Subscribe* subPtr = Adafruit_MQTT_Subscribe(&mqtt, "robotCmds/motors"); //possibly redundant
  Adafruit_MQTT_Subscribe* subPtr;
  

  //while subPtr isn't null I think
  while ((subPtr = mqtt.readSubscription(MQTT_READ_TIMEOUT))){
    if (subPtr == &inTopic){ 
      bool cmdRecv; //Never used anywhere else but ok
      cmdRecv = true;
      unsigned long timeLastRecv; //Never used anywhere either
      timeLastRecv = millis();

      // Read and transmit bytes
      // Incoming bytes currently come in the following format:
      // byte
      char* command = (char*) inTopic.lastread; 
      
      //Grabbing the motor values from the byte array
      char motorData = command[0];
      char leftData = motorData >> 4;
      char rightData = motorData & 15; // 15 == 4'b1111

      // Motor powers 
      int rightPower = 1500;
      int leftPower = 1500; 

      // TODO: Determine proper direction
      // Determining power of the left motors
      int powerData = leftData & 7;
      if(leftData >> 3) { // First bit set
        leftPower = (powerData == 4) ? 400 : (powerData == 2) ? 1000 : 1500;
      }
      else { // Otherwise
        leftPower = (powerData == 4) ? 2600 : (powerData == 2) ? 2000 : 1500;
      }

      // Determining the power of the right motors
      powerData = rightData & 7;
      if(rightData >> 3) { // First bit set
        rightPower = (powerData == 4) ? 400 : (powerData == 2) ? 1000 : 1500;
      }
      else { // Otherwise
        rightPower = (powerData == 4) ? 2600 : (powerData == 2) ? 2000 : 1500;
      }


      Serial.println("Left power = " + leftPower);
      Serial.println("Right power = " + rightPower);
      
    }
  }
}
