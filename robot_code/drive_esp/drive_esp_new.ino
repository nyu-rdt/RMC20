/*
 * Code for the radio in the locomotion subsystem, particularly for a NodeMCU with an ESP8266 
 * module. Reads commands from the MQTT network and relays them to the locomotion subsystem
 * controller.
*/

//Calling speicifc Wifi routine to connect to the network
#include <ESP8266WiFi.h>

// WiFi & Server Constants
#define WIFI_SSID "Team_16"
#define WIFI_PASS "lunabots"
#define WIFI_CHANNEL 1
#define SERVER_ADDR "192.168.1.10"
#define SERVER_PORT 1883
#define TOPIC_NAME_IN "robotCmds/motors"
#define TOPIC_NAME_OUT "robotState/motorPing"

// Create MQTT client on the ESP
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, SERVER_ADDR, SERVER_PORT, "motorTopic", "");
Adafruit_MQTT_Subscribe inTopic = Adafruit_MQTT_Subscribe(&mqtt, TOPIC_NAME_IN);
Adafruit_MQTT_Publish pingTopic = Adafruit_MQTT_Publish(&mqtt, TOPIC_NAME_OUT);

//Motor pins
int motor1Pin = 1;
int motor2Pin = 2;
int motor3Pin = 3;
int motor4Pin = 4;

void setup()
{
  Serial.begin(115200);
  Serial.println();

  //Setting pinModes for the motor Pins
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(motor3Pin, OUTPUT);
  pinMode(motor4Pin, OUTPUT);

  //MQTT STUFF
  //Connecting to Wifi network
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);

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
  analogWrite(motor1Pin)
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
      cmdRecv = true;
      timeLastRecv = millis();

      // Read and transmit bytes
      // Incoming bytes currently come in the following format:
      // byte byte byte byte
      // motor1 motor2 motor3 motor4
      char* command = (char*) inTopic.lastread; 
      
      //Grabbing the motor values from the byte array
      char motor1 = command[0];
      char motor2 = command[1];
      char motor3 = command[2];
      char motor4 = command[3];
      
      //
    }
  }
}