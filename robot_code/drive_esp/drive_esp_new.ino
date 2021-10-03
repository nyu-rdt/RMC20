/***************************************************
  Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "Team_16"
#define WLAN_PASS       "lunabots"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "...your AIO username (see https://accounts.adafruit.com)..."
#define AIO_KEY         "...your AIO key..."

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
//Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

//This is an old line from drive_esp
Adafruit_MQTT_Client mqtt(&client, "192.168.1.10", 1883, "motorsTopic", ""); 


/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
//Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/photocell");

// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe onoffbutton(&mqtt, "robotCmds/motors");

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
  mqtt.subscribe(&onoffbutton);
}

uint32_t x=0;

//w: 
//a: 
//s: 
//d: 
void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: ")); //F means format -maybe Justin
      Serial.println((char *)onoffbutton.lastread); //Dereferencing

      // Read and transmit bytes
      // Incoming bytes currently come in the following format:
      // byte
      char* command = (char*) onoffbutton.lastread; 
      
      //Grabbing the motor values from the byte array
      char motorData = command[0];
      char leftData = motorData >> 4;
      char rightData = motorData & 15; // 15 == 4'b1111

      //Why it b left bracket doe
      Serial.println((int)motorData);
      
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

  /* Doesn't pertain to us this sprint
  // Now we can publish stuff!
  Serial.print(F("\nSending photocell val "));
  Serial.print(x);
  Serial.print("...");
  if (! photocell.publish(x++)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  */

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
