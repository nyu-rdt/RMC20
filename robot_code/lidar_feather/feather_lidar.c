const int lidar_pins[4] = {13, 12, 11, 10};

void setup() {
  //begins serial monitor, sets the pinmode for all lidar pins to input
  Serial.begin(9600);
  Serial1.begin(9600);
  for (byte i = 0; i < 4; i++){
    pinMode(INPUT, lidar_pins[i]);
  }
}

//Lidar pins can only be read by varying the voltage to the pin and measuring the duration one at a time 
//due to limitations with the arduino and the pulseIn function
int read_lidar (int i){
  digitalWrite(i,HIGH);
  delayMicroseconds(2);
  digitalWrite(i,LOW);
  double duration = pulseIn(i, HIGH);
  //this converts the duration variable into decimeter
  int ret = (int)((duration * 171.5) / 1000);
  return ret;
}

void loop() {
  //declares array, with 4 values, to store the measurement 
  char dists[4];

  //measures each lidar pin using the read_lidar function and stores the measurement in dists_initial
  for (int i = 0; i < 4; i++){
    int val = read_lidar(lidar_pins[i]);
    //the integer value is stored as an char (ASCII), however the int cannot be greater 256
    if (val > 255) {
	    val = 255;
	  }
    char temp = val;
    dists[i] = temp;
    delayMicroseconds(65); 
    //for testing purposes:
    //Serial.print(String(i) + ": " + String(dists[i]) +",    ");
  }
  //Serial.println();
  //waits 100 microseconds before taking a new reading
  Serial1.write(dists,4);
}
