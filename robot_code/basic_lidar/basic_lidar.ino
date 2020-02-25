const int lidar_pins[4] = {13, 12, 11, 10};

void setup() {
  //begins serial monitor, sets the pinmode for all lidar pins to input
  Serial.begin(9600);
  for (byte i = 0; i < 4; i++){
    pinMode(INPUT, lidar_pins[i]);
  }
}

//Lidar pins can only be read by varying the voltage to the pin and measuring the duration one at a time 
//due to limitations with the arduino and the pulseIn function
double read_lidar (int i){
  digitalWrite(i,HIGH);
  delayMicroseconds(2);
  digitalWrite(i,LOW);
  double duration = pulseIn(i, HIGH);
  //this converts the duration variable into meters
  return (duration * 171.5)/1000000;
}

void loop() {
  //declares two arrays, each with 4 values, to store the measurement to compare later
  double dists_initial[4];
  double dists_post[4];

  //measures each lidar pin using the read_lidar function and stores the measurement in dists_initial
  for (int i = 0; i < 4; i++){
    double val = read_lidar(lidar_pins[i]);
    dists_initial[i] = val;
    delayMicroseconds(65); 
    Serial.print(String(i) + ": " + String(dists_initial[i]) +",    ");
  }
  Serial.println();
  //waits 100 microseconds before taking a new reading
  delay(1000);

  //does the same as previous loop, however with dists_post
  for (int i = 0; i < 4; i++){
    double val = read_lidar(lidar_pins[i]);
    dists_post[i] = val;
    delayMicroseconds(65);
    Serial.print(String(i) + ": " + String(dists_post[i])+",    ");
  }
  Serial.println();
  //compares the older values to the newer values on the left side to determine if obstacle is on left or right
  for (int i = 0; i < 2; i++) {
    //Serial.println("init: " + String(dists_initial[i]) + "past: " + String(dists_post[i]));
    if (abs(dists_initial[i] - dists_post[i]) > 0.10){
      Serial.println("Obstacle on left");
    }
  } 

  //compares the older values to the newer values on the right side to determine if obstacle is on left or right
  for (int i = 2; i < 4; i++){
  //Serial.println("init: " + String(dists_initial[i]) + "past: " + String(dists_post[i]));
    if (abs(dists_initial[i] - dists_post[i]) > 0.10){
      Serial.println("Obstacle on right");
    }
  }
  delay(100);
}
