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

void Lidar_Loop() {

  //measures each lidar pin using the read_lidar function and stores the measurement in dists_initial
  for (int i = 0; i < 4; i++){
    double val = read_lidar(lidar_pins[i]);
    dists_initial[i] = val;
    delayMicroseconds(65); 
    char data[] = 
    Serial.print(String(i) + ": " + String(dists_initial[i]) +",    ");
  }
  Serial.println();
  //waits 100 microseconds before taking a new reading
  delay(1000);

}