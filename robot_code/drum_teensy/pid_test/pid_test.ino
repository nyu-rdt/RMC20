#include "PID.h"
PID test_pid(1,0.1,0.1,0.7,0.3);
void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println((double)test_pid.update(1,1));
  Serial.println((double)test_pid.update(-1,-1));
  delay(100);
}
