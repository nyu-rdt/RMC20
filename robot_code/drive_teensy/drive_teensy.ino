/*
 * drive_teensy.ino
 * 
 * Code for the controller in the locomotion subsystem, particularly the Teensyduino. Takes in
 * a robot speed and offset byte and writes the command to the motors. Also handles special
 * drive modes such as turning in place and changing the digging drum elevation.
 * 
 * See README for descriptions on the format of the command messages.
 * 
 * TODO:
 * - Implement handlers for drive modes
 */

#include <Servo.h>

#define TEST_MOTOR_SPEED 70
#define CMD_RECV_TIMEOUT 100

// Wheels 
Servo Brown;
Servo Pink;
Servo Blue;
Servo Orange;

// Arms
Servo FrontArm;
Servo BackArm;

// Time the last command was received
long lastCmdTime;

void setup(){
  Serial1.begin(115200);  // Connection to ESP8266
  Serial.begin(115200);   // Serial Monitor

  // Attach motors and arms
  Brown.attach(3);
  Pink.attach(4);
  Blue.attach(6);
  Orange.attach(5);
  FrontArm.attach(9);
  BackArm.attach(10);
  // Hardcode function calling
  /*  forward();
   *  backward();
   *  turnLeft();
   *  turnRight();
   *  armsUp();
   *  armsDown();
   */
}

/*  reset()
 *   This function resets all motor speeds to neutral(not moving)
 */
void reset() {
  Brown.writeMicroseconds(1500);
  Pink.writeMicroseconds(1500);
  Blue.writeMicroseconds(1500);
  Orange.writeMicroseconds(1500);
  FrontArm.writeMicroseconds(1500);
  BackArm.writeMicroseconds(1500);
}

/*  forward()
 *   This function spins all motors so robot moves in the forward direction
 */
void forward(int val, int offset) {
  float valForward = map(val,0,200,1000,2000);
 // float offset = map(turn,0,200,1000,2000);
  float valBackward = map(val,0,200,2000,1000);
  int i;
  Serial.println("test");
  Serial.println(valForward);
  Serial.println(valBackward);
  if(offset < 100) {
  float leftRatio = (offset/200);
  float rightRatio = (offset)/200;

  Brown.writeMicroseconds(valForward);
  Pink.writeMicroseconds(valForward);
  Blue.writeMicroseconds(valBackward*leftRatio);
  Orange.writeMicroseconds(valBackward*leftRatio);
  }

  if(offset > 100) {
  float leftRatio = (offset/200);
  float rightRatio = (offset)/200;

  Brown.writeMicroseconds(valForward*leftRatio);
  Pink.writeMicroseconds(valForward*leftRatio);
  Blue.writeMicroseconds(valBackward);
  Orange.writeMicroseconds(valBackward);
  }
  if (offset == 100){

  float leftRatio = (offset/200);
  float rightRatio = (offset)/200;

  Brown.writeMicroseconds(valForward);
  Pink.writeMicroseconds(valForward);
  Blue.writeMicroseconds(valBackward);
  Orange.writeMicroseconds(valBackward);
    }
  

}
/*  backward()
 *   This function spins all motors so robot moves in the backward direction
 */
void backward(int val) {
  float valForward = map(val,0,200,1000,2000);
  float valBackward = map(val,0,200,2000,1000);
  Serial.println("test");
  Serial.println(valForward);
  Serial.println(valBackward);
  Brown.writeMicroseconds(valBackward);
  Pink.writeMicroseconds(valBackward);
  Blue.writeMicroseconds(valForward);
  Orange.writeMicroseconds(valForward);
}
/*  turnLeft()
 *   This function spins all motors so robot turns Left
 */
void turnLeft(int val) {
  int valForward = map(val,0,200,1000,2000);
  int valBackward = map(val,0,200,2000,1000);
  Brown.writeMicroseconds(valBackward);
  Pink.writeMicroseconds(valBackward);
  Blue.writeMicroseconds(valBackward);
  Orange.writeMicroseconds(valBackward);
}
/*  turnRight()
 *   This function spins all motors so robot turns Right
 */
void turnRight(int val) {
  int valForward = map(val,0,200,1000,2000);
  int valBackward = map(val,0,200,2000,1000);
  Brown.writeMicroseconds(valForward);
  Pink.writeMicroseconds(valForward);
  Blue.writeMicroseconds(valForward);
  Orange.writeMicroseconds(valForward);
}
/*  armsUp()
 *   This function spins falcon motors so robot's arm goes up
 *
 *   Update -
 *      Wheels should turn as the arms are going up
 *      - Tested(In progress)
 */
void armsUp() {
  int valForward = map(TEST_MOTOR_SPEED,-100,100,1000,2000);
  int valBackward = map(-TEST_MOTOR_SPEED,-100,100,1000,2000);
  Brown.writeMicroseconds(valForward);
  Pink.writeMicroseconds(valBackward);
  Blue.writeMicroseconds(valForward);
  Orange.writeMicroseconds(valBackward);
  FrontArm.writeMicroseconds(valBackward);
  BackArm.writeMicroseconds(valBackward);
}
/*  armsDown()
 *   This function spins all motors so robot turns Right
 *
 *   Update -
 *      Wheels should turn as the arms are going down
 *      - Tested(In progress)
 */
void armsDown() {
  int valForward = map(TEST_MOTOR_SPEED,-100,100,1000,2000);
  int valBackward = map(-TEST_MOTOR_SPEED,-100,100,1000,2000);
  Brown.writeMicroseconds(valBackward);
  Pink.writeMicroseconds(valForward);
  Blue.writeMicroseconds(valBackward);
  Orange.writeMicroseconds(valForward);
  FrontArm.writeMicroseconds(valForward);
  BackArm.writeMicroseconds(valForward);
}
void brown() {
  int valForward = map(TEST_MOTOR_SPEED, -100,100,1000,2000);
  Brown.writeMicroseconds(valForward);
  }
void pink(){
  int valForward = map(TEST_MOTOR_SPEED,-100,100,1000,2000);
  Pink.writeMicroseconds(valForward);
  }
void orange(){
  int valBackward = map(-TEST_MOTOR_SPEED,-100,100,1000,2000);
  Orange.writeMicroseconds(valBackward);
  }
void blue(){
  int valBackward = map(-TEST_MOTOR_SPEED,-100,100,1000,2000);
  Blue.writeMicroseconds(valBackward);
  }
/*  Incoming byte
 *   0 - Wheels(0) / Wheels(1)                      Toggle
 *   1 - Arms(0) / Arms(1)                          Toggle
 *   2 - Wheels Backward(0) / Wheels Forward(1)
 *   3 - Arms Down(0) / Arms Up(1)
 *   4 -
 *   5 -
 *   6 -
 *   7 -
 */
void loop() {
  if (Serial1.available() > 0) {
    lastCmdTime = millis();
    int inByte = Serial1.read();
//    Serial.println("received");
//    Serial.println(inByte);
    if (inByte == 255 ){ //synchro byte
      while(!(Serial1.available())){}
      int inByte2 = Serial1.read();
      while(!(Serial1.available())){}
      int inByte3= Serial1.read();
      forward(inByte2, inByte3);

      // read from Serial1, write to Serial, might be wrong
      Serial.write(255); // ackowledgement byte
      // don't need loop because this is in loop()
    }
    else if (millis() - lastCmdTime >= CMD_RECV_TIMEOUT) {
      forward(100, 100);
    }      
  }

}
