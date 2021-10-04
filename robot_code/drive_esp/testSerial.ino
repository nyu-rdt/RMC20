#include <Servo.h>

Servo frontR;    //front right motor
Servo frontL;    //front left motor
Servo backR;    //back right motor
Servo backL;    //back left motor
Servo diggingScrew; //digging screw motor

/*
 * Motor class to help with determining motor speeds
 * For sprint 1: Possibilities of no, half, and full power
 */
//following values are for Spark Max Motor controller
//comment out if needed
int neutral = 1500;
int startPropForward = 1515;
int fullForward = 2000;
int startPropBackwards = 1485;
int fullBackwards = 1000;

int data = -1;

void setup() {
  Serial.begin(115200);
  frontR.attach(3);
  frontL.attach(5);
  backR.attach(6);
  backL.attach(10);
}

void loop() {
  if (Serial.available() > 0){
    data = Serial.read();
    Serial.print("Recieved Data: ");
    Serial.print(data);
    Serial.print("-");

    if(data == 1){
      Serial.println("Forward");
      allForward();
    }
    else if(data == 2){
      Serial.println("Stop");
      allStop();
    }
    else if(data == 0){
      Serial.println("Backwards");
      allBackwards();
    }
    else if(data == 3){
      Serial.println("Right");
      moveRight();
    }
    else if(data == 4){
      Serial.println("Left");
      moveLeft();
    }
    delay(10);
    data = -1;
  }

}

void moveRight(){
   for(int i=0; i < 51; i++){
    backwards(frontR, i, false);
    forwards(frontL, i, false);
    backwards(backR, i, false);
    forwards(backL, i, false);
    delay(50);
   } 
}

void moveLeft(){
   for(int i=0; i < 51; i++){
    fowards(frontR, i, false);
    backwards(frontL, i, false);
    forwards(backR, i, false);
    backwards(backL, i, false);
    delay(50);
   } 
}

void allStop(){
  stopMotor(frontR, false);
  stopMotor(frontL, false);
  stopMotor(backR, false);
  stopMotor(backL, false);
}

void allBackwards(){
   for(int i=0; i < 51; i++){
    backwards(frontR, i, false);
    backwards(frontL, i, false);
    backwards(backR, i, false);
    backwards(backL, i, false);
    delay(50);
   } 
}

void allForward(){
  for(int i=0; i < 51; i++){
    forward(frontR, i, false);
    forward(frontL, i, false);
    forward(backR, i, false);
    forward(backL, i, false);
    delay(50);
  }
}

void forward(Servo motor, int inputSpeed, bool printMicroseconds){
  /*
   Function takes a Servo class variable and writes a PWM with a certain duration in microseconds.
   The map function is based on the variables set in the global scope
   Motor goes forward
   */
  //maps throttle from 0-100 to corresponding 
  int motorSpeed = map(inputSpeed, 0, 100, startPropForward, fullForward);
  if(printMicroseconds){
    Serial.print(motorSpeed);
  }
  motor.writeMicroseconds(motorSpeed);
}

void backwards(Servo motor, int inputSpeed, bool printMicroseconds){
    /*
   Function takes a Servo class variable and writes a PWM with a certain duration in microseconds.
   The map function is based on the variables set in the global scope.
   Motor goes backwards
   */
  int motorSpeed = map(inputSpeed, 0, 100, startPropBackwards, fullBackwards);
  if(printMicroseconds){
    Serial.print(motorSpeed);
  }
  motor.writeMicroseconds(motorSpeed);
}

void stopMotor(Servo motor, bool printMicroseconds){
    /*
   Function takes a Servo class variable and writes a PWM that corresponds to the controllers neutral.
   */
  if(printMicroseconds){
    Serial.print(neutral);
    }
  motor.writeMicroseconds(neutral);
}
