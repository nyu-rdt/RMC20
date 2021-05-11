#include <Arduino.h>
#include<Servo.h>
// command vars
const int recv_timeout = 100;
long lastcmd = 0;


// arm pins
const int arm_front_pin = 14;
const int arm_back_pin = 15;
Servo arm_front;
Servo arm_back;


// door pin
const int door_pin_1 = 4;
const int door_pin_2 = 3;
Servo door1;
Servo door2;
int olddoorval = 0;


// drum pins
const int LEFT_PIN = 5;
const int RIGHT_PIN  = 6;
const int LEFT_SENSOR = 16;
const int RIGHT_SENSOR = 2;
// drum variables
int val =  0; //0 - 100
bool direction = true; //true for clockwise (digging)

Servo left_wheel;
Servo right_wheel;


void setup() {
	Serial2.begin(9600);
	Serial1.begin(115200);
  Serial.begin(115200);
  left_wheel.attach(LEFT_PIN);
  right_wheel.attach(RIGHT_PIN);
  arm_front.attach(arm_front_pin);
  arm_back.attach(arm_back_pin);
  door1.attach(door_pin_1);
  door2.attach(door_pin_2);
  pinMode(LEFT_SENSOR, INPUT_PULLDOWN);
  pinMode(RIGHT_SENSOR, INPUT_PULLDOWN);
}

// mapping values for actually turning the wheel
int getForward(int val)
{
  return map(val,-100,100,1000,2000);
}
int getBackward(int val)
{
  return map(-val,-100,100,1000,2000);
}

// wheels running function
void run_cw()
{
  left_wheel.writeMicroseconds(getForward(val));
  right_wheel.writeMicroseconds(getBackward(val));
}
void run_ccw()
{
  left_wheel.writeMicroseconds(getBackward(val));
  right_wheel.writeMicroseconds(getForward(val));
}

void run_arms(int armval) {
  int forward = map(armval,-100,100,1000,2000);
  int backward = map(-armval,-100,100,1000,2000);
  arm_front.writeMicroseconds(forward);
  arm_back.writeMicroseconds(forward);
}

void run_drum(int drumval) {
  if (val > 0) {
    direction = true;
  }
  else {
    direction = false;
  }
  if (drumval != val) {
    val = drumval;
  }
}

void run_door(int doorval) {
	if (doorval != olddoorval) {
		if (doorval < 1) {
			door1.writeMicroseconds(0); //close
			door2.writeMicroseconds(0);
		}
		else {
			door1.writeMicroseconds(50); //open
			door2.writeMicroseconds(90);
		}
	}
}

	void run_linear(int linval) {
		linval = linval * 10; // make values readable
		Serial2.println(String(linval));
	}

	// looping function
	void loop()
	{
		if(Serial1.available() > 0) {
			int first = Serial1.read();
			if (first == 255) {
      while (!(Serial1.available())) {}
      int second = Serial1.read(); // door
      run_door(second);
      while (!(Serial1.available())) {}
      int third = Serial1.read(); // linear actuator
      run_linear(third);
      while (!(Serial1.available())) {}
      int fourth = Serial1.read(); // arm
      run_arms(fourth);
      while (!(Serial1.available())) {}
      int fifth = Serial1.read(); // drum
      run_drum(fifth);
      Serial1.write(255);
    }
    else if (millis() - lastcmd >= recv_timeout) {
      run_linear(0);
      run_arms(0);
      run_drum(0);
    }
  }
  if (direction) {
    run_cw();
  }
  else {
    run_ccw();
  }
}
