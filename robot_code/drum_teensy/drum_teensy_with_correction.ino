//Drum and actuators teensy code
#include<Servo.h>


// arm pins
const int arm_front_pin = 14;
const int arm_back_pin = 15;
Servo arm_front;
Servo arm_back;


// door pin
const int door_pin_1 = 4; // NEED TO FILL IN
const int door_pin_2 = 3; // NEED TO FILL IN
Servo door1;
Servo door2;

//hard coded value
int olddoorval = 0; 


// drum pins digging wheel
const int LEFT_PIN = 5; 
const int RIGHT_PIN  = 6;
const int LEFT_SENSOR = 20;
const int RIGHT_SENSOR = 21;

// drum variables
const float weightdiff = 0.7; 
const float weightinterval = 0.3;
const float circum = 15.75*3.141592653;
const int adjustmenttime = 300; // in milliseconds
const int magnum = 4; // magnets at 0, 90, 180, 270
const int recv_timeout = 100;
long lastcmd = 0;
int val =  0; //0 - 100
int left_interval= 5000;
int right_interval = 5000;
int left_pastread = 0;
int right_pastread = 0;
int left_speed = val;
int right_speed = val;
int difference = 0; // positive if left is ahead.
bool setleft = false;
bool setright = false;
bool direction = true; //true for clockwise (digging)

Servo left_wheel;
Servo right_wheel;

void setup() {
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

// lag of the magnets
void set_difference() {
  difference = left_pastread - right_pastread;
  setleft = false;
  setright = false;
}

void set_intervals() {
  int current = millis();
  // Only set past_read and interval if larger than .8 of previous interval - don't set during streak of LOW or false LOW in the middle
  if (digitalRead(LEFT_SENSOR) == HIGH && (((float) (current - left_pastread) / left_interval ) > (0.8 / magnum)))  {
    // Only set interval when it's below 1.7x of the previous interval - i.e we didn't miss an interval
    if (((float) (current - left_pastread) / left_interval ) < (1.7 / magnum)) { // Magnum is failsafe for intervals
      left_interval = (current - left_pastread) * magnum;
      Serial.print("Left Update: ");
      Serial.println(((float) left_interval) / 100);
    }
    // Record the past_read anyway to set the next interval
    left_pastread = current;
    setleft = true;
  }

  if (digitalRead(RIGHT_SENSOR) == HIGH && (((float) (current - right_pastread) / right_interval ) > (0.8 / magnum))) {
    if (((float) (current - right_pastread) / right_interval ) < (1.7 / magnum)) { 
      right_interval = (current - right_pastread) * magnum; 
      Serial.print("Right Update: ");
      Serial.println(((float) right_interval / 100));
    }
    right_pastread = current;
    setright = true;
  }
  // Left and right difference after pastread value updated
  if (setright && setleft) {
    set_difference();
  }
}

// reduce errors for both speed and positioning of magnets and adjust weights of each error
// we could make a pid controller, but it may be too much for the teensy (need to keep lots of points in memory)
// it would be nice to make a switching pid controller based on both of these errors
void setspeeds() {
	// adjust based on difference in magnet positioning
  int avg_interval = (left_interval + right_interval) / 2;
  float angle = difference / avg_interval;
  float speed_decrease = (adjustmenttime * angle) * weightdiff;
  if (speed_decrease < 0) {
    right_speed = right_speed + speed_decrease;
  }
  else {
    left_speed = left_speed - speed_decrease;
  }

  // adjust based on difference in speed
  if (left_interval > right_interval) {
    float adjustmentval = (right_interval - left_interval) * weightinterval;
    right_speed = right_speed + adjustmentval;
  }
  else {
    float adjustmentval = (left_interval - right_interval) * weightinterval;
    left_speed = left_speed + adjustmentval;
  }
}



// wheels running function
void run_cw()
{
  setspeeds();
  left_wheel.writeMicroseconds(getForward(left_speed));
  right_wheel.writeMicroseconds(getBackward(right_speed));
}
void run_ccw()
{
  setspeeds();
  left_wheel.writeMicroseconds(getBackward(left_speed));
  right_wheel.writeMicroseconds(getForward(right_speed));
}

// //Move the arms of the robot
// void run_arms(int armval) {
//   int forward = map(armval,-100,100,1000,2000);
//   int backward = map(-armval,-100,100,1000,2000);
//   arm_front.writeMicroseconds(forward);
//   arm_back.writeMicroseconds(backward);
// }

void run_drum(int drumval) {
  if (val > 0) {
    direction = true;
  }
  else {
    direction = false;
  }
  if (drumval != val) {
    val = drumval;
    left_interval = val * 100; // baseline guess for the first interval
    right_interval = val * 100; // baseline guess for the first interval
    left_speed = val;
    right_speed = val;
  }
}

void run_door(int doorval) {
	if (doorval != olddoorval) {
		if (doorval < 1) {
			door1.writeMicroseconds(1000); //close
			door2.writeMicroseconds(1000);
		}
		else {
    door1.writeMicroseconds(2000); //open
    door2.writeMicroseconds(2000);
		}
	}
}

  //Run linear actuators
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
        //run_arms(fourth);
        while (!(Serial1.available())) {}
        int fifth = Serial1.read(); // drum
        run_drum(fifth);
        Serial1.write(255);
    }
    else if (millis() - lastcmd >= recv_timeout) {
      run_linear(0);
      //run_arms(100);
      run_drum(0);
    }
  }
  if (direction) {
    set_intervals(); // uncomment to start adjusting motor speeds dynamically
    run_cw();
  }
  else {
    set_intervals();
    run_ccw();
  }
}
