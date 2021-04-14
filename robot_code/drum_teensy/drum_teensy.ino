#include<Servo.h>
#include "PID.h"
const int LEFT_PIN = 16;
const int RIGHT_PIN  = 20;
const int LEFT_SENSOR = 6;
const int RIGHT_SENSOR = 5;
int val =  10; //0 - 100
int left_interval = 5000;
int right_interval = 5000;
int left_pastread = 0;
int right_pastread = 0;
int left_speed = val;
int right_speed = val;
bool setleft = false;
bool setright = false;
int difference = 0; // positive if left is ahead.
int magnum = 4; // magnets at 0, 90, 180, 270
int adjustmenttime = 300; // in milliseconds
float weightdiff = 0.7;
float weightinterval = 0.3;
float circum = 15.75 * 3.141592653;

float Kp = 1;
float Ki = 0.01;
float Kd = 0.01;
PID drum_pid(1,0.1,0.1,0.7,0.3);
Servo left_wheel;
Servo right_wheel;
void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  left_wheel.attach(LEFT_PIN);
  right_wheel.attach(RIGHT_PIN);
  pinMode(LEFT_SENSOR, INPUT_PULLUP);
  pinMode(RIGHT_SENSOR, INPUT_PULLUP);
}

// mapping values for actually turning the wheel
int getForward(int val)
{
  return map(val, -100, 100, 1000, 2000);
}
int getBackward(int val)
{
  return map(-val, -100, 100, 1000, 2000);
}

// lag of the magnets
void set_difference() {
  difference = left_pastread - right_pastread;
}

void set_intervals() {
  int current = millis();

  // Only set past_read and interval if larger than .8 of previous interval - don't set during streak of LOW or false LOW in the middle
  if (digitalRead(LEFT_SENSOR) == HIGH && (((float) (current - left_pastread) / left_interval ) > (0.8 / magnum)))  {
    // Only set interval when it's below 1.7x of the previous interval - i.e we didn't miss an interval
    if (((float) (current - left_pastread) / left_interval ) < (1.7 / magnum)) {
      left_interval = (current - left_pastread) * magnum;
      Serial.print("Left Update: ");
      // Serial.println(((float) left_interval) / 100);
    }
    // Record the past_read anyway to set the next interval
    left_pastread = current;
    setleft = true;
  }

  if (digitalRead(RIGHT_SENSOR) == HIGH && (((float) (current - right_pastread) / right_interval ) > (0.8 / magnum))) {
    if (((float) (current - right_pastread) / right_interval ) < (1.7 / magnum)) {
      right_interval = (current - right_pastread) * magnum;
      Serial.print("Right Update: ");
      // Serial.println(((float) right_interval / 100));
    }
    right_pastread = current;
    setright = true;
  }
  if (setright && setleft) {
    Serial.println("Updated");
    set_difference();
    setleft = false;
    setright = false;
  }
}

// reduce errors for both speed and positioning of magnets and adjust weights of each error
// we could make a pid controller, but it may be too much for the teensy (need to keep lots of points in memory)
// it would be nice to make a switching pid controller based on both of these errors
void setspeeds() {
  // adjust based on difference in magnet positioning
  float speed_decrease = drum_pid.update(difference, (right_interval - left_interval));
  if (speed_decrease > 0) {
    left_speed -= (int) speed_decrease;
  }
  else {
    right_speed += (int) speed_decrease;
  }
  /*
  Serial.print("speed_decrease: ");
  Serial.println(speed_decrease);
  Serial.print("pos_difference: ");
  Serial.println(difference);
  Serial.println("speed_difference ");
  Serial.println(right_interval - left_interval);
  */
}

// wheels running function
void run_cw(int val)
{
  setspeeds();
  left_wheel.writeMicroseconds(getForward(left_speed));
  right_wheel.writeMicroseconds(getBackward(right_speed));
}
void run_ccw(int val)
{
  setspeeds();
  left_wheel.writeMicroseconds(getBackward(left_speed));
  right_wheel.writeMicroseconds(getForward(right_speed));
}


// looping function
void loop()
{
  if (Serial1.available() > 0) {
    val = Serial1.read();
    if (val != 0) {
      left_interval = val * 100; // this is baseline guess for first interval that will be adjusted later
      right_interval = val * 100; // this is baseline guess for first interval that will be adjusted later
    }
  }
   //set_intervals(); // uncomment to start adjusting motor speeds dynamically
  run_cw(val);
  Serial.println(digitalRead(RIGHT_SENSOR));
  Serial.println(digitalRead(LEFT_SENSOR));
}
