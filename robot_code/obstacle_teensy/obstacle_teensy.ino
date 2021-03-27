/*
 * obstacle_teensy.ino
 *
 * Code for parsing the data from 360 lidar. The interested scan region is determined by a middle angle and an angle range.
 * The middle angle is updated to be the angle at the minimal distance within the range.
 *
 * Within the interested scan region, the code calculate the actual distance to the ground in the plane of lidar.
 * Compare the calculated actual distance with the minimal distance, which should be the same if both are detecting ground.
 * If the difference is bigger than THR_OBSTACLE, record the result.
 * A variable is used to count the number of hill/hole on both sides. At every new round of the lidar, the code will
 * return the strategy to avoid obstacles. (0 for turning left; 1 for no obstacles; 2 for turning right)
 *
 * See README for descriptions on the format of the command messages.
 *
 * TODO:
 * - detemine parameter values in the code (Search TODO in the code)
 */


#include "RPLidar.h"

// You need to create an driver instance
RPLidar lidar;


#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal

#define ROT_SPEED 255   // The rotation speed of the lidar (Max speed: 255)

// TODO: require test
#define THR_OBSTACLE 30   // The threshold for detecting a hole (unit: mm).
                          // Note: this distance should be estimated with considerations of the installation slope of the lidar

// angles (unit: degree)
// TODO: determine the range of interested angles
float angleRange = 180;                   // The range of the interested angle relatively to the mid angle
// TODO: determine the init middl angle
float angleMid = 10;                     // The middle angle pointing to the ground with shortest distance
float minDistance = 0;                   // To compare the standard distance to the ground
// float angleAtMinDist = 0;                // Use to init the mid angle
// bool init = true;                        // If the angleMid and minDistance need to be inited
int count = 0;                           // To count the left and right obstacles; minus 1 on the left, add 1 on the right

void setup() {
    // bind the RPLIDAR driver to the arduino hardware serial
    lidar.begin(Serial2);   // TODO: not sure which serial to use
    Serial.begin(115200);   // Serial Monitor
    Serial1.begin(115200);  // Connection to ESP8266

    // set pin modes
    pinMode(RPLIDAR_MOTOR, OUTPUT);
}

// Calculate the distance using angle
// angle should be relative value to the mid angle
float calculateDistance(float angle, float distance) {
  return cos(angle / 180 * 3.1415926) * distance;
}

void loop() {
    if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    // offset the angle by the middle angle
    float angle    = lidar.getCurrentPoint().angle; //angle value in degree
    // bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    // byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    // debugging codes
    //Serial.print("Count: ");
    //Serial.println(count);

    //perform data processing here...
    if (lidar.getCurrentPoint().startBit) {

      // a new scan, send the previous result
      // set a thr of 5 in case of noise
      if (abs(count) < 5) {
        Serial1.write(1);
      } else if (count > 0) {
        Serial1.write(0);          // obstacles on right, turn left
      } else {
        Serial1.write(2);          // obstacles on left, turn right
      }
      // reset count variable
      count = 0;

    } else if (abs(angle) < angleRange) {   // angle within the interested range
      if ( distance > 0 &&
            distance < minDistance ) {
        // calibrate the mid angle and min distance
        minDistance = distance;
        angleMid = angle;
        Serial.print("min distance: ");
        Serial.print(minDistance);
        Serial.print(" the angle:");
        Serial.println(angle);
      }

      float actualDistance = calculateDistance(angle, distance);
      if (abs(actualDistance - minDistance) > THR_OBSTACLE) {     // if detecting a hill or hole
        Serial.print("Obstacle detect at ");
        Serial.print(angle);
        Serial.print(" actual dist diff: ");
        Serial.println(actualDistance - minDistance);
        // if angle smaller than 0, assuming obstacles on the left and vice versa
        if (angle < 0) {
          count -= 1;
        } else if (angle > 0) {
          count += 1;
        }
      }

      // for debug purpose only
      //no lol you can't % float
      /*if (angle % 10 < 0.5) {
        Serial.print("Distance at ");
        Serial.print(angle);
        Seiral.print(" distance: ");
        Serial.println(distances);
      }
      */
    }

  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();

       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, ROT_SPEED);
       delay(1000);
    }
  }
}
