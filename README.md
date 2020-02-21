# NASA Lunabotics 2020

For any clarifications, suggested changes, etc. please contact:

**Current maintainer:** Dan Shafman (@danshafman on Slack)

## Table of Contents
[1&nbsp;&nbsp;Communications](#communications)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1&nbsp;&nbsp;Server ⟺ Robot](#server-robot)

[2&nbsp;&nbsp;Coming soon](#todo)



<a name="communications"></a>

## 1&nbsp;&nbsp;&nbsp;&nbsp;Communications

<a name="server-robot"></a>

### 1.1&nbsp;&nbsp;&nbsp;&nbsp;Server ⟺ Robot

One Teensy/ESP pair does not have enough power to handle all the functionality of the robot at once, due to the Teensy's limited output capability and processing power. As such, the the robot functionality is handled over four Teensy/ESP pairs, each representing one subset of the robot's functionality. Specifically, this means there is a Teensy/ESP pair for:

 **1. Writing commands to the drivetrain</b>\
 2. Writing commands to the rest of the robot's limbs</b>\
 3. Reading sensor (except LIDAR) data and sending it back to the server
  4. Reading LIDAR data and sending it back to the server **

In this section, Teensy/ESP pair #1 will be referred to as the **Drivetrain ESP**, #2 will be referred to as the **Limb ESP**, #3 will be referred to as the **Sensor ESP**, and #4 will be referred to as the **LIDAR ESP**. These communicate with the server nodes `send_drive_vector` (#1), `send_limb_controls` (#2), and `read_robot_state` (#3,4). A diagram of this can be found at the bottom of this section.

This communication done using the [MQTT](https://www.youtube.com/watch?v=2aHV2Fn0I60) communication protocol, a method of wireless data transfer based on the [publisher/subscriber pattern](https://docs.microsoft.com/en-us/azure/architecture/patterns/publisher-subscriber) (the same pattern that ROS follows). This is implemented with the [Paho MQTT ](https://pypi.org/project/paho-mqtt/) Python library on the server and the [Adafruit MQTT Library](https://learn.adafruit.com/mqtt-adafruit-io-and-you/overview) for Arduino. An example of this can be found on the [TestBot20 repository under "MQTTCommsExample"](https://github.com/nyu-rdt/TestBot20.git).

This protocol is used in autonomy and teleoperated mode identically, since the server is always required to send (or relay) commands to the robot. 

Data is transferred between the server and the robot over several channels (or topics), divided into the main categories `robotState` and `robotCmds`. `robotState` is used to convey information about the state of the robot, from the robot to the server. `robotCmds` is used to send commands from the server to the robot. The subdivisions of these topics are as follows (once again, a diagram of this can be found at the bottom of this section):
___
#### `robotCmds/drive`
Used to send drive commands from the server, specifically from the `send_drive_vector` node on the server, to the Drivetrain ESP. These commands will be used to control the robot's drivetrain. The data is sent in the following format: 

```
{ float robotSpeed,
float offset,
int driveMode }
```

- `float robotSpeed` can take on a value from **-100 to 100**, representing a percentage of the robot's potential speed to write to the controller.

- `float offset` can take on a value from **-100 to 100**, representing the offset between the value that will be written to the left/right-side wheels. This is used for turning. **Note: this parameter is only relevant when** `driveMode` **is equal to 0**, for reasons explained in the next section.

  For an example, given a `robotSpeed` of *80*, an `offset` value of *0* means that the left-side wheels and the right-side wheels will receive the same value (80%), so the robot will go straight. Alternatively, an `offset` of *30* means that the left-side wheels will get a speed of *80%* while the right-side wheels will receive a speed of 80 - 30 = *50%*. This will cause the robot to veer right.  

- `int driveMode` can take on a value from **0 to 3**. Though intuitively a robot should only need a `robotSpeed` and `offset` parameter, just these two fields do not allow for in-place turning, or controlling the front-side and back-side wheels independently (as opposed to the left- and right-side wheels). Thus, the `driveMode` parameter exists so that the Drivetrain ESP

___
#### `robotCmds/limbs`
Used to send drive commands from the server, specifically from the `send_limb_controls` node on the server, to the Limb ESP to actuate the arms. The data is sent in the following format: **work in progress**

___
#### `robotCmds/door`
Used to send drive commands from the server, specifically from the `send_limb_controls` node on the server, to the Limb ESP, to actuate the deposition door. The data is sent in the following format: **work in progress**

___
#### `robotCmds/drum`
Used to send drive commands from the server, specifically from the `send_limb_controls` node on the server, to the Limb ESP, to rotate the digging drum. The data is sent in the following format: **work in progress**

___
#### `robotCmds/linActs`
Used to send drive commands from the server, specifically from the `send_limb_controls` node on the server, to the Limb ESP, to actuate the linear actuators. The data is sent in the following format: **work in progress**

___
#### `robotState/obstacleData`
Used to send an array of data points (generated by the obstacle-detecting LIDARs), representing distances, from the Sensor ESP to the `read_robot_state` node. The data is sent in the following format: **work in progress**

___
#### `robotState/armsData`
Used to send data about the arms' current state from the Sensor ESP to the `read_robot_state` node. This data is obtained by potentiometers on the robot. The data is sent in the following format: **work in progress**

___
#### `robotState/drumData`
Used to send data about the drum's current rate of rotation from the Sensor ESP to the `read_robot_state` node. This data is obtained by Reed switches on the robot. The data is sent in the following format: **work in progress**

___
#### `robotState/doorData`
Used to send data about the deposition door's current state from the Sensor ESP to the `read_robot_state` node. This data is obtained by potentiometers on the robot. The data is sent in the following format: **work in progress**

___
#### `robotState/linActsData`
Used to send data about the linear actuators' current state from the Sensor ESP to the `read_robot_state` node. This data is obtained by potentiometers on the robot. The data is sent in the following format: **work in progress**





<a name="todo"></a>

## ADD TO README (WIP)

- Description of the robot
- Code structure (NUC, robot, GCS)
- Term definitions (like ESP and NUC and server and limbs)
- Link the trainings from Google Drive
