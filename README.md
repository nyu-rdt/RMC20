﻿
# NASA Lunabotics 2020
NYU Robotic Design Team's full robot code

## Table of Contents
1) Communications</b>
1A) Server ⟺ Robot

## 1) Communications

### 1A) Server ⟺ Robot

One Teensy/ESP pair does not have enough power to handle all the functionality of the robot at once, due to the Teensy's limited output capability and processing power. As such, the the robot functionality is handled over three Teensy/ESP pairs, each representing one subset of the robot's functionality. Specifically, this means there is a Teensy/ESP pair for:

 **1. Writing commands to the drivetrain</b>
 2. Writing commands to the rest of the robot's limbs</b>
 3. Reading sensor data and sending it back to the server**

In this section, ESP #1 will be referred to as the **Drivetrain ESP**, #2 will be referred to as the **Limb ESP**, and #3 will be referred to as the **Sensor ESP**. These communicate with the server nodes `send_drive_vector`, `send_limb_controls`, and `read_robot_state`, respectively. A diagram of this can be found at the bottom of this section.

This communication done using the [MQTT](https://www.youtube.com/watch?v=2aHV2Fn0I60) communication protocol, a method of wireless data transfer based on the [publisher/subscriber pattern](https://docs.microsoft.com/en-us/azure/architecture/patterns/publisher-subscriber). This is implemented with the [Paho MQTT ](https://pypi.org/project/paho-mqtt/) Python library on the server and the [Adafruit MQTT Library](https://learn.adafruit.com/mqtt-adafruit-io-and-you/overview) for Arduino. An example of this can be found on the [TestBot20 repository under "MQTTCommsExample"](https://github.com/nyu-rdt/TestBot20.git).

This protocol is used in autonomy and teleoperated mode identically, since the server is always required to send (or relay) commands to the robot. 

Data is transferred between the server and the robot over several channels (or topics), divided into the main categories `robotState` and `robotCmds`. `robotState` is used to convey information about the state of the robot, from the robot to the server. `robotCmds` is used to send commands from the server to the robot. The subdivisions of these topics are as follows (once again, a diagram of this can be found at the bottom of this section):
___
#### `robotCmds/drive`
Used to send drive commands from the server, specifically from the `send_drive_vector` node on the server, to the Drivetrain ESP. These commands will be used to control the robot's drivetrain. The data is sent in the following format: **work in progress**

___
#### `robotCmds/arms`
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

## ADD TO README (WIP)
- Description of the robot
- Code structure (NUC, robot, GCS)
- Term definitions (like ESP and NUC and server and limbs)
- Link the trainings from Google Drive
