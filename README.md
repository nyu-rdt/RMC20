<p align="center">
  <img src="/docs/luna_logo.jpg?raw=true" alt="Lunabotics Logo"/>
</p>

# NYU Robotic Design Team - 2020

Full robot code for NASA Lunabotics 2020 competition. For any clarifications, suggested changes, etc. please contact:

**Current maintainer:** Dan Shafman (@danshafman on Slack)

## Table of Contents
[2&nbsp;&nbsp;Communications](#communications)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1&nbsp;&nbsp;Server ⟺ Robot](#server-robot)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2&nbsp;&nbsp;GCS ⟺ Server](#gcs-server)

[3&nbsp;&nbsp;Additional Resources](#additional-resources)

[4&nbsp;&nbsp;Coming soon](#todo)

<br />

<br />

<br />

<a name="introduction"></a>

## 1&nbsp;&nbsp;&nbsp;&nbsp;Introduction

In this section we will be introducing the NASA Lunabotics 2020 competition and the purpose, design, and operation of our robot. We will also be introducing some technical terms and vocabulary that we use throughout this README.



<br />

<br />

<br />

<a name="communications"></a>

## 2&nbsp;&nbsp;&nbsp;&nbsp;Communications

<a name="server-robot"></a>

### 2.1&nbsp;&nbsp;&nbsp;&nbsp;Server ⟺ Robot

One Teensy/ESP pair does not have enough power to handle all the functionality of the robot at once, due to the Teensy's limited output capability and processing power. As such, the the robot functionality is handled over four Teensy/ESP pairs, each representing one subset of the robot's functionality. Specifically, this means there is a Teensy/ESP pair for:

 **1. Writing commands to the drivetrain</b>\
 2. Writing commands to the rest of the robot's limbs</b>\
 3. Reading sensor (except LIDAR) data and sending it back to the server</b>\
  4. Reading LIDAR data and sending it back to the server**

In this section, Teensy/ESP pair #1 will be referred to as the **Drivetrain ESP**, #2 will be referred to as the **Limb ESP**, #3 will be referred to as the **Sensor ESP**, and #4 will be referred to as the **LIDAR ESP**. These communicate with the server nodes `send_drive_vector` (#1), `send_limb_controls` (#2), and `read_robot_state` (#3,4). A diagram of this can be found at the bottom of this section.

This communication is done using the [MQTT](https://www.youtube.com/watch?v=2aHV2Fn0I60) communication protocol, a method of wireless data transfer based on the [publisher/subscriber pattern](https://docs.microsoft.com/en-us/azure/architecture/patterns/publisher-subscriber) (the same pattern that ROS follows). This is implemented with the [Paho MQTT ](https://pypi.org/project/paho-mqtt/) Python library on the server and the [Adafruit MQTT Library](https://learn.adafruit.com/mqtt-adafruit-io-and-you/overview) for Arduino. An example of this can be found on the [TestBot20 repository under "MQTTCommsExample"](https://github.com/nyu-rdt/TestBot20.git).

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

- `float robotSpeed` can take on a value from **-100 to 100**, representing a percentage of the robot's potential speed to write to the motor controllers.

- `float offset` can take on a value from **-100 to 100**, representing the offset between the value that will be written to the left/right-side wheels. This is used for turning. **Note: this parameter is only relevant when** `driveMode` **is equal to 0**, for reasons explained in the next section.

  For an example, given a `robotSpeed` of *80*, an `offset` value of *0* means that the left-side wheels and the right-side wheels will receive the same value (80%), so the robot will go straight. Alternatively, an `offset` of *30* means that the left-side wheels will get a speed of *80%* while the right-side wheels will receive a speed of 80 - 30 = *50%*. This will cause the robot to veer right.  

- `int driveMode` can take on a value from **0 to 3**. Though intuitively a robot should only need a `robotSpeed` and `offset` parameter, just these two fields do not allow for in-place turning, or controlling the front-side and back-side wheels independently (as opposed to the left- and right-side wheels). Thus, the `driveMode` parameter exists so that the Drivetrain ESP can interpret its drive commands different ways in different contexts (drive modes).
  - **Drive mode 0** is reserved for ESTOP. In this mode, `robotSpeed` and `offset` parameters are ignored and a speed of 0 is written to all controllers.
  - **Drive mode 1** is the "forward-driving" mode of the robot, when it is attempting to move itself such as in **ConOps states 6 and 16**. In this mode, the left-side and right-side wheels are treated as two independent units, each being written `robotSpeed` with a turning offset specified by `offset`.
  - **Drive mode 2** is the "changing digging drum elevation" mode of the robot, such as in **ConOps states 8 and 11**. In this mode, the robot's arms are attempting to raise or lower the digging drum, and the wheels must concurrently turn at a speed to match. The front-side and back-side wheels are treated as two independent units, each being written `robotSpeed`. No offset is necessary for this mode, since at no point in the ConOps are the arms rotating at different speeds.
  - **Drive mode 3** is the "turning in place" mode of the robot, where it is rotating in-place around its own center axis such as in **ConOps states 15 and 17**. Like drive mode 1, the left- and right-side wheels are treated as two independent units, each being written `robotSpeed`. No offset is necessary for this mode and its value can be ignored.

___
#### `robotCmds/limbs`
Used to send limb-controlling commands from the server, specifically from the `send_limb_controls` node on the server, to the Limb ESP. These commands control ALL of the robot's limbs, except the drivetrain. The data is sent in the following format: 

```
{ int drum
int arms
int linActs
bool door }
```

- `int drum` can take on a value from **-100 to 100**, representing a percentage of the digging drum's potential speed to write to its motor controller.
- `int arms` can take on a value from **-100 to 100**, representing a percentage of the arms' potential speed to write to their motor controllers. During the lowering and lifting drum autonomy states, such as **ConOps states 8 and 11**, the arms need to synchronize their speed with the front and back wheels for a smooth digging motion; **this calculation is performed in the** `state_controller` **node on the server, NOT on the Limb ESP!**
- `int linActs` can take on a value from **-100 to 100**, representing a percentage of the linear actuators' potential speed to write to their motor controllers.
- `bool door` can have the value **True or False** since the deposition door only has 2 potential states, "open" or "closed." In this case, True means an open door and False means a closed door.

___
#### `robotState/sensorData`
Used to send sensor data from the robot's Sensor ESP to the `read_robot_state` node on the server. The LIDAR sensor data is **NOT included in this!** This sensor data will give the `state_controller` node on the server infomation about the state of all the robot's limbs so that it can plan the next command accordingly. The data is sent in the following format: **work in progress** 

___
#### `robotState/obstacleData`
Used to send sensor data, specifically LIDAR data, from the robot's LIDAR ESP to the `read_robot_state` node on the server. This sensor data will give the `state_controller` node on the server information on the topology of the ground ahead of it. This then allows the `obstacle_detection` node to perform calculations on this data to determine whether or not there is an obstacle (hill or pit) in the robot's path. 

There are 4 LIDARs mounted in an array on the front of the robot, angled downwards to scan for obstacles. The data is sent in the following format: 

```
{ float lidar1
float lidar2
float lidar3
float lidar4 }
```

- `float lidar1` , `float lidar2` , `float lidar3` , `float lidar4`  are the distances that the 4 LIDARs read, from left to right. The distances are in centimeters.

___
<br />

<a name="gcs-server"></a>

### 2.2&nbsp;&nbsp;&nbsp;&nbsp;GCS ⟺ Server

Work in progress

___

<br />

<br />

<br />

<a name="additional-resources"></a>

## 3&nbsp;&nbsp;&nbsp;&nbsp;Additional Resources

All of the Academic Year 2020 software trainings can be found [here on Drive](https://drive.google.com/drive/folders/1dmQQXun6l71bd5Ws7cGFXm-T3bJ-IMG3?usp=sharing). You can also find many other resources on most of the topics used in our code (like Python programming, control theory, MQTT, etc.) [in this sheet](https://docs.google.com/document/d/1CRIx5FonDIs1Xs_TqEQANZTEb21N-sBxE2AgeskZIi8/edit?usp=sharing). These resources include blogs, YouTube tutorials, and textbooks. 

<br />

<br />

<br />

<a name="todo"></a>

## 4&nbsp;&nbsp;&nbsp;&nbsp;ADD TO README (WIP)

- Description of the robot
- Code structure (NUC, robot, GCS)
- Term definitions (like ESP and NUC and server and limbs)
- Link the trainings from Google Drive
