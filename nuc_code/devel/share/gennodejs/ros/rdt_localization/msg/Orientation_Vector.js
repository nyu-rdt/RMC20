// Auto-generated. Do not edit!

// (in-package rdt_localization.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Pose = require('./Pose.js');
let Location = require('./Location.js');

//-----------------------------------------------------------

class Orientation_Vector {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_pose = null;
      this.target_zone = null;
      this.robot_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_pose')) {
        this.robot_pose = initObj.robot_pose
      }
      else {
        this.robot_pose = new Pose();
      }
      if (initObj.hasOwnProperty('target_zone')) {
        this.target_zone = initObj.target_zone
      }
      else {
        this.target_zone = new Location();
      }
      if (initObj.hasOwnProperty('robot_speed')) {
        this.robot_speed = initObj.robot_speed
      }
      else {
        this.robot_speed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Orientation_Vector
    // Serialize message field [robot_pose]
    bufferOffset = Pose.serialize(obj.robot_pose, buffer, bufferOffset);
    // Serialize message field [target_zone]
    bufferOffset = Location.serialize(obj.target_zone, buffer, bufferOffset);
    // Serialize message field [robot_speed]
    bufferOffset = _serializer.int64(obj.robot_speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Orientation_Vector
    let len;
    let data = new Orientation_Vector(null);
    // Deserialize message field [robot_pose]
    data.robot_pose = Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_zone]
    data.target_zone = Location.deserialize(buffer, bufferOffset);
    // Deserialize message field [robot_speed]
    data.robot_speed = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rdt_localization/Orientation_Vector';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '32c0233ad09fff327409092d7163201c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Pose robot_pose
    Location target_zone
    int64 robot_speed
    ================================================================================
    MSG: rdt_localization/Pose
    float32 x
    float32 y
    float32 orientation
    ================================================================================
    MSG: rdt_localization/Location
    float32 x
    float32 y
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Orientation_Vector(null);
    if (msg.robot_pose !== undefined) {
      resolved.robot_pose = Pose.Resolve(msg.robot_pose)
    }
    else {
      resolved.robot_pose = new Pose()
    }

    if (msg.target_zone !== undefined) {
      resolved.target_zone = Location.Resolve(msg.target_zone)
    }
    else {
      resolved.target_zone = new Location()
    }

    if (msg.robot_speed !== undefined) {
      resolved.robot_speed = msg.robot_speed;
    }
    else {
      resolved.robot_speed = 0
    }

    return resolved;
    }
};

module.exports = Orientation_Vector;
