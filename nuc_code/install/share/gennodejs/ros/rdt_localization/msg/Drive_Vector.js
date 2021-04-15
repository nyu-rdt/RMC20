// Auto-generated. Do not edit!

// (in-package rdt_localization.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Drive_Vector {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_spd = null;
      this.offset_driveMode = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_spd')) {
        this.robot_spd = initObj.robot_spd
      }
      else {
        this.robot_spd = 0;
      }
      if (initObj.hasOwnProperty('offset_driveMode')) {
        this.offset_driveMode = initObj.offset_driveMode
      }
      else {
        this.offset_driveMode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Drive_Vector
    // Serialize message field [robot_spd]
    bufferOffset = _serializer.int64(obj.robot_spd, buffer, bufferOffset);
    // Serialize message field [offset_driveMode]
    bufferOffset = _serializer.int64(obj.offset_driveMode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Drive_Vector
    let len;
    let data = new Drive_Vector(null);
    // Deserialize message field [robot_spd]
    data.robot_spd = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [offset_driveMode]
    data.offset_driveMode = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rdt_localization/Drive_Vector';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9bb2e6492d363aa84cce71e895f95bc0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 robot_spd
    int64 offset_driveMode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Drive_Vector(null);
    if (msg.robot_spd !== undefined) {
      resolved.robot_spd = msg.robot_spd;
    }
    else {
      resolved.robot_spd = 0
    }

    if (msg.offset_driveMode !== undefined) {
      resolved.offset_driveMode = msg.offset_driveMode;
    }
    else {
      resolved.offset_driveMode = 0
    }

    return resolved;
    }
};

module.exports = Drive_Vector;
