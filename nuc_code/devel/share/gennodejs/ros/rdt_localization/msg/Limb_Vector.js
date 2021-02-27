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

class Limb_Vector {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.door = null;
      this.linActs_speed = null;
      this.arm_speed = null;
      this.drum_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('door')) {
        this.door = initObj.door
      }
      else {
        this.door = false;
      }
      if (initObj.hasOwnProperty('linActs_speed')) {
        this.linActs_speed = initObj.linActs_speed
      }
      else {
        this.linActs_speed = 0;
      }
      if (initObj.hasOwnProperty('arm_speed')) {
        this.arm_speed = initObj.arm_speed
      }
      else {
        this.arm_speed = 0;
      }
      if (initObj.hasOwnProperty('drum_speed')) {
        this.drum_speed = initObj.drum_speed
      }
      else {
        this.drum_speed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Limb_Vector
    // Serialize message field [door]
    bufferOffset = _serializer.bool(obj.door, buffer, bufferOffset);
    // Serialize message field [linActs_speed]
    bufferOffset = _serializer.int64(obj.linActs_speed, buffer, bufferOffset);
    // Serialize message field [arm_speed]
    bufferOffset = _serializer.int64(obj.arm_speed, buffer, bufferOffset);
    // Serialize message field [drum_speed]
    bufferOffset = _serializer.int64(obj.drum_speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Limb_Vector
    let len;
    let data = new Limb_Vector(null);
    // Deserialize message field [door]
    data.door = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [linActs_speed]
    data.linActs_speed = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [arm_speed]
    data.arm_speed = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [drum_speed]
    data.drum_speed = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rdt_localization/Limb_Vector';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '95198abd0938eea51c580fdd9748821b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool door
    int64 linActs_speed
    int64 arm_speed
    int64 drum_speed
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Limb_Vector(null);
    if (msg.door !== undefined) {
      resolved.door = msg.door;
    }
    else {
      resolved.door = false
    }

    if (msg.linActs_speed !== undefined) {
      resolved.linActs_speed = msg.linActs_speed;
    }
    else {
      resolved.linActs_speed = 0
    }

    if (msg.arm_speed !== undefined) {
      resolved.arm_speed = msg.arm_speed;
    }
    else {
      resolved.arm_speed = 0
    }

    if (msg.drum_speed !== undefined) {
      resolved.drum_speed = msg.drum_speed;
    }
    else {
      resolved.drum_speed = 0
    }

    return resolved;
    }
};

module.exports = Limb_Vector;
