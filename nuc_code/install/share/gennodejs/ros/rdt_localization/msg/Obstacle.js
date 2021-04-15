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

class Obstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.front_left = null;
      this.front_right = null;
      this.front_left_mid = null;
      this.front_right_mid = null;
      this.depo_front = null;
      this.depo_back = null;
    }
    else {
      if (initObj.hasOwnProperty('front_left')) {
        this.front_left = initObj.front_left
      }
      else {
        this.front_left = 0;
      }
      if (initObj.hasOwnProperty('front_right')) {
        this.front_right = initObj.front_right
      }
      else {
        this.front_right = 0;
      }
      if (initObj.hasOwnProperty('front_left_mid')) {
        this.front_left_mid = initObj.front_left_mid
      }
      else {
        this.front_left_mid = 0;
      }
      if (initObj.hasOwnProperty('front_right_mid')) {
        this.front_right_mid = initObj.front_right_mid
      }
      else {
        this.front_right_mid = 0;
      }
      if (initObj.hasOwnProperty('depo_front')) {
        this.depo_front = initObj.depo_front
      }
      else {
        this.depo_front = 0;
      }
      if (initObj.hasOwnProperty('depo_back')) {
        this.depo_back = initObj.depo_back
      }
      else {
        this.depo_back = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Obstacle
    // Serialize message field [front_left]
    bufferOffset = _serializer.int64(obj.front_left, buffer, bufferOffset);
    // Serialize message field [front_right]
    bufferOffset = _serializer.int64(obj.front_right, buffer, bufferOffset);
    // Serialize message field [front_left_mid]
    bufferOffset = _serializer.int64(obj.front_left_mid, buffer, bufferOffset);
    // Serialize message field [front_right_mid]
    bufferOffset = _serializer.int64(obj.front_right_mid, buffer, bufferOffset);
    // Serialize message field [depo_front]
    bufferOffset = _serializer.int64(obj.depo_front, buffer, bufferOffset);
    // Serialize message field [depo_back]
    bufferOffset = _serializer.int64(obj.depo_back, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Obstacle
    let len;
    let data = new Obstacle(null);
    // Deserialize message field [front_left]
    data.front_left = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [front_right]
    data.front_right = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [front_left_mid]
    data.front_left_mid = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [front_right_mid]
    data.front_right_mid = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [depo_front]
    data.depo_front = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [depo_back]
    data.depo_back = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rdt_localization/Obstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8b91b992300f93c3e218076979730b49';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 front_left
    int64 front_right
    int64 front_left_mid
    int64 front_right_mid
    int64 depo_front
    int64 depo_back
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Obstacle(null);
    if (msg.front_left !== undefined) {
      resolved.front_left = msg.front_left;
    }
    else {
      resolved.front_left = 0
    }

    if (msg.front_right !== undefined) {
      resolved.front_right = msg.front_right;
    }
    else {
      resolved.front_right = 0
    }

    if (msg.front_left_mid !== undefined) {
      resolved.front_left_mid = msg.front_left_mid;
    }
    else {
      resolved.front_left_mid = 0
    }

    if (msg.front_right_mid !== undefined) {
      resolved.front_right_mid = msg.front_right_mid;
    }
    else {
      resolved.front_right_mid = 0
    }

    if (msg.depo_front !== undefined) {
      resolved.depo_front = msg.depo_front;
    }
    else {
      resolved.depo_front = 0
    }

    if (msg.depo_back !== undefined) {
      resolved.depo_back = msg.depo_back;
    }
    else {
      resolved.depo_back = 0
    }

    return resolved;
    }
};

module.exports = Obstacle;
