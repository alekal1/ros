// Auto-generated. Do not edit!

// (in-package differential_robot_185104iaib.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class counter_message {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.count_left = null;
      this.count_right = null;
    }
    else {
      if (initObj.hasOwnProperty('count_left')) {
        this.count_left = initObj.count_left
      }
      else {
        this.count_left = 0;
      }
      if (initObj.hasOwnProperty('count_right')) {
        this.count_right = initObj.count_right
      }
      else {
        this.count_right = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type counter_message
    // Serialize message field [count_left]
    bufferOffset = _serializer.int32(obj.count_left, buffer, bufferOffset);
    // Serialize message field [count_right]
    bufferOffset = _serializer.int32(obj.count_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type counter_message
    let len;
    let data = new counter_message(null);
    // Deserialize message field [count_left]
    data.count_left = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [count_right]
    data.count_right = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'differential_robot_185104iaib/counter_message';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9acad0024d496a45d7194e5310734a3c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int32 count_left
    int32 count_right
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new counter_message(null);
    if (msg.count_left !== undefined) {
      resolved.count_left = msg.count_left;
    }
    else {
      resolved.count_left = 0
    }

    if (msg.count_right !== undefined) {
      resolved.count_right = msg.count_right;
    }
    else {
      resolved.count_right = 0
    }

    return resolved;
    }
};

module.exports = counter_message;
