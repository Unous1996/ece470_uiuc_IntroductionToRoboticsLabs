// Auto-generated. Do not edit!

// (in-package ece470_ur3_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.destination = null;
      this.grip = null;
      this.softhome = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('destination')) {
        this.destination = initObj.destination
      }
      else {
        this.destination = [];
      }
      if (initObj.hasOwnProperty('grip')) {
        this.grip = initObj.grip
      }
      else {
        this.grip = 0;
      }
      if (initObj.hasOwnProperty('softhome')) {
        this.softhome = initObj.softhome
      }
      else {
        this.softhome = 0;
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type command
    // Serialize message field [destination]
    bufferOffset = _arraySerializer.float64(obj.destination, buffer, bufferOffset, null);
    // Serialize message field [grip]
    bufferOffset = _serializer.int32(obj.grip, buffer, bufferOffset);
    // Serialize message field [softhome]
    bufferOffset = _serializer.int32(obj.softhome, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float64(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type command
    let len;
    let data = new command(null);
    // Deserialize message field [destination]
    data.destination = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [grip]
    data.grip = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [softhome]
    data.softhome = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.destination.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ece470_ur3_driver/command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2d2c2700daec81019d4338afb533bad6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] destination
    int32 grip
    int32 softhome
    float64 duration
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new command(null);
    if (msg.destination !== undefined) {
      resolved.destination = msg.destination;
    }
    else {
      resolved.destination = []
    }

    if (msg.grip !== undefined) {
      resolved.grip = msg.grip;
    }
    else {
      resolved.grip = 0
    }

    if (msg.softhome !== undefined) {
      resolved.softhome = msg.softhome;
    }
    else {
      resolved.softhome = 0
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0.0
    }

    return resolved;
    }
};

module.exports = command;
