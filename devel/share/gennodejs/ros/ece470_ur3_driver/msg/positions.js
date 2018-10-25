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

class positions {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.velocity = null;
      this.grip = null;
      this.isReady = null;
      this.state = null;
      this.pending = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = [];
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = [];
      }
      if (initObj.hasOwnProperty('grip')) {
        this.grip = initObj.grip
      }
      else {
        this.grip = 0;
      }
      if (initObj.hasOwnProperty('isReady')) {
        this.isReady = initObj.isReady
      }
      else {
        this.isReady = false;
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = '';
      }
      if (initObj.hasOwnProperty('pending')) {
        this.pending = initObj.pending
      }
      else {
        this.pending = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type positions
    // Serialize message field [position]
    bufferOffset = _arraySerializer.float64(obj.position, buffer, bufferOffset, null);
    // Serialize message field [velocity]
    bufferOffset = _arraySerializer.float64(obj.velocity, buffer, bufferOffset, null);
    // Serialize message field [grip]
    bufferOffset = _serializer.int32(obj.grip, buffer, bufferOffset);
    // Serialize message field [isReady]
    bufferOffset = _serializer.bool(obj.isReady, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.string(obj.state, buffer, bufferOffset);
    // Serialize message field [pending]
    bufferOffset = _serializer.bool(obj.pending, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type positions
    let len;
    let data = new positions(null);
    // Deserialize message field [position]
    data.position = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [velocity]
    data.velocity = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [grip]
    data.grip = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [isReady]
    data.isReady = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pending]
    data.pending = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.position.length;
    length += 8 * object.velocity.length;
    length += object.state.length;
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ece470_ur3_driver/positions';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bf2455dc51ac9b75c8e422304ce3ab30';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] position
    float64[] velocity
    int32 grip
    bool isReady
    string state
    bool pending
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new positions(null);
    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = []
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = []
    }

    if (msg.grip !== undefined) {
      resolved.grip = msg.grip;
    }
    else {
      resolved.grip = 0
    }

    if (msg.isReady !== undefined) {
      resolved.isReady = msg.isReady;
    }
    else {
      resolved.isReady = false
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = ''
    }

    if (msg.pending !== undefined) {
      resolved.pending = msg.pending;
    }
    else {
      resolved.pending = false
    }

    return resolved;
    }
};

module.exports = positions;
