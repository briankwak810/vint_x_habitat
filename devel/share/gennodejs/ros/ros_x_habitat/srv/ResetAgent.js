// Auto-generated. Do not edit!

// (in-package ros_x_habitat.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ResetAgentRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reset = null;
      this.seed = null;
    }
    else {
      if (initObj.hasOwnProperty('reset')) {
        this.reset = initObj.reset
      }
      else {
        this.reset = 0;
      }
      if (initObj.hasOwnProperty('seed')) {
        this.seed = initObj.seed
      }
      else {
        this.seed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetAgentRequest
    // Serialize message field [reset]
    bufferOffset = _serializer.int16(obj.reset, buffer, bufferOffset);
    // Serialize message field [seed]
    bufferOffset = _serializer.int32(obj.seed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetAgentRequest
    let len;
    let data = new ResetAgentRequest(null);
    // Deserialize message field [reset]
    data.reset = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [seed]
    data.seed = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_x_habitat/ResetAgentRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ece443822a688f9f324e3458e0560a90';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 reset # 0 for reset, 1 for shutdown
    int32 seed # seed to instantiate agent
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetAgentRequest(null);
    if (msg.reset !== undefined) {
      resolved.reset = msg.reset;
    }
    else {
      resolved.reset = 0
    }

    if (msg.seed !== undefined) {
      resolved.seed = msg.seed;
    }
    else {
      resolved.seed = 0
    }

    return resolved;
    }
};

class ResetAgentResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.done = null;
    }
    else {
      if (initObj.hasOwnProperty('done')) {
        this.done = initObj.done
      }
      else {
        this.done = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetAgentResponse
    // Serialize message field [done]
    bufferOffset = _serializer.bool(obj.done, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetAgentResponse
    let len;
    let data = new ResetAgentResponse(null);
    // Deserialize message field [done]
    data.done = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_x_habitat/ResetAgentResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '89bb254424e4cffedbf494e7b0ddbfea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool done
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetAgentResponse(null);
    if (msg.done !== undefined) {
      resolved.done = msg.done;
    }
    else {
      resolved.done = false
    }

    return resolved;
    }
};

module.exports = {
  Request: ResetAgentRequest,
  Response: ResetAgentResponse,
  md5sum() { return 'ce8257351a1f8b1ef5fb3350a09efd5a'; },
  datatype() { return 'ros_x_habitat/ResetAgent'; }
};
