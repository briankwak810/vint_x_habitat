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

class EvalEpisodeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.episode_id_last = null;
      this.scene_id_last = null;
    }
    else {
      if (initObj.hasOwnProperty('episode_id_last')) {
        this.episode_id_last = initObj.episode_id_last
      }
      else {
        this.episode_id_last = '';
      }
      if (initObj.hasOwnProperty('scene_id_last')) {
        this.scene_id_last = initObj.scene_id_last
      }
      else {
        this.scene_id_last = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EvalEpisodeRequest
    // Serialize message field [episode_id_last]
    bufferOffset = _serializer.string(obj.episode_id_last, buffer, bufferOffset);
    // Serialize message field [scene_id_last]
    bufferOffset = _serializer.string(obj.scene_id_last, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EvalEpisodeRequest
    let len;
    let data = new EvalEpisodeRequest(null);
    // Deserialize message field [episode_id_last]
    data.episode_id_last = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [scene_id_last]
    data.scene_id_last = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.episode_id_last);
    length += _getByteLength(object.scene_id_last);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_x_habitat/EvalEpisodeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7ac0e35740d3cb94d954674abbd9cf6c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string episode_id_last
    string scene_id_last
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EvalEpisodeRequest(null);
    if (msg.episode_id_last !== undefined) {
      resolved.episode_id_last = msg.episode_id_last;
    }
    else {
      resolved.episode_id_last = ''
    }

    if (msg.scene_id_last !== undefined) {
      resolved.scene_id_last = msg.scene_id_last;
    }
    else {
      resolved.scene_id_last = ''
    }

    return resolved;
    }
};

class EvalEpisodeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.episode_id = null;
      this.scene_id = null;
      this.distance_to_goal = null;
      this.success = null;
      this.spl = null;
      this.num_steps = null;
      this.agent_time = null;
      this.sim_time = null;
      this.reset_time = null;
    }
    else {
      if (initObj.hasOwnProperty('episode_id')) {
        this.episode_id = initObj.episode_id
      }
      else {
        this.episode_id = '';
      }
      if (initObj.hasOwnProperty('scene_id')) {
        this.scene_id = initObj.scene_id
      }
      else {
        this.scene_id = '';
      }
      if (initObj.hasOwnProperty('distance_to_goal')) {
        this.distance_to_goal = initObj.distance_to_goal
      }
      else {
        this.distance_to_goal = 0.0;
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = 0.0;
      }
      if (initObj.hasOwnProperty('spl')) {
        this.spl = initObj.spl
      }
      else {
        this.spl = 0.0;
      }
      if (initObj.hasOwnProperty('num_steps')) {
        this.num_steps = initObj.num_steps
      }
      else {
        this.num_steps = 0;
      }
      if (initObj.hasOwnProperty('agent_time')) {
        this.agent_time = initObj.agent_time
      }
      else {
        this.agent_time = 0.0;
      }
      if (initObj.hasOwnProperty('sim_time')) {
        this.sim_time = initObj.sim_time
      }
      else {
        this.sim_time = 0.0;
      }
      if (initObj.hasOwnProperty('reset_time')) {
        this.reset_time = initObj.reset_time
      }
      else {
        this.reset_time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EvalEpisodeResponse
    // Serialize message field [episode_id]
    bufferOffset = _serializer.string(obj.episode_id, buffer, bufferOffset);
    // Serialize message field [scene_id]
    bufferOffset = _serializer.string(obj.scene_id, buffer, bufferOffset);
    // Serialize message field [distance_to_goal]
    bufferOffset = _serializer.float32(obj.distance_to_goal, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.float32(obj.success, buffer, bufferOffset);
    // Serialize message field [spl]
    bufferOffset = _serializer.float32(obj.spl, buffer, bufferOffset);
    // Serialize message field [num_steps]
    bufferOffset = _serializer.int32(obj.num_steps, buffer, bufferOffset);
    // Serialize message field [agent_time]
    bufferOffset = _serializer.float32(obj.agent_time, buffer, bufferOffset);
    // Serialize message field [sim_time]
    bufferOffset = _serializer.float32(obj.sim_time, buffer, bufferOffset);
    // Serialize message field [reset_time]
    bufferOffset = _serializer.float32(obj.reset_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EvalEpisodeResponse
    let len;
    let data = new EvalEpisodeResponse(null);
    // Deserialize message field [episode_id]
    data.episode_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [scene_id]
    data.scene_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [distance_to_goal]
    data.distance_to_goal = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [spl]
    data.spl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [num_steps]
    data.num_steps = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [agent_time]
    data.agent_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sim_time]
    data.sim_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [reset_time]
    data.reset_time = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.episode_id);
    length += _getByteLength(object.scene_id);
    return length + 36;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_x_habitat/EvalEpisodeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '493793cd7bc05e29a188144407a41392';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string episode_id
    string scene_id
    float32 distance_to_goal
    float32 success
    float32 spl
    int32 num_steps
    float32 agent_time
    float32 sim_time
    float32 reset_time
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EvalEpisodeResponse(null);
    if (msg.episode_id !== undefined) {
      resolved.episode_id = msg.episode_id;
    }
    else {
      resolved.episode_id = ''
    }

    if (msg.scene_id !== undefined) {
      resolved.scene_id = msg.scene_id;
    }
    else {
      resolved.scene_id = ''
    }

    if (msg.distance_to_goal !== undefined) {
      resolved.distance_to_goal = msg.distance_to_goal;
    }
    else {
      resolved.distance_to_goal = 0.0
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = 0.0
    }

    if (msg.spl !== undefined) {
      resolved.spl = msg.spl;
    }
    else {
      resolved.spl = 0.0
    }

    if (msg.num_steps !== undefined) {
      resolved.num_steps = msg.num_steps;
    }
    else {
      resolved.num_steps = 0
    }

    if (msg.agent_time !== undefined) {
      resolved.agent_time = msg.agent_time;
    }
    else {
      resolved.agent_time = 0.0
    }

    if (msg.sim_time !== undefined) {
      resolved.sim_time = msg.sim_time;
    }
    else {
      resolved.sim_time = 0.0
    }

    if (msg.reset_time !== undefined) {
      resolved.reset_time = msg.reset_time;
    }
    else {
      resolved.reset_time = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: EvalEpisodeRequest,
  Response: EvalEpisodeResponse,
  md5sum() { return '4cf4a4cf4a1d60db72496f8e3635a2d3'; },
  datatype() { return 'ros_x_habitat/EvalEpisode'; }
};
