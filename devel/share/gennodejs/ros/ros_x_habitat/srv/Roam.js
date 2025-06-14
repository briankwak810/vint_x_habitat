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

class RoamRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.episode_id_last = null;
      this.scene_id_last = null;
      this.make_video = null;
      this.video_frame_period = null;
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
      if (initObj.hasOwnProperty('make_video')) {
        this.make_video = initObj.make_video
      }
      else {
        this.make_video = false;
      }
      if (initObj.hasOwnProperty('video_frame_period')) {
        this.video_frame_period = initObj.video_frame_period
      }
      else {
        this.video_frame_period = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RoamRequest
    // Serialize message field [episode_id_last]
    bufferOffset = _serializer.string(obj.episode_id_last, buffer, bufferOffset);
    // Serialize message field [scene_id_last]
    bufferOffset = _serializer.string(obj.scene_id_last, buffer, bufferOffset);
    // Serialize message field [make_video]
    bufferOffset = _serializer.bool(obj.make_video, buffer, bufferOffset);
    // Serialize message field [video_frame_period]
    bufferOffset = _serializer.int32(obj.video_frame_period, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RoamRequest
    let len;
    let data = new RoamRequest(null);
    // Deserialize message field [episode_id_last]
    data.episode_id_last = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [scene_id_last]
    data.scene_id_last = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [make_video]
    data.make_video = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [video_frame_period]
    data.video_frame_period = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.episode_id_last);
    length += _getByteLength(object.scene_id_last);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_x_habitat/RoamRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f3c5236285c0e72cc3a66cf98d0485af';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string episode_id_last
    string scene_id_last
    bool make_video
    int32 video_frame_period
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RoamRequest(null);
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

    if (msg.make_video !== undefined) {
      resolved.make_video = msg.make_video;
    }
    else {
      resolved.make_video = false
    }

    if (msg.video_frame_period !== undefined) {
      resolved.video_frame_period = msg.video_frame_period;
    }
    else {
      resolved.video_frame_period = 0
    }

    return resolved;
    }
};

class RoamResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ack = null;
    }
    else {
      if (initObj.hasOwnProperty('ack')) {
        this.ack = initObj.ack
      }
      else {
        this.ack = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RoamResponse
    // Serialize message field [ack]
    bufferOffset = _serializer.bool(obj.ack, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RoamResponse
    let len;
    let data = new RoamResponse(null);
    // Deserialize message field [ack]
    data.ack = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ros_x_habitat/RoamResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8f5729177853f34b146e2e57766d4dc2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ack
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RoamResponse(null);
    if (msg.ack !== undefined) {
      resolved.ack = msg.ack;
    }
    else {
      resolved.ack = false
    }

    return resolved;
    }
};

module.exports = {
  Request: RoamRequest,
  Response: RoamResponse,
  md5sum() { return '1c6692b437c77a3364ed4415992a3409'; },
  datatype() { return 'ros_x_habitat/Roam'; }
};
