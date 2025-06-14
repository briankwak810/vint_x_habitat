// Auto-generated. Do not edit!

// (in-package ros_x_habitat.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PointGoalWithGPSCompass {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.distance_to_goal = null;
      this.angle_to_goal = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('distance_to_goal')) {
        this.distance_to_goal = initObj.distance_to_goal
      }
      else {
        this.distance_to_goal = 0.0;
      }
      if (initObj.hasOwnProperty('angle_to_goal')) {
        this.angle_to_goal = initObj.angle_to_goal
      }
      else {
        this.angle_to_goal = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PointGoalWithGPSCompass
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [distance_to_goal]
    bufferOffset = _serializer.float64(obj.distance_to_goal, buffer, bufferOffset);
    // Serialize message field [angle_to_goal]
    bufferOffset = _serializer.float64(obj.angle_to_goal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PointGoalWithGPSCompass
    let len;
    let data = new PointGoalWithGPSCompass(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [distance_to_goal]
    data.distance_to_goal = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angle_to_goal]
    data.angle_to_goal = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_x_habitat/PointGoalWithGPSCompass';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f1c2ae08f20a219947d5152f75072a85';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # A representation of readings from Habitat's POINTGOAL_WITH_GPS_COMPASS sensor
    
    Header header
    float64 distance_to_goal
    float64 angle_to_goal # in radians
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PointGoalWithGPSCompass(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.distance_to_goal !== undefined) {
      resolved.distance_to_goal = msg.distance_to_goal;
    }
    else {
      resolved.distance_to_goal = 0.0
    }

    if (msg.angle_to_goal !== undefined) {
      resolved.angle_to_goal = msg.angle_to_goal;
    }
    else {
      resolved.angle_to_goal = 0.0
    }

    return resolved;
    }
};

module.exports = PointGoalWithGPSCompass;
