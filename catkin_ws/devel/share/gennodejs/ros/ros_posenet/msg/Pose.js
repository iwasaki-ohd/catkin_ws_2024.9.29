// Auto-generated. Do not edit!

// (in-package ros_posenet.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Keypoint = require('./Keypoint.js');

//-----------------------------------------------------------

class Pose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.keypoints = null;
    }
    else {
      if (initObj.hasOwnProperty('keypoints')) {
        this.keypoints = initObj.keypoints
      }
      else {
        this.keypoints = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Pose
    // Serialize message field [keypoints]
    // Serialize the length for message field [keypoints]
    bufferOffset = _serializer.uint32(obj.keypoints.length, buffer, bufferOffset);
    obj.keypoints.forEach((val) => {
      bufferOffset = Keypoint.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Pose
    let len;
    let data = new Pose(null);
    // Deserialize message field [keypoints]
    // Deserialize array length for message field [keypoints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.keypoints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.keypoints[i] = Keypoint.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.keypoints.forEach((val) => {
      length += Keypoint.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_posenet/Pose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3e476a684ef3c919da15f180dd754305';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ros_posenet/Keypoint[] keypoints
    ================================================================================
    MSG: ros_posenet/Keypoint
    string part
    float64 score
    geometry_msgs/Point position
    geometry_msgs/Point image_position
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Pose(null);
    if (msg.keypoints !== undefined) {
      resolved.keypoints = new Array(msg.keypoints.length);
      for (let i = 0; i < resolved.keypoints.length; ++i) {
        resolved.keypoints[i] = Keypoint.Resolve(msg.keypoints[i]);
      }
    }
    else {
      resolved.keypoints = []
    }

    return resolved;
    }
};

module.exports = Pose;
