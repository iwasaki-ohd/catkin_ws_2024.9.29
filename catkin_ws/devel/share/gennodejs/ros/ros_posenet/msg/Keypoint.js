// Auto-generated. Do not edit!

// (in-package ros_posenet.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Keypoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.part = null;
      this.score = null;
      this.position = null;
      this.image_position = null;
    }
    else {
      if (initObj.hasOwnProperty('part')) {
        this.part = initObj.part
      }
      else {
        this.part = '';
      }
      if (initObj.hasOwnProperty('score')) {
        this.score = initObj.score
      }
      else {
        this.score = 0.0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('image_position')) {
        this.image_position = initObj.image_position
      }
      else {
        this.image_position = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Keypoint
    // Serialize message field [part]
    bufferOffset = _serializer.string(obj.part, buffer, bufferOffset);
    // Serialize message field [score]
    bufferOffset = _serializer.float64(obj.score, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [image_position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.image_position, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Keypoint
    let len;
    let data = new Keypoint(null);
    // Deserialize message field [part]
    data.part = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [score]
    data.score = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [image_position]
    data.image_position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.part);
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_posenet/Keypoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ed7553fef5f9989af54ac621aed181dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Keypoint(null);
    if (msg.part !== undefined) {
      resolved.part = msg.part;
    }
    else {
      resolved.part = ''
    }

    if (msg.score !== undefined) {
      resolved.score = msg.score;
    }
    else {
      resolved.score = 0.0
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.image_position !== undefined) {
      resolved.image_position = geometry_msgs.msg.Point.Resolve(msg.image_position)
    }
    else {
      resolved.image_position = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = Keypoint;
