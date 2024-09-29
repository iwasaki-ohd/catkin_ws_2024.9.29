// Auto-generated. Do not edit!

// (in-package motpy_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let darknet_ros_msgs = _finder('darknet_ros_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class motpy_bboxRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input_bboxes = null;
    }
    else {
      if (initObj.hasOwnProperty('input_bboxes')) {
        this.input_bboxes = initObj.input_bboxes
      }
      else {
        this.input_bboxes = new darknet_ros_msgs.msg.BoundingBoxes();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type motpy_bboxRequest
    // Serialize message field [input_bboxes]
    bufferOffset = darknet_ros_msgs.msg.BoundingBoxes.serialize(obj.input_bboxes, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type motpy_bboxRequest
    let len;
    let data = new motpy_bboxRequest(null);
    // Deserialize message field [input_bboxes]
    data.input_bboxes = darknet_ros_msgs.msg.BoundingBoxes.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += darknet_ros_msgs.msg.BoundingBoxes.getMessageSize(object.input_bboxes);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motpy_ros/motpy_bboxRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '89c24386ecd269e72479b7994ea9d64b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    darknet_ros_msgs/BoundingBoxes input_bboxes
    
    
    ================================================================================
    MSG: darknet_ros_msgs/BoundingBoxes
    Header header
    Header image_header
    BoundingBox[] bounding_boxes
    
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
    
    ================================================================================
    MSG: darknet_ros_msgs/BoundingBox
    float64 probability
    int64 xmin
    int64 ymin
    int64 xmax
    int64 ymax
    int16 id
    string Class
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new motpy_bboxRequest(null);
    if (msg.input_bboxes !== undefined) {
      resolved.input_bboxes = darknet_ros_msgs.msg.BoundingBoxes.Resolve(msg.input_bboxes)
    }
    else {
      resolved.input_bboxes = new darknet_ros_msgs.msg.BoundingBoxes()
    }

    return resolved;
    }
};

class motpy_bboxResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tracking_bboxes = null;
    }
    else {
      if (initObj.hasOwnProperty('tracking_bboxes')) {
        this.tracking_bboxes = initObj.tracking_bboxes
      }
      else {
        this.tracking_bboxes = new darknet_ros_msgs.msg.BoundingBoxes();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type motpy_bboxResponse
    // Serialize message field [tracking_bboxes]
    bufferOffset = darknet_ros_msgs.msg.BoundingBoxes.serialize(obj.tracking_bboxes, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type motpy_bboxResponse
    let len;
    let data = new motpy_bboxResponse(null);
    // Deserialize message field [tracking_bboxes]
    data.tracking_bboxes = darknet_ros_msgs.msg.BoundingBoxes.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += darknet_ros_msgs.msg.BoundingBoxes.getMessageSize(object.tracking_bboxes);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motpy_ros/motpy_bboxResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2926738309370b5390bcb94eb9638965';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    darknet_ros_msgs/BoundingBoxes tracking_bboxes
    
    ================================================================================
    MSG: darknet_ros_msgs/BoundingBoxes
    Header header
    Header image_header
    BoundingBox[] bounding_boxes
    
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
    
    ================================================================================
    MSG: darknet_ros_msgs/BoundingBox
    float64 probability
    int64 xmin
    int64 ymin
    int64 xmax
    int64 ymax
    int16 id
    string Class
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new motpy_bboxResponse(null);
    if (msg.tracking_bboxes !== undefined) {
      resolved.tracking_bboxes = darknet_ros_msgs.msg.BoundingBoxes.Resolve(msg.tracking_bboxes)
    }
    else {
      resolved.tracking_bboxes = new darknet_ros_msgs.msg.BoundingBoxes()
    }

    return resolved;
    }
};

module.exports = {
  Request: motpy_bboxRequest,
  Response: motpy_bboxResponse,
  md5sum() { return '65b9571f1ed7828d4fcc56bef7d7664b'; },
  datatype() { return 'motpy_ros/motpy_bbox'; }
};
