; Auto-generated. Do not edit!


(cl:in-package motpy_ros-srv)


;//! \htmlinclude motpy_bbox-request.msg.html

(cl:defclass <motpy_bbox-request> (roslisp-msg-protocol:ros-message)
  ((input_bboxes
    :reader input_bboxes
    :initarg :input_bboxes
    :type darknet_ros_msgs-msg:BoundingBoxes
    :initform (cl:make-instance 'darknet_ros_msgs-msg:BoundingBoxes)))
)

(cl:defclass motpy_bbox-request (<motpy_bbox-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motpy_bbox-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motpy_bbox-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motpy_ros-srv:<motpy_bbox-request> is deprecated: use motpy_ros-srv:motpy_bbox-request instead.")))

(cl:ensure-generic-function 'input_bboxes-val :lambda-list '(m))
(cl:defmethod input_bboxes-val ((m <motpy_bbox-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motpy_ros-srv:input_bboxes-val is deprecated.  Use motpy_ros-srv:input_bboxes instead.")
  (input_bboxes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motpy_bbox-request>) ostream)
  "Serializes a message object of type '<motpy_bbox-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input_bboxes) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motpy_bbox-request>) istream)
  "Deserializes a message object of type '<motpy_bbox-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input_bboxes) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motpy_bbox-request>)))
  "Returns string type for a service object of type '<motpy_bbox-request>"
  "motpy_ros/motpy_bboxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motpy_bbox-request)))
  "Returns string type for a service object of type 'motpy_bbox-request"
  "motpy_ros/motpy_bboxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motpy_bbox-request>)))
  "Returns md5sum for a message object of type '<motpy_bbox-request>"
  "65b9571f1ed7828d4fcc56bef7d7664b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motpy_bbox-request)))
  "Returns md5sum for a message object of type 'motpy_bbox-request"
  "65b9571f1ed7828d4fcc56bef7d7664b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motpy_bbox-request>)))
  "Returns full string definition for message of type '<motpy_bbox-request>"
  (cl:format cl:nil "darknet_ros_msgs/BoundingBoxes input_bboxes~%~%~%================================================================================~%MSG: darknet_ros_msgs/BoundingBoxes~%Header header~%Header image_header~%BoundingBox[] bounding_boxes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: darknet_ros_msgs/BoundingBox~%float64 probability~%int64 xmin~%int64 ymin~%int64 xmax~%int64 ymax~%int16 id~%string Class~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motpy_bbox-request)))
  "Returns full string definition for message of type 'motpy_bbox-request"
  (cl:format cl:nil "darknet_ros_msgs/BoundingBoxes input_bboxes~%~%~%================================================================================~%MSG: darknet_ros_msgs/BoundingBoxes~%Header header~%Header image_header~%BoundingBox[] bounding_boxes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: darknet_ros_msgs/BoundingBox~%float64 probability~%int64 xmin~%int64 ymin~%int64 xmax~%int64 ymax~%int16 id~%string Class~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motpy_bbox-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input_bboxes))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motpy_bbox-request>))
  "Converts a ROS message object to a list"
  (cl:list 'motpy_bbox-request
    (cl:cons ':input_bboxes (input_bboxes msg))
))
;//! \htmlinclude motpy_bbox-response.msg.html

(cl:defclass <motpy_bbox-response> (roslisp-msg-protocol:ros-message)
  ((tracking_bboxes
    :reader tracking_bboxes
    :initarg :tracking_bboxes
    :type darknet_ros_msgs-msg:BoundingBoxes
    :initform (cl:make-instance 'darknet_ros_msgs-msg:BoundingBoxes)))
)

(cl:defclass motpy_bbox-response (<motpy_bbox-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motpy_bbox-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motpy_bbox-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motpy_ros-srv:<motpy_bbox-response> is deprecated: use motpy_ros-srv:motpy_bbox-response instead.")))

(cl:ensure-generic-function 'tracking_bboxes-val :lambda-list '(m))
(cl:defmethod tracking_bboxes-val ((m <motpy_bbox-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motpy_ros-srv:tracking_bboxes-val is deprecated.  Use motpy_ros-srv:tracking_bboxes instead.")
  (tracking_bboxes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motpy_bbox-response>) ostream)
  "Serializes a message object of type '<motpy_bbox-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tracking_bboxes) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motpy_bbox-response>) istream)
  "Deserializes a message object of type '<motpy_bbox-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tracking_bboxes) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motpy_bbox-response>)))
  "Returns string type for a service object of type '<motpy_bbox-response>"
  "motpy_ros/motpy_bboxResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motpy_bbox-response)))
  "Returns string type for a service object of type 'motpy_bbox-response"
  "motpy_ros/motpy_bboxResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motpy_bbox-response>)))
  "Returns md5sum for a message object of type '<motpy_bbox-response>"
  "65b9571f1ed7828d4fcc56bef7d7664b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motpy_bbox-response)))
  "Returns md5sum for a message object of type 'motpy_bbox-response"
  "65b9571f1ed7828d4fcc56bef7d7664b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motpy_bbox-response>)))
  "Returns full string definition for message of type '<motpy_bbox-response>"
  (cl:format cl:nil "~%darknet_ros_msgs/BoundingBoxes tracking_bboxes~%~%================================================================================~%MSG: darknet_ros_msgs/BoundingBoxes~%Header header~%Header image_header~%BoundingBox[] bounding_boxes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: darknet_ros_msgs/BoundingBox~%float64 probability~%int64 xmin~%int64 ymin~%int64 xmax~%int64 ymax~%int16 id~%string Class~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motpy_bbox-response)))
  "Returns full string definition for message of type 'motpy_bbox-response"
  (cl:format cl:nil "~%darknet_ros_msgs/BoundingBoxes tracking_bboxes~%~%================================================================================~%MSG: darknet_ros_msgs/BoundingBoxes~%Header header~%Header image_header~%BoundingBox[] bounding_boxes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: darknet_ros_msgs/BoundingBox~%float64 probability~%int64 xmin~%int64 ymin~%int64 xmax~%int64 ymax~%int16 id~%string Class~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motpy_bbox-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tracking_bboxes))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motpy_bbox-response>))
  "Converts a ROS message object to a list"
  (cl:list 'motpy_bbox-response
    (cl:cons ':tracking_bboxes (tracking_bboxes msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'motpy_bbox)))
  'motpy_bbox-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'motpy_bbox)))
  'motpy_bbox-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motpy_bbox)))
  "Returns string type for a service object of type '<motpy_bbox>"
  "motpy_ros/motpy_bbox")