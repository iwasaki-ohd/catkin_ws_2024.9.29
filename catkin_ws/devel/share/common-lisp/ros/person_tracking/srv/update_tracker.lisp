; Auto-generated. Do not edit!


(cl:in-package person_tracking-srv)


;//! \htmlinclude update_tracker-request.msg.html

(cl:defclass <update_tracker-request> (roslisp-msg-protocol:ros-message)
  ((img
    :reader img
    :initarg :img
    :type sensor_msgs-msg:CompressedImage
    :initform (cl:make-instance 'sensor_msgs-msg:CompressedImage)))
)

(cl:defclass update_tracker-request (<update_tracker-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <update_tracker-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'update_tracker-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name person_tracking-srv:<update_tracker-request> is deprecated: use person_tracking-srv:update_tracker-request instead.")))

(cl:ensure-generic-function 'img-val :lambda-list '(m))
(cl:defmethod img-val ((m <update_tracker-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader person_tracking-srv:img-val is deprecated.  Use person_tracking-srv:img instead.")
  (img m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <update_tracker-request>) ostream)
  "Serializes a message object of type '<update_tracker-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'img) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <update_tracker-request>) istream)
  "Deserializes a message object of type '<update_tracker-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'img) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<update_tracker-request>)))
  "Returns string type for a service object of type '<update_tracker-request>"
  "person_tracking/update_trackerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'update_tracker-request)))
  "Returns string type for a service object of type 'update_tracker-request"
  "person_tracking/update_trackerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<update_tracker-request>)))
  "Returns md5sum for a message object of type '<update_tracker-request>"
  "4a50c39106e487c8fc1d6f02a4dd5e96")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'update_tracker-request)))
  "Returns md5sum for a message object of type 'update_tracker-request"
  "4a50c39106e487c8fc1d6f02a4dd5e96")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<update_tracker-request>)))
  "Returns full string definition for message of type '<update_tracker-request>"
  (cl:format cl:nil "sensor_msgs/CompressedImage img~%~%================================================================================~%MSG: sensor_msgs/CompressedImage~%# This message contains a compressed image~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%~%string format        # Specifies the format of the data~%                     #   Acceptable values:~%                     #     jpeg, png~%uint8[] data         # Compressed image buffer~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'update_tracker-request)))
  "Returns full string definition for message of type 'update_tracker-request"
  (cl:format cl:nil "sensor_msgs/CompressedImage img~%~%================================================================================~%MSG: sensor_msgs/CompressedImage~%# This message contains a compressed image~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%~%string format        # Specifies the format of the data~%                     #   Acceptable values:~%                     #     jpeg, png~%uint8[] data         # Compressed image buffer~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <update_tracker-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'img))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <update_tracker-request>))
  "Converts a ROS message object to a list"
  (cl:list 'update_tracker-request
    (cl:cons ':img (img msg))
))
;//! \htmlinclude update_tracker-response.msg.html

(cl:defclass <update_tracker-response> (roslisp-msg-protocol:ros-message)
  ((coordinates
    :reader coordinates
    :initarg :coordinates
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass update_tracker-response (<update_tracker-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <update_tracker-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'update_tracker-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name person_tracking-srv:<update_tracker-response> is deprecated: use person_tracking-srv:update_tracker-response instead.")))

(cl:ensure-generic-function 'coordinates-val :lambda-list '(m))
(cl:defmethod coordinates-val ((m <update_tracker-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader person_tracking-srv:coordinates-val is deprecated.  Use person_tracking-srv:coordinates instead.")
  (coordinates m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <update_tracker-response>) ostream)
  "Serializes a message object of type '<update_tracker-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'coordinates) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <update_tracker-response>) istream)
  "Deserializes a message object of type '<update_tracker-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'coordinates) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<update_tracker-response>)))
  "Returns string type for a service object of type '<update_tracker-response>"
  "person_tracking/update_trackerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'update_tracker-response)))
  "Returns string type for a service object of type 'update_tracker-response"
  "person_tracking/update_trackerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<update_tracker-response>)))
  "Returns md5sum for a message object of type '<update_tracker-response>"
  "4a50c39106e487c8fc1d6f02a4dd5e96")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'update_tracker-response)))
  "Returns md5sum for a message object of type 'update_tracker-response"
  "4a50c39106e487c8fc1d6f02a4dd5e96")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<update_tracker-response>)))
  "Returns full string definition for message of type '<update_tracker-response>"
  (cl:format cl:nil "geometry_msgs/Point coordinates~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'update_tracker-response)))
  "Returns full string definition for message of type 'update_tracker-response"
  (cl:format cl:nil "geometry_msgs/Point coordinates~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <update_tracker-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'coordinates))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <update_tracker-response>))
  "Converts a ROS message object to a list"
  (cl:list 'update_tracker-response
    (cl:cons ':coordinates (coordinates msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'update_tracker)))
  'update_tracker-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'update_tracker)))
  'update_tracker-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'update_tracker)))
  "Returns string type for a service object of type '<update_tracker>"
  "person_tracking/update_tracker")