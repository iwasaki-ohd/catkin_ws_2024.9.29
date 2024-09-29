; Auto-generated. Do not edit!


(cl:in-package ros_posenet-msg)


;//! \htmlinclude aPoses.msg.html

(cl:defclass <aPoses> (roslisp-msg-protocol:ros-message)
  ((part
    :reader part
    :initarg :part
    :type cl:string
    :initform "")
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass aPoses (<aPoses>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <aPoses>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'aPoses)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_posenet-msg:<aPoses> is deprecated: use ros_posenet-msg:aPoses instead.")))

(cl:ensure-generic-function 'part-val :lambda-list '(m))
(cl:defmethod part-val ((m <aPoses>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_posenet-msg:part-val is deprecated.  Use ros_posenet-msg:part instead.")
  (part m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <aPoses>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_posenet-msg:position-val is deprecated.  Use ros_posenet-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <aPoses>) ostream)
  "Serializes a message object of type '<aPoses>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'part))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'part))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <aPoses>) istream)
  "Deserializes a message object of type '<aPoses>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'part) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'part) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<aPoses>)))
  "Returns string type for a message object of type '<aPoses>"
  "ros_posenet/aPoses")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'aPoses)))
  "Returns string type for a message object of type 'aPoses"
  "ros_posenet/aPoses")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<aPoses>)))
  "Returns md5sum for a message object of type '<aPoses>"
  "48f6b649951fc40ababaa77312f83e65")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'aPoses)))
  "Returns md5sum for a message object of type 'aPoses"
  "48f6b649951fc40ababaa77312f83e65")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<aPoses>)))
  "Returns full string definition for message of type '<aPoses>"
  (cl:format cl:nil "string part~%geometry_msgs/Point position~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'aPoses)))
  "Returns full string definition for message of type 'aPoses"
  (cl:format cl:nil "string part~%geometry_msgs/Point position~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <aPoses>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'part))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <aPoses>))
  "Converts a ROS message object to a list"
  (cl:list 'aPoses
    (cl:cons ':part (part msg))
    (cl:cons ':position (position msg))
))
