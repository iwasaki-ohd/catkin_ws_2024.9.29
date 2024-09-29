; Auto-generated. Do not edit!


(cl:in-package ros_posenet-msg)


;//! \htmlinclude Keypoint.msg.html

(cl:defclass <Keypoint> (roslisp-msg-protocol:ros-message)
  ((part
    :reader part
    :initarg :part
    :type cl:string
    :initform "")
   (score
    :reader score
    :initarg :score
    :type cl:float
    :initform 0.0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (image_position
    :reader image_position
    :initarg :image_position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass Keypoint (<Keypoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Keypoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Keypoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_posenet-msg:<Keypoint> is deprecated: use ros_posenet-msg:Keypoint instead.")))

(cl:ensure-generic-function 'part-val :lambda-list '(m))
(cl:defmethod part-val ((m <Keypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_posenet-msg:part-val is deprecated.  Use ros_posenet-msg:part instead.")
  (part m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <Keypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_posenet-msg:score-val is deprecated.  Use ros_posenet-msg:score instead.")
  (score m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Keypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_posenet-msg:position-val is deprecated.  Use ros_posenet-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'image_position-val :lambda-list '(m))
(cl:defmethod image_position-val ((m <Keypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_posenet-msg:image_position-val is deprecated.  Use ros_posenet-msg:image_position instead.")
  (image_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Keypoint>) ostream)
  "Serializes a message object of type '<Keypoint>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'part))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'part))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image_position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Keypoint>) istream)
  "Deserializes a message object of type '<Keypoint>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'part) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'part) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'score) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image_position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Keypoint>)))
  "Returns string type for a message object of type '<Keypoint>"
  "ros_posenet/Keypoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Keypoint)))
  "Returns string type for a message object of type 'Keypoint"
  "ros_posenet/Keypoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Keypoint>)))
  "Returns md5sum for a message object of type '<Keypoint>"
  "ed7553fef5f9989af54ac621aed181dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Keypoint)))
  "Returns md5sum for a message object of type 'Keypoint"
  "ed7553fef5f9989af54ac621aed181dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Keypoint>)))
  "Returns full string definition for message of type '<Keypoint>"
  (cl:format cl:nil "string part~%float64 score~%geometry_msgs/Point position~%geometry_msgs/Point image_position~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Keypoint)))
  "Returns full string definition for message of type 'Keypoint"
  (cl:format cl:nil "string part~%float64 score~%geometry_msgs/Point position~%geometry_msgs/Point image_position~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Keypoint>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'part))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image_position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Keypoint>))
  "Converts a ROS message object to a list"
  (cl:list 'Keypoint
    (cl:cons ':part (part msg))
    (cl:cons ':score (score msg))
    (cl:cons ':position (position msg))
    (cl:cons ':image_position (image_position msg))
))
