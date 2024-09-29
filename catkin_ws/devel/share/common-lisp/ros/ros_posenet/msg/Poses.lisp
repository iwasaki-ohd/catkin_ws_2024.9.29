; Auto-generated. Do not edit!


(cl:in-package ros_posenet-msg)


;//! \htmlinclude Poses.msg.html

(cl:defclass <Poses> (roslisp-msg-protocol:ros-message)
  ((poses
    :reader poses
    :initarg :poses
    :type (cl:vector ros_posenet-msg:Pose)
   :initform (cl:make-array 0 :element-type 'ros_posenet-msg:Pose :initial-element (cl:make-instance 'ros_posenet-msg:Pose))))
)

(cl:defclass Poses (<Poses>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Poses>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Poses)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_posenet-msg:<Poses> is deprecated: use ros_posenet-msg:Poses instead.")))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <Poses>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_posenet-msg:poses-val is deprecated.  Use ros_posenet-msg:poses instead.")
  (poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Poses>) ostream)
  "Serializes a message object of type '<Poses>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Poses>) istream)
  "Deserializes a message object of type '<Poses>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ros_posenet-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Poses>)))
  "Returns string type for a message object of type '<Poses>"
  "ros_posenet/Poses")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Poses)))
  "Returns string type for a message object of type 'Poses"
  "ros_posenet/Poses")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Poses>)))
  "Returns md5sum for a message object of type '<Poses>"
  "0e46ae84d5c891cad2f3c5e03526bd70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Poses)))
  "Returns md5sum for a message object of type 'Poses"
  "0e46ae84d5c891cad2f3c5e03526bd70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Poses>)))
  "Returns full string definition for message of type '<Poses>"
  (cl:format cl:nil "Pose[] poses~%================================================================================~%MSG: ros_posenet/Pose~%ros_posenet/Keypoint[] keypoints~%================================================================================~%MSG: ros_posenet/Keypoint~%string part~%float64 score~%geometry_msgs/Point position~%geometry_msgs/Point image_position~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Poses)))
  "Returns full string definition for message of type 'Poses"
  (cl:format cl:nil "Pose[] poses~%================================================================================~%MSG: ros_posenet/Pose~%ros_posenet/Keypoint[] keypoints~%================================================================================~%MSG: ros_posenet/Keypoint~%string part~%float64 score~%geometry_msgs/Point position~%geometry_msgs/Point image_position~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Poses>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Poses>))
  "Converts a ROS message object to a list"
  (cl:list 'Poses
    (cl:cons ':poses (poses msg))
))
