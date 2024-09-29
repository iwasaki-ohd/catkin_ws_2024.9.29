; Auto-generated. Do not edit!


(cl:in-package person_tracking-srv)


;//! \htmlinclude choose_target-request.msg.html

(cl:defclass <choose_target-request> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type cl:integer
    :initform 0))
)

(cl:defclass choose_target-request (<choose_target-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <choose_target-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'choose_target-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name person_tracking-srv:<choose_target-request> is deprecated: use person_tracking-srv:choose_target-request instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <choose_target-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader person_tracking-srv:target-val is deprecated.  Use person_tracking-srv:target instead.")
  (target m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <choose_target-request>) ostream)
  "Serializes a message object of type '<choose_target-request>"
  (cl:let* ((signed (cl:slot-value msg 'target)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <choose_target-request>) istream)
  "Deserializes a message object of type '<choose_target-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<choose_target-request>)))
  "Returns string type for a service object of type '<choose_target-request>"
  "person_tracking/choose_targetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'choose_target-request)))
  "Returns string type for a service object of type 'choose_target-request"
  "person_tracking/choose_targetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<choose_target-request>)))
  "Returns md5sum for a message object of type '<choose_target-request>"
  "02caa2968e58f6cef6489327a5fdc1c4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'choose_target-request)))
  "Returns md5sum for a message object of type 'choose_target-request"
  "02caa2968e58f6cef6489327a5fdc1c4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<choose_target-request>)))
  "Returns full string definition for message of type '<choose_target-request>"
  (cl:format cl:nil "int32 target~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'choose_target-request)))
  "Returns full string definition for message of type 'choose_target-request"
  (cl:format cl:nil "int32 target~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <choose_target-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <choose_target-request>))
  "Converts a ROS message object to a list"
  (cl:list 'choose_target-request
    (cl:cons ':target (target msg))
))
;//! \htmlinclude choose_target-response.msg.html

(cl:defclass <choose_target-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass choose_target-response (<choose_target-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <choose_target-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'choose_target-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name person_tracking-srv:<choose_target-response> is deprecated: use person_tracking-srv:choose_target-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <choose_target-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader person_tracking-srv:success-val is deprecated.  Use person_tracking-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <choose_target-response>) ostream)
  "Serializes a message object of type '<choose_target-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <choose_target-response>) istream)
  "Deserializes a message object of type '<choose_target-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<choose_target-response>)))
  "Returns string type for a service object of type '<choose_target-response>"
  "person_tracking/choose_targetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'choose_target-response)))
  "Returns string type for a service object of type 'choose_target-response"
  "person_tracking/choose_targetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<choose_target-response>)))
  "Returns md5sum for a message object of type '<choose_target-response>"
  "02caa2968e58f6cef6489327a5fdc1c4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'choose_target-response)))
  "Returns md5sum for a message object of type 'choose_target-response"
  "02caa2968e58f6cef6489327a5fdc1c4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<choose_target-response>)))
  "Returns full string definition for message of type '<choose_target-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'choose_target-response)))
  "Returns full string definition for message of type 'choose_target-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <choose_target-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <choose_target-response>))
  "Converts a ROS message object to a list"
  (cl:list 'choose_target-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'choose_target)))
  'choose_target-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'choose_target)))
  'choose_target-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'choose_target)))
  "Returns string type for a service object of type '<choose_target>"
  "person_tracking/choose_target")