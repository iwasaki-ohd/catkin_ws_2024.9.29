; Auto-generated. Do not edit!


(cl:in-package person_tracking-srv)


;//! \htmlinclude clear_target-request.msg.html

(cl:defclass <clear_target-request> (roslisp-msg-protocol:ros-message)
  ((clear
    :reader clear
    :initarg :clear
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass clear_target-request (<clear_target-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <clear_target-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'clear_target-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name person_tracking-srv:<clear_target-request> is deprecated: use person_tracking-srv:clear_target-request instead.")))

(cl:ensure-generic-function 'clear-val :lambda-list '(m))
(cl:defmethod clear-val ((m <clear_target-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader person_tracking-srv:clear-val is deprecated.  Use person_tracking-srv:clear instead.")
  (clear m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <clear_target-request>) ostream)
  "Serializes a message object of type '<clear_target-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clear) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <clear_target-request>) istream)
  "Deserializes a message object of type '<clear_target-request>"
    (cl:setf (cl:slot-value msg 'clear) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<clear_target-request>)))
  "Returns string type for a service object of type '<clear_target-request>"
  "person_tracking/clear_targetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'clear_target-request)))
  "Returns string type for a service object of type 'clear_target-request"
  "person_tracking/clear_targetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<clear_target-request>)))
  "Returns md5sum for a message object of type '<clear_target-request>"
  "8d891785a3ff0d216ac0e1514a9a3432")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'clear_target-request)))
  "Returns md5sum for a message object of type 'clear_target-request"
  "8d891785a3ff0d216ac0e1514a9a3432")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<clear_target-request>)))
  "Returns full string definition for message of type '<clear_target-request>"
  (cl:format cl:nil "bool clear~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'clear_target-request)))
  "Returns full string definition for message of type 'clear_target-request"
  (cl:format cl:nil "bool clear~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <clear_target-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <clear_target-request>))
  "Converts a ROS message object to a list"
  (cl:list 'clear_target-request
    (cl:cons ':clear (clear msg))
))
;//! \htmlinclude clear_target-response.msg.html

(cl:defclass <clear_target-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass clear_target-response (<clear_target-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <clear_target-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'clear_target-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name person_tracking-srv:<clear_target-response> is deprecated: use person_tracking-srv:clear_target-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <clear_target-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader person_tracking-srv:success-val is deprecated.  Use person_tracking-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <clear_target-response>) ostream)
  "Serializes a message object of type '<clear_target-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <clear_target-response>) istream)
  "Deserializes a message object of type '<clear_target-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<clear_target-response>)))
  "Returns string type for a service object of type '<clear_target-response>"
  "person_tracking/clear_targetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'clear_target-response)))
  "Returns string type for a service object of type 'clear_target-response"
  "person_tracking/clear_targetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<clear_target-response>)))
  "Returns md5sum for a message object of type '<clear_target-response>"
  "8d891785a3ff0d216ac0e1514a9a3432")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'clear_target-response)))
  "Returns md5sum for a message object of type 'clear_target-response"
  "8d891785a3ff0d216ac0e1514a9a3432")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<clear_target-response>)))
  "Returns full string definition for message of type '<clear_target-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'clear_target-response)))
  "Returns full string definition for message of type 'clear_target-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <clear_target-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <clear_target-response>))
  "Converts a ROS message object to a list"
  (cl:list 'clear_target-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'clear_target)))
  'clear_target-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'clear_target)))
  'clear_target-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'clear_target)))
  "Returns string type for a service object of type '<clear_target>"
  "person_tracking/clear_target")