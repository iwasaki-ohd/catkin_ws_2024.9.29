;; Auto-generated. Do not edit!


(when (boundp 'motpy_ros::motpy_bbox)
  (if (not (find-package "MOTPY_ROS"))
    (make-package "MOTPY_ROS"))
  (shadow 'motpy_bbox (find-package "MOTPY_ROS")))
(unless (find-package "MOTPY_ROS::MOTPY_BBOX")
  (make-package "MOTPY_ROS::MOTPY_BBOX"))
(unless (find-package "MOTPY_ROS::MOTPY_BBOXREQUEST")
  (make-package "MOTPY_ROS::MOTPY_BBOXREQUEST"))
(unless (find-package "MOTPY_ROS::MOTPY_BBOXRESPONSE")
  (make-package "MOTPY_ROS::MOTPY_BBOXRESPONSE"))

(in-package "ROS")

(if (not (find-package "DARKNET_ROS_MSGS"))
  (ros::roseus-add-msgs "darknet_ros_msgs"))


(if (not (find-package "DARKNET_ROS_MSGS"))
  (ros::roseus-add-msgs "darknet_ros_msgs"))


(defclass motpy_ros::motpy_bboxRequest
  :super ros::object
  :slots (_input_bboxes ))

(defmethod motpy_ros::motpy_bboxRequest
  (:init
   (&key
    ((:input_bboxes __input_bboxes) (instance darknet_ros_msgs::BoundingBoxes :init))
    )
   (send-super :init)
   (setq _input_bboxes __input_bboxes)
   self)
  (:input_bboxes
   (&rest __input_bboxes)
   (if (keywordp (car __input_bboxes))
       (send* _input_bboxes __input_bboxes)
     (progn
       (if __input_bboxes (setq _input_bboxes (car __input_bboxes)))
       _input_bboxes)))
  (:serialization-length
   ()
   (+
    ;; darknet_ros_msgs/BoundingBoxes _input_bboxes
    (send _input_bboxes :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; darknet_ros_msgs/BoundingBoxes _input_bboxes
       (send _input_bboxes :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; darknet_ros_msgs/BoundingBoxes _input_bboxes
     (send _input_bboxes :deserialize buf ptr-) (incf ptr- (send _input_bboxes :serialization-length))
   ;;
   self)
  )

(defclass motpy_ros::motpy_bboxResponse
  :super ros::object
  :slots (_tracking_bboxes ))

(defmethod motpy_ros::motpy_bboxResponse
  (:init
   (&key
    ((:tracking_bboxes __tracking_bboxes) (instance darknet_ros_msgs::BoundingBoxes :init))
    )
   (send-super :init)
   (setq _tracking_bboxes __tracking_bboxes)
   self)
  (:tracking_bboxes
   (&rest __tracking_bboxes)
   (if (keywordp (car __tracking_bboxes))
       (send* _tracking_bboxes __tracking_bboxes)
     (progn
       (if __tracking_bboxes (setq _tracking_bboxes (car __tracking_bboxes)))
       _tracking_bboxes)))
  (:serialization-length
   ()
   (+
    ;; darknet_ros_msgs/BoundingBoxes _tracking_bboxes
    (send _tracking_bboxes :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; darknet_ros_msgs/BoundingBoxes _tracking_bboxes
       (send _tracking_bboxes :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; darknet_ros_msgs/BoundingBoxes _tracking_bboxes
     (send _tracking_bboxes :deserialize buf ptr-) (incf ptr- (send _tracking_bboxes :serialization-length))
   ;;
   self)
  )

(defclass motpy_ros::motpy_bbox
  :super ros::object
  :slots ())

(setf (get motpy_ros::motpy_bbox :md5sum-) "65b9571f1ed7828d4fcc56bef7d7664b")
(setf (get motpy_ros::motpy_bbox :datatype-) "motpy_ros/motpy_bbox")
(setf (get motpy_ros::motpy_bbox :request) motpy_ros::motpy_bboxRequest)
(setf (get motpy_ros::motpy_bbox :response) motpy_ros::motpy_bboxResponse)

(defmethod motpy_ros::motpy_bboxRequest
  (:response () (instance motpy_ros::motpy_bboxResponse :init)))

(setf (get motpy_ros::motpy_bboxRequest :md5sum-) "65b9571f1ed7828d4fcc56bef7d7664b")
(setf (get motpy_ros::motpy_bboxRequest :datatype-) "motpy_ros/motpy_bboxRequest")
(setf (get motpy_ros::motpy_bboxRequest :definition-)
      "darknet_ros_msgs/BoundingBoxes input_bboxes


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
---

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
")

(setf (get motpy_ros::motpy_bboxResponse :md5sum-) "65b9571f1ed7828d4fcc56bef7d7664b")
(setf (get motpy_ros::motpy_bboxResponse :datatype-) "motpy_ros/motpy_bboxResponse")
(setf (get motpy_ros::motpy_bboxResponse :definition-)
      "darknet_ros_msgs/BoundingBoxes input_bboxes


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
---

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
")



(provide :motpy_ros/motpy_bbox "65b9571f1ed7828d4fcc56bef7d7664b")


