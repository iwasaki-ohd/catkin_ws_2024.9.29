;; Auto-generated. Do not edit!


(when (boundp 'person_tracking::update_tracker)
  (if (not (find-package "PERSON_TRACKING"))
    (make-package "PERSON_TRACKING"))
  (shadow 'update_tracker (find-package "PERSON_TRACKING")))
(unless (find-package "PERSON_TRACKING::UPDATE_TRACKER")
  (make-package "PERSON_TRACKING::UPDATE_TRACKER"))
(unless (find-package "PERSON_TRACKING::UPDATE_TRACKERREQUEST")
  (make-package "PERSON_TRACKING::UPDATE_TRACKERREQUEST"))
(unless (find-package "PERSON_TRACKING::UPDATE_TRACKERRESPONSE")
  (make-package "PERSON_TRACKING::UPDATE_TRACKERRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass person_tracking::update_trackerRequest
  :super ros::object
  :slots (_img ))

(defmethod person_tracking::update_trackerRequest
  (:init
   (&key
    ((:img __img) (instance sensor_msgs::CompressedImage :init))
    )
   (send-super :init)
   (setq _img __img)
   self)
  (:img
   (&rest __img)
   (if (keywordp (car __img))
       (send* _img __img)
     (progn
       (if __img (setq _img (car __img)))
       _img)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/CompressedImage _img
    (send _img :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/CompressedImage _img
       (send _img :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/CompressedImage _img
     (send _img :deserialize buf ptr-) (incf ptr- (send _img :serialization-length))
   ;;
   self)
  )

(defclass person_tracking::update_trackerResponse
  :super ros::object
  :slots (_coordinates ))

(defmethod person_tracking::update_trackerResponse
  (:init
   (&key
    ((:coordinates __coordinates) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _coordinates __coordinates)
   self)
  (:coordinates
   (&rest __coordinates)
   (if (keywordp (car __coordinates))
       (send* _coordinates __coordinates)
     (progn
       (if __coordinates (setq _coordinates (car __coordinates)))
       _coordinates)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _coordinates
    (send _coordinates :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _coordinates
       (send _coordinates :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _coordinates
     (send _coordinates :deserialize buf ptr-) (incf ptr- (send _coordinates :serialization-length))
   ;;
   self)
  )

(defclass person_tracking::update_tracker
  :super ros::object
  :slots ())

(setf (get person_tracking::update_tracker :md5sum-) "4a50c39106e487c8fc1d6f02a4dd5e96")
(setf (get person_tracking::update_tracker :datatype-) "person_tracking/update_tracker")
(setf (get person_tracking::update_tracker :request) person_tracking::update_trackerRequest)
(setf (get person_tracking::update_tracker :response) person_tracking::update_trackerResponse)

(defmethod person_tracking::update_trackerRequest
  (:response () (instance person_tracking::update_trackerResponse :init)))

(setf (get person_tracking::update_trackerRequest :md5sum-) "4a50c39106e487c8fc1d6f02a4dd5e96")
(setf (get person_tracking::update_trackerRequest :datatype-) "person_tracking/update_trackerRequest")
(setf (get person_tracking::update_trackerRequest :definition-)
      "sensor_msgs/CompressedImage img

================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image

string format        # Specifies the format of the data
                     #   Acceptable values:
                     #     jpeg, png
uint8[] data         # Compressed image buffer

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
---
geometry_msgs/Point coordinates

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get person_tracking::update_trackerResponse :md5sum-) "4a50c39106e487c8fc1d6f02a4dd5e96")
(setf (get person_tracking::update_trackerResponse :datatype-) "person_tracking/update_trackerResponse")
(setf (get person_tracking::update_trackerResponse :definition-)
      "sensor_msgs/CompressedImage img

================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image

string format        # Specifies the format of the data
                     #   Acceptable values:
                     #     jpeg, png
uint8[] data         # Compressed image buffer

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
---
geometry_msgs/Point coordinates

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :person_tracking/update_tracker "4a50c39106e487c8fc1d6f02a4dd5e96")


