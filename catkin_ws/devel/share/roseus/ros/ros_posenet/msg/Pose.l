;; Auto-generated. Do not edit!


(when (boundp 'ros_posenet::Pose)
  (if (not (find-package "ROS_POSENET"))
    (make-package "ROS_POSENET"))
  (shadow 'Pose (find-package "ROS_POSENET")))
(unless (find-package "ROS_POSENET::POSE")
  (make-package "ROS_POSENET::POSE"))

(in-package "ROS")
;;//! \htmlinclude Pose.msg.html


(defclass ros_posenet::Pose
  :super ros::object
  :slots (_keypoints ))

(defmethod ros_posenet::Pose
  (:init
   (&key
    ((:keypoints __keypoints) ())
    )
   (send-super :init)
   (setq _keypoints __keypoints)
   self)
  (:keypoints
   (&rest __keypoints)
   (if (keywordp (car __keypoints))
       (send* _keypoints __keypoints)
     (progn
       (if __keypoints (setq _keypoints (car __keypoints)))
       _keypoints)))
  (:serialization-length
   ()
   (+
    ;; ros_posenet/Keypoint[] _keypoints
    (apply #'+ (send-all _keypoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ros_posenet/Keypoint[] _keypoints
     (write-long (length _keypoints) s)
     (dolist (elem _keypoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ros_posenet/Keypoint[] _keypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _keypoints (let (r) (dotimes (i n) (push (instance ros_posenet::Keypoint :init) r)) r))
     (dolist (elem- _keypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ros_posenet::Pose :md5sum-) "3e476a684ef3c919da15f180dd754305")
(setf (get ros_posenet::Pose :datatype-) "ros_posenet/Pose")
(setf (get ros_posenet::Pose :definition-)
      "ros_posenet/Keypoint[] keypoints
================================================================================
MSG: ros_posenet/Keypoint
string part
float64 score
geometry_msgs/Point position
geometry_msgs/Point image_position

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :ros_posenet/Pose "3e476a684ef3c919da15f180dd754305")


