;; Auto-generated. Do not edit!


(when (boundp 'ros_posenet::Poses)
  (if (not (find-package "ROS_POSENET"))
    (make-package "ROS_POSENET"))
  (shadow 'Poses (find-package "ROS_POSENET")))
(unless (find-package "ROS_POSENET::POSES")
  (make-package "ROS_POSENET::POSES"))

(in-package "ROS")
;;//! \htmlinclude Poses.msg.html


(defclass ros_posenet::Poses
  :super ros::object
  :slots (_poses ))

(defmethod ros_posenet::Poses
  (:init
   (&key
    ((:poses __poses) ())
    )
   (send-super :init)
   (setq _poses __poses)
   self)
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:serialization-length
   ()
   (+
    ;; ros_posenet/Pose[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ros_posenet/Pose[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ros_posenet/Pose[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance ros_posenet::Pose :init) r)) r))
     (dolist (elem- _poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ros_posenet::Poses :md5sum-) "0e46ae84d5c891cad2f3c5e03526bd70")
(setf (get ros_posenet::Poses :datatype-) "ros_posenet/Poses")
(setf (get ros_posenet::Poses :definition-)
      "Pose[] poses
================================================================================
MSG: ros_posenet/Pose
ros_posenet/Keypoint[] keypoints
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



(provide :ros_posenet/Poses "0e46ae84d5c891cad2f3c5e03526bd70")


