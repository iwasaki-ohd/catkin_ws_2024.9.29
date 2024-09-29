;; Auto-generated. Do not edit!


(when (boundp 'ros_posenet::aPoses)
  (if (not (find-package "ROS_POSENET"))
    (make-package "ROS_POSENET"))
  (shadow 'aPoses (find-package "ROS_POSENET")))
(unless (find-package "ROS_POSENET::APOSES")
  (make-package "ROS_POSENET::APOSES"))

(in-package "ROS")
;;//! \htmlinclude aPoses.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ros_posenet::aPoses
  :super ros::object
  :slots (_part _position ))

(defmethod ros_posenet::aPoses
  (:init
   (&key
    ((:part __part) "")
    ((:position __position) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _part (string __part))
   (setq _position __position)
   self)
  (:part
   (&optional __part)
   (if __part (setq _part __part)) _part)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:serialization-length
   ()
   (+
    ;; string _part
    4 (length _part)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _part
       (write-long (length _part) s) (princ _part s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _part
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _part (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;;
   self)
  )

(setf (get ros_posenet::aPoses :md5sum-) "48f6b649951fc40ababaa77312f83e65")
(setf (get ros_posenet::aPoses :datatype-) "ros_posenet/aPoses")
(setf (get ros_posenet::aPoses :definition-)
      "string part
geometry_msgs/Point position

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :ros_posenet/aPoses "48f6b649951fc40ababaa77312f83e65")


