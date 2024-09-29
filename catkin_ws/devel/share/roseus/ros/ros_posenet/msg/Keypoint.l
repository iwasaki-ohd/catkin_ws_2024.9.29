;; Auto-generated. Do not edit!


(when (boundp 'ros_posenet::Keypoint)
  (if (not (find-package "ROS_POSENET"))
    (make-package "ROS_POSENET"))
  (shadow 'Keypoint (find-package "ROS_POSENET")))
(unless (find-package "ROS_POSENET::KEYPOINT")
  (make-package "ROS_POSENET::KEYPOINT"))

(in-package "ROS")
;;//! \htmlinclude Keypoint.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ros_posenet::Keypoint
  :super ros::object
  :slots (_part _score _position _image_position ))

(defmethod ros_posenet::Keypoint
  (:init
   (&key
    ((:part __part) "")
    ((:score __score) 0.0)
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:image_position __image_position) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _part (string __part))
   (setq _score (float __score))
   (setq _position __position)
   (setq _image_position __image_position)
   self)
  (:part
   (&optional __part)
   (if __part (setq _part __part)) _part)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:image_position
   (&rest __image_position)
   (if (keywordp (car __image_position))
       (send* _image_position __image_position)
     (progn
       (if __image_position (setq _image_position (car __image_position)))
       _image_position)))
  (:serialization-length
   ()
   (+
    ;; string _part
    4 (length _part)
    ;; float64 _score
    8
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; geometry_msgs/Point _image_position
    (send _image_position :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _part
       (write-long (length _part) s) (princ _part s)
     ;; float64 _score
       (sys::poke _score (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; geometry_msgs/Point _image_position
       (send _image_position :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _part
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _part (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _score
     (setq _score (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Point _image_position
     (send _image_position :deserialize buf ptr-) (incf ptr- (send _image_position :serialization-length))
   ;;
   self)
  )

(setf (get ros_posenet::Keypoint :md5sum-) "ed7553fef5f9989af54ac621aed181dd")
(setf (get ros_posenet::Keypoint :datatype-) "ros_posenet/Keypoint")
(setf (get ros_posenet::Keypoint :definition-)
      "string part
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



(provide :ros_posenet/Keypoint "ed7553fef5f9989af54ac621aed181dd")


