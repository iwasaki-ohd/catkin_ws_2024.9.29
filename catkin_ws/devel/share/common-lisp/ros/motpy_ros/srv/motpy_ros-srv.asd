
(cl:in-package :asdf)

(defsystem "motpy_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :darknet_ros_msgs-msg
)
  :components ((:file "_package")
    (:file "motpy_bbox" :depends-on ("_package_motpy_bbox"))
    (:file "_package_motpy_bbox" :depends-on ("_package"))
  ))