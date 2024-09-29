
(cl:in-package :asdf)

(defsystem "ros_posenet-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Human" :depends-on ("_package_Human"))
    (:file "_package_Human" :depends-on ("_package"))
    (:file "Keypoint" :depends-on ("_package_Keypoint"))
    (:file "_package_Keypoint" :depends-on ("_package"))
    (:file "Pose" :depends-on ("_package_Pose"))
    (:file "_package_Pose" :depends-on ("_package"))
    (:file "Poses" :depends-on ("_package_Poses"))
    (:file "_package_Poses" :depends-on ("_package"))
    (:file "aPoses" :depends-on ("_package_aPoses"))
    (:file "_package_aPoses" :depends-on ("_package"))
  ))