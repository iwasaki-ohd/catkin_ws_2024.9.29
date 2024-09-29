
(cl:in-package :asdf)

(defsystem "person_tracking-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "choose_target" :depends-on ("_package_choose_target"))
    (:file "_package_choose_target" :depends-on ("_package"))
    (:file "clear_target" :depends-on ("_package_clear_target"))
    (:file "_package_clear_target" :depends-on ("_package"))
    (:file "update_tracker" :depends-on ("_package_update_tracker"))
    (:file "_package_update_tracker" :depends-on ("_package"))
  ))