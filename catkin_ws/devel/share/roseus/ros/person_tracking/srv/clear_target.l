;; Auto-generated. Do not edit!


(when (boundp 'person_tracking::clear_target)
  (if (not (find-package "PERSON_TRACKING"))
    (make-package "PERSON_TRACKING"))
  (shadow 'clear_target (find-package "PERSON_TRACKING")))
(unless (find-package "PERSON_TRACKING::CLEAR_TARGET")
  (make-package "PERSON_TRACKING::CLEAR_TARGET"))
(unless (find-package "PERSON_TRACKING::CLEAR_TARGETREQUEST")
  (make-package "PERSON_TRACKING::CLEAR_TARGETREQUEST"))
(unless (find-package "PERSON_TRACKING::CLEAR_TARGETRESPONSE")
  (make-package "PERSON_TRACKING::CLEAR_TARGETRESPONSE"))

(in-package "ROS")





(defclass person_tracking::clear_targetRequest
  :super ros::object
  :slots (_clear ))

(defmethod person_tracking::clear_targetRequest
  (:init
   (&key
    ((:clear __clear) nil)
    )
   (send-super :init)
   (setq _clear __clear)
   self)
  (:clear
   (&optional (__clear :null))
   (if (not (eq __clear :null)) (setq _clear __clear)) _clear)
  (:serialization-length
   ()
   (+
    ;; bool _clear
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _clear
       (if _clear (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _clear
     (setq _clear (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass person_tracking::clear_targetResponse
  :super ros::object
  :slots (_success ))

(defmethod person_tracking::clear_targetResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass person_tracking::clear_target
  :super ros::object
  :slots ())

(setf (get person_tracking::clear_target :md5sum-) "8d891785a3ff0d216ac0e1514a9a3432")
(setf (get person_tracking::clear_target :datatype-) "person_tracking/clear_target")
(setf (get person_tracking::clear_target :request) person_tracking::clear_targetRequest)
(setf (get person_tracking::clear_target :response) person_tracking::clear_targetResponse)

(defmethod person_tracking::clear_targetRequest
  (:response () (instance person_tracking::clear_targetResponse :init)))

(setf (get person_tracking::clear_targetRequest :md5sum-) "8d891785a3ff0d216ac0e1514a9a3432")
(setf (get person_tracking::clear_targetRequest :datatype-) "person_tracking/clear_targetRequest")
(setf (get person_tracking::clear_targetRequest :definition-)
      "bool clear
---
bool success
")

(setf (get person_tracking::clear_targetResponse :md5sum-) "8d891785a3ff0d216ac0e1514a9a3432")
(setf (get person_tracking::clear_targetResponse :datatype-) "person_tracking/clear_targetResponse")
(setf (get person_tracking::clear_targetResponse :definition-)
      "bool clear
---
bool success
")



(provide :person_tracking/clear_target "8d891785a3ff0d216ac0e1514a9a3432")


