;; Auto-generated. Do not edit!


(when (boundp 'person_tracking::choose_target)
  (if (not (find-package "PERSON_TRACKING"))
    (make-package "PERSON_TRACKING"))
  (shadow 'choose_target (find-package "PERSON_TRACKING")))
(unless (find-package "PERSON_TRACKING::CHOOSE_TARGET")
  (make-package "PERSON_TRACKING::CHOOSE_TARGET"))
(unless (find-package "PERSON_TRACKING::CHOOSE_TARGETREQUEST")
  (make-package "PERSON_TRACKING::CHOOSE_TARGETREQUEST"))
(unless (find-package "PERSON_TRACKING::CHOOSE_TARGETRESPONSE")
  (make-package "PERSON_TRACKING::CHOOSE_TARGETRESPONSE"))

(in-package "ROS")





(defclass person_tracking::choose_targetRequest
  :super ros::object
  :slots (_target ))

(defmethod person_tracking::choose_targetRequest
  (:init
   (&key
    ((:target __target) 0)
    )
   (send-super :init)
   (setq _target (round __target))
   self)
  (:target
   (&optional __target)
   (if __target (setq _target __target)) _target)
  (:serialization-length
   ()
   (+
    ;; int32 _target
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _target
       (write-long _target s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _target
     (setq _target (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass person_tracking::choose_targetResponse
  :super ros::object
  :slots (_success ))

(defmethod person_tracking::choose_targetResponse
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

(defclass person_tracking::choose_target
  :super ros::object
  :slots ())

(setf (get person_tracking::choose_target :md5sum-) "02caa2968e58f6cef6489327a5fdc1c4")
(setf (get person_tracking::choose_target :datatype-) "person_tracking/choose_target")
(setf (get person_tracking::choose_target :request) person_tracking::choose_targetRequest)
(setf (get person_tracking::choose_target :response) person_tracking::choose_targetResponse)

(defmethod person_tracking::choose_targetRequest
  (:response () (instance person_tracking::choose_targetResponse :init)))

(setf (get person_tracking::choose_targetRequest :md5sum-) "02caa2968e58f6cef6489327a5fdc1c4")
(setf (get person_tracking::choose_targetRequest :datatype-) "person_tracking/choose_targetRequest")
(setf (get person_tracking::choose_targetRequest :definition-)
      "int32 target
---
bool success
")

(setf (get person_tracking::choose_targetResponse :md5sum-) "02caa2968e58f6cef6489327a5fdc1c4")
(setf (get person_tracking::choose_targetResponse :datatype-) "person_tracking/choose_targetResponse")
(setf (get person_tracking::choose_targetResponse :definition-)
      "int32 target
---
bool success
")



(provide :person_tracking/choose_target "02caa2968e58f6cef6489327a5fdc1c4")


