;; Auto-generated. Do not edit!


(when (boundp 'framework::IntStr)
  (if (not (find-package "FRAMEWORK"))
    (make-package "FRAMEWORK"))
  (shadow 'IntStr (find-package "FRAMEWORK")))
(unless (find-package "FRAMEWORK::INTSTR")
  (make-package "FRAMEWORK::INTSTR"))
(unless (find-package "FRAMEWORK::INTSTRREQUEST")
  (make-package "FRAMEWORK::INTSTRREQUEST"))
(unless (find-package "FRAMEWORK::INTSTRRESPONSE")
  (make-package "FRAMEWORK::INTSTRRESPONSE"))

(in-package "ROS")





(defclass framework::IntStrRequest
  :super ros::object
  :slots (_ID _data ))

(defmethod framework::IntStrRequest
  (:init
   (&key
    ((:ID __ID) 0)
    ((:data __data) "")
    )
   (send-super :init)
   (setq _ID (round __ID))
   (setq _data (string __data))
   self)
  (:ID
   (&optional __ID)
   (if __ID (setq _ID __ID)) _ID)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int32 _ID
    4
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _ID
       (write-long _ID s)
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _ID
     (setq _ID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass framework::IntStrResponse
  :super ros::object
  :slots (_success ))

(defmethod framework::IntStrResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
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

(defclass framework::IntStr
  :super ros::object
  :slots ())

(setf (get framework::IntStr :md5sum-) "e4e1a089454a32166064b947dbe98aff")
(setf (get framework::IntStr :datatype-) "framework/IntStr")
(setf (get framework::IntStr :request) framework::IntStrRequest)
(setf (get framework::IntStr :response) framework::IntStrResponse)

(defmethod framework::IntStrRequest
  (:response () (instance framework::IntStrResponse :init)))

(setf (get framework::IntStrRequest :md5sum-) "e4e1a089454a32166064b947dbe98aff")
(setf (get framework::IntStrRequest :datatype-) "framework/IntStrRequest")
(setf (get framework::IntStrRequest :definition-)
      "int32 ID
string data
---
bool success

")

(setf (get framework::IntStrResponse :md5sum-) "e4e1a089454a32166064b947dbe98aff")
(setf (get framework::IntStrResponse :datatype-) "framework/IntStrResponse")
(setf (get framework::IntStrResponse :definition-)
      "int32 ID
string data
---
bool success

")



(provide :framework/IntStr "e4e1a089454a32166064b947dbe98aff")


