;; Auto-generated. Do not edit!


(when (boundp 'framework::StrInt)
  (if (not (find-package "FRAMEWORK"))
    (make-package "FRAMEWORK"))
  (shadow 'StrInt (find-package "FRAMEWORK")))
(unless (find-package "FRAMEWORK::STRINT")
  (make-package "FRAMEWORK::STRINT"))
(unless (find-package "FRAMEWORK::STRINTREQUEST")
  (make-package "FRAMEWORK::STRINTREQUEST"))
(unless (find-package "FRAMEWORK::STRINTRESPONSE")
  (make-package "FRAMEWORK::STRINTRESPONSE"))

(in-package "ROS")





(defclass framework::StrIntRequest
  :super ros::object
  :slots (_data ))

(defmethod framework::StrIntRequest
  (:init
   (&key
    ((:data __data) "")
    )
   (send-super :init)
   (setq _data (string __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass framework::StrIntResponse
  :super ros::object
  :slots (_data ))

(defmethod framework::StrIntResponse
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _data
       (write-long _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _data
     (setq _data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass framework::StrInt
  :super ros::object
  :slots ())

(setf (get framework::StrInt :md5sum-) "3df8c623795dfc3d210f796f50fd2d3c")
(setf (get framework::StrInt :datatype-) "framework/StrInt")
(setf (get framework::StrInt :request) framework::StrIntRequest)
(setf (get framework::StrInt :response) framework::StrIntResponse)

(defmethod framework::StrIntRequest
  (:response () (instance framework::StrIntResponse :init)))

(setf (get framework::StrIntRequest :md5sum-) "3df8c623795dfc3d210f796f50fd2d3c")
(setf (get framework::StrIntRequest :datatype-) "framework/StrIntRequest")
(setf (get framework::StrIntRequest :definition-)
      "string data
---
int32 data

")

(setf (get framework::StrIntResponse :md5sum-) "3df8c623795dfc3d210f796f50fd2d3c")
(setf (get framework::StrIntResponse :datatype-) "framework/StrIntResponse")
(setf (get framework::StrIntResponse :definition-)
      "string data
---
int32 data

")



(provide :framework/StrInt "3df8c623795dfc3d210f796f50fd2d3c")


