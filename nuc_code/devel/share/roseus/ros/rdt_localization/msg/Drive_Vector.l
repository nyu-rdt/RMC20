;; Auto-generated. Do not edit!


(when (boundp 'rdt_localization::Drive_Vector)
  (if (not (find-package "RDT_LOCALIZATION"))
    (make-package "RDT_LOCALIZATION"))
  (shadow 'Drive_Vector (find-package "RDT_LOCALIZATION")))
(unless (find-package "RDT_LOCALIZATION::DRIVE_VECTOR")
  (make-package "RDT_LOCALIZATION::DRIVE_VECTOR"))

(in-package "ROS")
;;//! \htmlinclude Drive_Vector.msg.html


(defclass rdt_localization::Drive_Vector
  :super ros::object
  :slots (_robot_spd _offset_driveMode ))

(defmethod rdt_localization::Drive_Vector
  (:init
   (&key
    ((:robot_spd __robot_spd) 0)
    ((:offset_driveMode __offset_driveMode) 0)
    )
   (send-super :init)
   (setq _robot_spd (round __robot_spd))
   (setq _offset_driveMode (round __offset_driveMode))
   self)
  (:robot_spd
   (&optional __robot_spd)
   (if __robot_spd (setq _robot_spd __robot_spd)) _robot_spd)
  (:offset_driveMode
   (&optional __offset_driveMode)
   (if __offset_driveMode (setq _offset_driveMode __offset_driveMode)) _offset_driveMode)
  (:serialization-length
   ()
   (+
    ;; int64 _robot_spd
    8
    ;; int64 _offset_driveMode
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _robot_spd
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _robot_spd (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _robot_spd) (= (length (_robot_spd . bv)) 2)) ;; bignum
              (write-long (ash (elt (_robot_spd . bv) 0) 0) s)
              (write-long (ash (elt (_robot_spd . bv) 1) -1) s))
             ((and (class _robot_spd) (= (length (_robot_spd . bv)) 1)) ;; big1
              (write-long (elt (_robot_spd . bv) 0) s)
              (write-long (if (>= _robot_spd 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _robot_spd s)(write-long (if (>= _robot_spd 0) 0 #xffffffff) s)))
     ;; int64 _offset_driveMode
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _offset_driveMode (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _offset_driveMode) (= (length (_offset_driveMode . bv)) 2)) ;; bignum
              (write-long (ash (elt (_offset_driveMode . bv) 0) 0) s)
              (write-long (ash (elt (_offset_driveMode . bv) 1) -1) s))
             ((and (class _offset_driveMode) (= (length (_offset_driveMode . bv)) 1)) ;; big1
              (write-long (elt (_offset_driveMode . bv) 0) s)
              (write-long (if (>= _offset_driveMode 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _offset_driveMode s)(write-long (if (>= _offset_driveMode 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _robot_spd
#+(or :alpha :irix6 :x86_64)
      (setf _robot_spd (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _robot_spd (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _offset_driveMode
#+(or :alpha :irix6 :x86_64)
      (setf _offset_driveMode (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _offset_driveMode (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get rdt_localization::Drive_Vector :md5sum-) "9bb2e6492d363aa84cce71e895f95bc0")
(setf (get rdt_localization::Drive_Vector :datatype-) "rdt_localization/Drive_Vector")
(setf (get rdt_localization::Drive_Vector :definition-)
      "int64 robot_spd
int64 offset_driveMode
")



(provide :rdt_localization/Drive_Vector "9bb2e6492d363aa84cce71e895f95bc0")


