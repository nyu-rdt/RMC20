;; Auto-generated. Do not edit!


(when (boundp 'rdt_localization::Orientation_Vector)
  (if (not (find-package "RDT_LOCALIZATION"))
    (make-package "RDT_LOCALIZATION"))
  (shadow 'Orientation_Vector (find-package "RDT_LOCALIZATION")))
(unless (find-package "RDT_LOCALIZATION::ORIENTATION_VECTOR")
  (make-package "RDT_LOCALIZATION::ORIENTATION_VECTOR"))

(in-package "ROS")
;;//! \htmlinclude Orientation_Vector.msg.html


(defclass rdt_localization::Orientation_Vector
  :super ros::object
  :slots (_robot_pose _target_zone _robot_speed ))

(defmethod rdt_localization::Orientation_Vector
  (:init
   (&key
    ((:robot_pose __robot_pose) (instance rdt_localization::Pose :init))
    ((:target_zone __target_zone) (instance rdt_localization::Location :init))
    ((:robot_speed __robot_speed) 0)
    )
   (send-super :init)
   (setq _robot_pose __robot_pose)
   (setq _target_zone __target_zone)
   (setq _robot_speed (round __robot_speed))
   self)
  (:robot_pose
   (&rest __robot_pose)
   (if (keywordp (car __robot_pose))
       (send* _robot_pose __robot_pose)
     (progn
       (if __robot_pose (setq _robot_pose (car __robot_pose)))
       _robot_pose)))
  (:target_zone
   (&rest __target_zone)
   (if (keywordp (car __target_zone))
       (send* _target_zone __target_zone)
     (progn
       (if __target_zone (setq _target_zone (car __target_zone)))
       _target_zone)))
  (:robot_speed
   (&optional __robot_speed)
   (if __robot_speed (setq _robot_speed __robot_speed)) _robot_speed)
  (:serialization-length
   ()
   (+
    ;; rdt_localization/Pose _robot_pose
    (send _robot_pose :serialization-length)
    ;; rdt_localization/Location _target_zone
    (send _target_zone :serialization-length)
    ;; int64 _robot_speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; rdt_localization/Pose _robot_pose
       (send _robot_pose :serialize s)
     ;; rdt_localization/Location _target_zone
       (send _target_zone :serialize s)
     ;; int64 _robot_speed
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _robot_speed (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _robot_speed) (= (length (_robot_speed . bv)) 2)) ;; bignum
              (write-long (ash (elt (_robot_speed . bv) 0) 0) s)
              (write-long (ash (elt (_robot_speed . bv) 1) -1) s))
             ((and (class _robot_speed) (= (length (_robot_speed . bv)) 1)) ;; big1
              (write-long (elt (_robot_speed . bv) 0) s)
              (write-long (if (>= _robot_speed 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _robot_speed s)(write-long (if (>= _robot_speed 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; rdt_localization/Pose _robot_pose
     (send _robot_pose :deserialize buf ptr-) (incf ptr- (send _robot_pose :serialization-length))
   ;; rdt_localization/Location _target_zone
     (send _target_zone :deserialize buf ptr-) (incf ptr- (send _target_zone :serialization-length))
   ;; int64 _robot_speed
#+(or :alpha :irix6 :x86_64)
      (setf _robot_speed (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _robot_speed (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(setf (get rdt_localization::Orientation_Vector :md5sum-) "32c0233ad09fff327409092d7163201c")
(setf (get rdt_localization::Orientation_Vector :datatype-) "rdt_localization/Orientation_Vector")
(setf (get rdt_localization::Orientation_Vector :definition-)
      "Pose robot_pose
Location target_zone
int64 robot_speed
================================================================================
MSG: rdt_localization/Pose
float32 x
float32 y
float32 orientation
================================================================================
MSG: rdt_localization/Location
float32 x
float32 y
")



(provide :rdt_localization/Orientation_Vector "32c0233ad09fff327409092d7163201c")


