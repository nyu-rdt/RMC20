;; Auto-generated. Do not edit!


(when (boundp 'rdt_localization::Limb_Vector)
  (if (not (find-package "RDT_LOCALIZATION"))
    (make-package "RDT_LOCALIZATION"))
  (shadow 'Limb_Vector (find-package "RDT_LOCALIZATION")))
(unless (find-package "RDT_LOCALIZATION::LIMB_VECTOR")
  (make-package "RDT_LOCALIZATION::LIMB_VECTOR"))

(in-package "ROS")
;;//! \htmlinclude Limb_Vector.msg.html


(defclass rdt_localization::Limb_Vector
  :super ros::object
  :slots (_door _linActs_speed _arm_speed _drum_speed ))

(defmethod rdt_localization::Limb_Vector
  (:init
   (&key
    ((:door __door) nil)
    ((:linActs_speed __linActs_speed) 0)
    ((:arm_speed __arm_speed) 0)
    ((:drum_speed __drum_speed) 0)
    )
   (send-super :init)
   (setq _door __door)
   (setq _linActs_speed (round __linActs_speed))
   (setq _arm_speed (round __arm_speed))
   (setq _drum_speed (round __drum_speed))
   self)
  (:door
   (&optional __door)
   (if __door (setq _door __door)) _door)
  (:linActs_speed
   (&optional __linActs_speed)
   (if __linActs_speed (setq _linActs_speed __linActs_speed)) _linActs_speed)
  (:arm_speed
   (&optional __arm_speed)
   (if __arm_speed (setq _arm_speed __arm_speed)) _arm_speed)
  (:drum_speed
   (&optional __drum_speed)
   (if __drum_speed (setq _drum_speed __drum_speed)) _drum_speed)
  (:serialization-length
   ()
   (+
    ;; bool _door
    1
    ;; int64 _linActs_speed
    8
    ;; int64 _arm_speed
    8
    ;; int64 _drum_speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _door
       (if _door (write-byte -1 s) (write-byte 0 s))
     ;; int64 _linActs_speed
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _linActs_speed (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _linActs_speed) (= (length (_linActs_speed . bv)) 2)) ;; bignum
              (write-long (ash (elt (_linActs_speed . bv) 0) 0) s)
              (write-long (ash (elt (_linActs_speed . bv) 1) -1) s))
             ((and (class _linActs_speed) (= (length (_linActs_speed . bv)) 1)) ;; big1
              (write-long (elt (_linActs_speed . bv) 0) s)
              (write-long (if (>= _linActs_speed 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _linActs_speed s)(write-long (if (>= _linActs_speed 0) 0 #xffffffff) s)))
     ;; int64 _arm_speed
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _arm_speed (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _arm_speed) (= (length (_arm_speed . bv)) 2)) ;; bignum
              (write-long (ash (elt (_arm_speed . bv) 0) 0) s)
              (write-long (ash (elt (_arm_speed . bv) 1) -1) s))
             ((and (class _arm_speed) (= (length (_arm_speed . bv)) 1)) ;; big1
              (write-long (elt (_arm_speed . bv) 0) s)
              (write-long (if (>= _arm_speed 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _arm_speed s)(write-long (if (>= _arm_speed 0) 0 #xffffffff) s)))
     ;; int64 _drum_speed
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _drum_speed (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _drum_speed) (= (length (_drum_speed . bv)) 2)) ;; bignum
              (write-long (ash (elt (_drum_speed . bv) 0) 0) s)
              (write-long (ash (elt (_drum_speed . bv) 1) -1) s))
             ((and (class _drum_speed) (= (length (_drum_speed . bv)) 1)) ;; big1
              (write-long (elt (_drum_speed . bv) 0) s)
              (write-long (if (>= _drum_speed 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _drum_speed s)(write-long (if (>= _drum_speed 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _door
     (setq _door (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int64 _linActs_speed
#+(or :alpha :irix6 :x86_64)
      (setf _linActs_speed (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _linActs_speed (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _arm_speed
#+(or :alpha :irix6 :x86_64)
      (setf _arm_speed (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _arm_speed (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _drum_speed
#+(or :alpha :irix6 :x86_64)
      (setf _drum_speed (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _drum_speed (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(setf (get rdt_localization::Limb_Vector :md5sum-) "95198abd0938eea51c580fdd9748821b")
(setf (get rdt_localization::Limb_Vector :datatype-) "rdt_localization/Limb_Vector")
(setf (get rdt_localization::Limb_Vector :definition-)
      "bool door
int64 linActs_speed
int64 arm_speed
int64 drum_speed
")



(provide :rdt_localization/Limb_Vector "95198abd0938eea51c580fdd9748821b")


