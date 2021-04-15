;; Auto-generated. Do not edit!


(when (boundp 'rdt_localization::Obstacle)
  (if (not (find-package "RDT_LOCALIZATION"))
    (make-package "RDT_LOCALIZATION"))
  (shadow 'Obstacle (find-package "RDT_LOCALIZATION")))
(unless (find-package "RDT_LOCALIZATION::OBSTACLE")
  (make-package "RDT_LOCALIZATION::OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude Obstacle.msg.html


(defclass rdt_localization::Obstacle
  :super ros::object
  :slots (_front_left _front_right _front_left_mid _front_right_mid _depo_front _depo_back ))

(defmethod rdt_localization::Obstacle
  (:init
   (&key
    ((:front_left __front_left) 0)
    ((:front_right __front_right) 0)
    ((:front_left_mid __front_left_mid) 0)
    ((:front_right_mid __front_right_mid) 0)
    ((:depo_front __depo_front) 0)
    ((:depo_back __depo_back) 0)
    )
   (send-super :init)
   (setq _front_left (round __front_left))
   (setq _front_right (round __front_right))
   (setq _front_left_mid (round __front_left_mid))
   (setq _front_right_mid (round __front_right_mid))
   (setq _depo_front (round __depo_front))
   (setq _depo_back (round __depo_back))
   self)
  (:front_left
   (&optional __front_left)
   (if __front_left (setq _front_left __front_left)) _front_left)
  (:front_right
   (&optional __front_right)
   (if __front_right (setq _front_right __front_right)) _front_right)
  (:front_left_mid
   (&optional __front_left_mid)
   (if __front_left_mid (setq _front_left_mid __front_left_mid)) _front_left_mid)
  (:front_right_mid
   (&optional __front_right_mid)
   (if __front_right_mid (setq _front_right_mid __front_right_mid)) _front_right_mid)
  (:depo_front
   (&optional __depo_front)
   (if __depo_front (setq _depo_front __depo_front)) _depo_front)
  (:depo_back
   (&optional __depo_back)
   (if __depo_back (setq _depo_back __depo_back)) _depo_back)
  (:serialization-length
   ()
   (+
    ;; int64 _front_left
    8
    ;; int64 _front_right
    8
    ;; int64 _front_left_mid
    8
    ;; int64 _front_right_mid
    8
    ;; int64 _depo_front
    8
    ;; int64 _depo_back
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _front_left
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _front_left (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _front_left) (= (length (_front_left . bv)) 2)) ;; bignum
              (write-long (ash (elt (_front_left . bv) 0) 0) s)
              (write-long (ash (elt (_front_left . bv) 1) -1) s))
             ((and (class _front_left) (= (length (_front_left . bv)) 1)) ;; big1
              (write-long (elt (_front_left . bv) 0) s)
              (write-long (if (>= _front_left 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _front_left s)(write-long (if (>= _front_left 0) 0 #xffffffff) s)))
     ;; int64 _front_right
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _front_right (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _front_right) (= (length (_front_right . bv)) 2)) ;; bignum
              (write-long (ash (elt (_front_right . bv) 0) 0) s)
              (write-long (ash (elt (_front_right . bv) 1) -1) s))
             ((and (class _front_right) (= (length (_front_right . bv)) 1)) ;; big1
              (write-long (elt (_front_right . bv) 0) s)
              (write-long (if (>= _front_right 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _front_right s)(write-long (if (>= _front_right 0) 0 #xffffffff) s)))
     ;; int64 _front_left_mid
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _front_left_mid (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _front_left_mid) (= (length (_front_left_mid . bv)) 2)) ;; bignum
              (write-long (ash (elt (_front_left_mid . bv) 0) 0) s)
              (write-long (ash (elt (_front_left_mid . bv) 1) -1) s))
             ((and (class _front_left_mid) (= (length (_front_left_mid . bv)) 1)) ;; big1
              (write-long (elt (_front_left_mid . bv) 0) s)
              (write-long (if (>= _front_left_mid 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _front_left_mid s)(write-long (if (>= _front_left_mid 0) 0 #xffffffff) s)))
     ;; int64 _front_right_mid
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _front_right_mid (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _front_right_mid) (= (length (_front_right_mid . bv)) 2)) ;; bignum
              (write-long (ash (elt (_front_right_mid . bv) 0) 0) s)
              (write-long (ash (elt (_front_right_mid . bv) 1) -1) s))
             ((and (class _front_right_mid) (= (length (_front_right_mid . bv)) 1)) ;; big1
              (write-long (elt (_front_right_mid . bv) 0) s)
              (write-long (if (>= _front_right_mid 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _front_right_mid s)(write-long (if (>= _front_right_mid 0) 0 #xffffffff) s)))
     ;; int64 _depo_front
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _depo_front (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _depo_front) (= (length (_depo_front . bv)) 2)) ;; bignum
              (write-long (ash (elt (_depo_front . bv) 0) 0) s)
              (write-long (ash (elt (_depo_front . bv) 1) -1) s))
             ((and (class _depo_front) (= (length (_depo_front . bv)) 1)) ;; big1
              (write-long (elt (_depo_front . bv) 0) s)
              (write-long (if (>= _depo_front 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _depo_front s)(write-long (if (>= _depo_front 0) 0 #xffffffff) s)))
     ;; int64 _depo_back
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _depo_back (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _depo_back) (= (length (_depo_back . bv)) 2)) ;; bignum
              (write-long (ash (elt (_depo_back . bv) 0) 0) s)
              (write-long (ash (elt (_depo_back . bv) 1) -1) s))
             ((and (class _depo_back) (= (length (_depo_back . bv)) 1)) ;; big1
              (write-long (elt (_depo_back . bv) 0) s)
              (write-long (if (>= _depo_back 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _depo_back s)(write-long (if (>= _depo_back 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _front_left
#+(or :alpha :irix6 :x86_64)
      (setf _front_left (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _front_left (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _front_right
#+(or :alpha :irix6 :x86_64)
      (setf _front_right (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _front_right (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _front_left_mid
#+(or :alpha :irix6 :x86_64)
      (setf _front_left_mid (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _front_left_mid (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _front_right_mid
#+(or :alpha :irix6 :x86_64)
      (setf _front_right_mid (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _front_right_mid (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _depo_front
#+(or :alpha :irix6 :x86_64)
      (setf _depo_front (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _depo_front (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _depo_back
#+(or :alpha :irix6 :x86_64)
      (setf _depo_back (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _depo_back (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(setf (get rdt_localization::Obstacle :md5sum-) "8b91b992300f93c3e218076979730b49")
(setf (get rdt_localization::Obstacle :datatype-) "rdt_localization/Obstacle")
(setf (get rdt_localization::Obstacle :definition-)
      "int64 front_left
int64 front_right
int64 front_left_mid
int64 front_right_mid
int64 depo_front
int64 depo_back
")



(provide :rdt_localization/Obstacle "8b91b992300f93c3e218076979730b49")


