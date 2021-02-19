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
  :slots (_left _right ))

(defmethod rdt_localization::Obstacle
  (:init
   (&key
    ((:left __left) nil)
    ((:right __right) nil)
    )
   (send-super :init)
   (setq _left __left)
   (setq _right __right)
   self)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:serialization-length
   ()
   (+
    ;; bool _left
    1
    ;; bool _right
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _left
       (if _left (write-byte -1 s) (write-byte 0 s))
     ;; bool _right
       (if _right (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _left
     (setq _left (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right
     (setq _right (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rdt_localization::Obstacle :md5sum-) "0544cac0b98e92509d14f758d50cf24b")
(setf (get rdt_localization::Obstacle :datatype-) "rdt_localization/Obstacle")
(setf (get rdt_localization::Obstacle :definition-)
      "bool left
bool right
")



(provide :rdt_localization/Obstacle "0544cac0b98e92509d14f758d50cf24b")


