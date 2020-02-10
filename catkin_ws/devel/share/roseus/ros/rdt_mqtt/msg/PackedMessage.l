;; Auto-generated. Do not edit!


(when (boundp 'rdt_mqtt::PackedMessage)
  (if (not (find-package "RDT_MQTT"))
    (make-package "RDT_MQTT"))
  (shadow 'PackedMessage (find-package "RDT_MQTT")))
(unless (find-package "RDT_MQTT::PACKEDMESSAGE")
  (make-package "RDT_MQTT::PACKEDMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude PackedMessage.msg.html


(defclass rdt_mqtt::PackedMessage
  :super ros::object
  :slots (_channel_name _field _value ))

(defmethod rdt_mqtt::PackedMessage
  (:init
   (&key
    ((:channel_name __channel_name) "")
    ((:field __field) "")
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _channel_name (string __channel_name))
   (setq _field (string __field))
   (setq _value (round __value))
   self)
  (:channel_name
   (&optional __channel_name)
   (if __channel_name (setq _channel_name __channel_name)) _channel_name)
  (:field
   (&optional __field)
   (if __field (setq _field __field)) _field)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _channel_name
    4 (length _channel_name)
    ;; string _field
    4 (length _field)
    ;; int32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _channel_name
       (write-long (length _channel_name) s) (princ _channel_name s)
     ;; string _field
       (write-long (length _field) s) (princ _field s)
     ;; int32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _channel_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _channel_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _field
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _field (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rdt_mqtt::PackedMessage :md5sum-) "760519a0e798e18cf38f17054d3d31c8")
(setf (get rdt_mqtt::PackedMessage :datatype-) "rdt_mqtt/PackedMessage")
(setf (get rdt_mqtt::PackedMessage :definition-)
      "string channel_name
string field
int32 value

")



(provide :rdt_mqtt/PackedMessage "760519a0e798e18cf38f17054d3d31c8")


