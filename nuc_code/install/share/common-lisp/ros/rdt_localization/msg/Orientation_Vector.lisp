; Auto-generated. Do not edit!


(cl:in-package rdt_localization-msg)


;//! \htmlinclude Orientation_Vector.msg.html

(cl:defclass <Orientation_Vector> (roslisp-msg-protocol:ros-message)
  ((robot_pose
    :reader robot_pose
    :initarg :robot_pose
    :type rdt_localization-msg:Pose
    :initform (cl:make-instance 'rdt_localization-msg:Pose))
   (target_zone
    :reader target_zone
    :initarg :target_zone
    :type rdt_localization-msg:Location
    :initform (cl:make-instance 'rdt_localization-msg:Location))
   (robot_speed
    :reader robot_speed
    :initarg :robot_speed
    :type cl:integer
    :initform 0))
)

(cl:defclass Orientation_Vector (<Orientation_Vector>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Orientation_Vector>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Orientation_Vector)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rdt_localization-msg:<Orientation_Vector> is deprecated: use rdt_localization-msg:Orientation_Vector instead.")))

(cl:ensure-generic-function 'robot_pose-val :lambda-list '(m))
(cl:defmethod robot_pose-val ((m <Orientation_Vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rdt_localization-msg:robot_pose-val is deprecated.  Use rdt_localization-msg:robot_pose instead.")
  (robot_pose m))

(cl:ensure-generic-function 'target_zone-val :lambda-list '(m))
(cl:defmethod target_zone-val ((m <Orientation_Vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rdt_localization-msg:target_zone-val is deprecated.  Use rdt_localization-msg:target_zone instead.")
  (target_zone m))

(cl:ensure-generic-function 'robot_speed-val :lambda-list '(m))
(cl:defmethod robot_speed-val ((m <Orientation_Vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rdt_localization-msg:robot_speed-val is deprecated.  Use rdt_localization-msg:robot_speed instead.")
  (robot_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Orientation_Vector>) ostream)
  "Serializes a message object of type '<Orientation_Vector>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_zone) ostream)
  (cl:let* ((signed (cl:slot-value msg 'robot_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Orientation_Vector>) istream)
  "Deserializes a message object of type '<Orientation_Vector>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_zone) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_speed) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Orientation_Vector>)))
  "Returns string type for a message object of type '<Orientation_Vector>"
  "rdt_localization/Orientation_Vector")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Orientation_Vector)))
  "Returns string type for a message object of type 'Orientation_Vector"
  "rdt_localization/Orientation_Vector")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Orientation_Vector>)))
  "Returns md5sum for a message object of type '<Orientation_Vector>"
  "32c0233ad09fff327409092d7163201c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Orientation_Vector)))
  "Returns md5sum for a message object of type 'Orientation_Vector"
  "32c0233ad09fff327409092d7163201c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Orientation_Vector>)))
  "Returns full string definition for message of type '<Orientation_Vector>"
  (cl:format cl:nil "Pose robot_pose~%Location target_zone~%int64 robot_speed~%================================================================================~%MSG: rdt_localization/Pose~%float32 x~%float32 y~%float32 orientation~%================================================================================~%MSG: rdt_localization/Location~%float32 x~%float32 y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Orientation_Vector)))
  "Returns full string definition for message of type 'Orientation_Vector"
  (cl:format cl:nil "Pose robot_pose~%Location target_zone~%int64 robot_speed~%================================================================================~%MSG: rdt_localization/Pose~%float32 x~%float32 y~%float32 orientation~%================================================================================~%MSG: rdt_localization/Location~%float32 x~%float32 y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Orientation_Vector>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_zone))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Orientation_Vector>))
  "Converts a ROS message object to a list"
  (cl:list 'Orientation_Vector
    (cl:cons ':robot_pose (robot_pose msg))
    (cl:cons ':target_zone (target_zone msg))
    (cl:cons ':robot_speed (robot_speed msg))
))
