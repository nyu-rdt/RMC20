; Auto-generated. Do not edit!


(cl:in-package rdt_localization-msg)


;//! \htmlinclude Limb_Vector.msg.html

(cl:defclass <Limb_Vector> (roslisp-msg-protocol:ros-message)
  ((door
    :reader door
    :initarg :door
    :type cl:boolean
    :initform cl:nil)
   (linActs_speed
    :reader linActs_speed
    :initarg :linActs_speed
    :type cl:integer
    :initform 0)
   (arm_speed
    :reader arm_speed
    :initarg :arm_speed
    :type cl:integer
    :initform 0)
   (drum_speed
    :reader drum_speed
    :initarg :drum_speed
    :type cl:integer
    :initform 0))
)

(cl:defclass Limb_Vector (<Limb_Vector>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Limb_Vector>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Limb_Vector)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rdt_localization-msg:<Limb_Vector> is deprecated: use rdt_localization-msg:Limb_Vector instead.")))

(cl:ensure-generic-function 'door-val :lambda-list '(m))
(cl:defmethod door-val ((m <Limb_Vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rdt_localization-msg:door-val is deprecated.  Use rdt_localization-msg:door instead.")
  (door m))

(cl:ensure-generic-function 'linActs_speed-val :lambda-list '(m))
(cl:defmethod linActs_speed-val ((m <Limb_Vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rdt_localization-msg:linActs_speed-val is deprecated.  Use rdt_localization-msg:linActs_speed instead.")
  (linActs_speed m))

(cl:ensure-generic-function 'arm_speed-val :lambda-list '(m))
(cl:defmethod arm_speed-val ((m <Limb_Vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rdt_localization-msg:arm_speed-val is deprecated.  Use rdt_localization-msg:arm_speed instead.")
  (arm_speed m))

(cl:ensure-generic-function 'drum_speed-val :lambda-list '(m))
(cl:defmethod drum_speed-val ((m <Limb_Vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rdt_localization-msg:drum_speed-val is deprecated.  Use rdt_localization-msg:drum_speed instead.")
  (drum_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Limb_Vector>) ostream)
  "Serializes a message object of type '<Limb_Vector>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'door) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'linActs_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'arm_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'drum_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Limb_Vector>) istream)
  "Deserializes a message object of type '<Limb_Vector>"
    (cl:setf (cl:slot-value msg 'door) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'linActs_speed) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arm_speed) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drum_speed) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Limb_Vector>)))
  "Returns string type for a message object of type '<Limb_Vector>"
  "rdt_localization/Limb_Vector")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Limb_Vector)))
  "Returns string type for a message object of type 'Limb_Vector"
  "rdt_localization/Limb_Vector")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Limb_Vector>)))
  "Returns md5sum for a message object of type '<Limb_Vector>"
  "95198abd0938eea51c580fdd9748821b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Limb_Vector)))
  "Returns md5sum for a message object of type 'Limb_Vector"
  "95198abd0938eea51c580fdd9748821b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Limb_Vector>)))
  "Returns full string definition for message of type '<Limb_Vector>"
  (cl:format cl:nil "bool door~%int64 linActs_speed~%int64 arm_speed~%int64 drum_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Limb_Vector)))
  "Returns full string definition for message of type 'Limb_Vector"
  (cl:format cl:nil "bool door~%int64 linActs_speed~%int64 arm_speed~%int64 drum_speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Limb_Vector>))
  (cl:+ 0
     1
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Limb_Vector>))
  "Converts a ROS message object to a list"
  (cl:list 'Limb_Vector
    (cl:cons ':door (door msg))
    (cl:cons ':linActs_speed (linActs_speed msg))
    (cl:cons ':arm_speed (arm_speed msg))
    (cl:cons ':drum_speed (drum_speed msg))
))
