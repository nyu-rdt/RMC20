; Auto-generated. Do not edit!


(cl:in-package rdt_localization-msg)


;//! \htmlinclude Drive_Vector.msg.html

(cl:defclass <Drive_Vector> (roslisp-msg-protocol:ros-message)
  ((robot_spd
    :reader robot_spd
    :initarg :robot_spd
    :type cl:integer
    :initform 0)
   (offset_driveMode
    :reader offset_driveMode
    :initarg :offset_driveMode
    :type cl:integer
    :initform 0))
)

(cl:defclass Drive_Vector (<Drive_Vector>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Drive_Vector>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Drive_Vector)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rdt_localization-msg:<Drive_Vector> is deprecated: use rdt_localization-msg:Drive_Vector instead.")))

(cl:ensure-generic-function 'robot_spd-val :lambda-list '(m))
(cl:defmethod robot_spd-val ((m <Drive_Vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rdt_localization-msg:robot_spd-val is deprecated.  Use rdt_localization-msg:robot_spd instead.")
  (robot_spd m))

(cl:ensure-generic-function 'offset_driveMode-val :lambda-list '(m))
(cl:defmethod offset_driveMode-val ((m <Drive_Vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rdt_localization-msg:offset_driveMode-val is deprecated.  Use rdt_localization-msg:offset_driveMode instead.")
  (offset_driveMode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Drive_Vector>) ostream)
  "Serializes a message object of type '<Drive_Vector>"
  (cl:let* ((signed (cl:slot-value msg 'robot_spd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'offset_driveMode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Drive_Vector>) istream)
  "Deserializes a message object of type '<Drive_Vector>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_spd) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'offset_driveMode) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Drive_Vector>)))
  "Returns string type for a message object of type '<Drive_Vector>"
  "rdt_localization/Drive_Vector")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Drive_Vector)))
  "Returns string type for a message object of type 'Drive_Vector"
  "rdt_localization/Drive_Vector")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Drive_Vector>)))
  "Returns md5sum for a message object of type '<Drive_Vector>"
  "9bb2e6492d363aa84cce71e895f95bc0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Drive_Vector)))
  "Returns md5sum for a message object of type 'Drive_Vector"
  "9bb2e6492d363aa84cce71e895f95bc0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Drive_Vector>)))
  "Returns full string definition for message of type '<Drive_Vector>"
  (cl:format cl:nil "int64 robot_spd~%int64 offset_driveMode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Drive_Vector)))
  "Returns full string definition for message of type 'Drive_Vector"
  (cl:format cl:nil "int64 robot_spd~%int64 offset_driveMode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Drive_Vector>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Drive_Vector>))
  "Converts a ROS message object to a list"
  (cl:list 'Drive_Vector
    (cl:cons ':robot_spd (robot_spd msg))
    (cl:cons ':offset_driveMode (offset_driveMode msg))
))
