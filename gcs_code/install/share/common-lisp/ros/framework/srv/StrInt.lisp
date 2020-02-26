; Auto-generated. Do not edit!


(cl:in-package framework-srv)


;//! \htmlinclude StrInt-request.msg.html

(cl:defclass <StrInt-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass StrInt-request (<StrInt-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StrInt-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StrInt-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name framework-srv:<StrInt-request> is deprecated: use framework-srv:StrInt-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <StrInt-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader framework-srv:data-val is deprecated.  Use framework-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StrInt-request>) ostream)
  "Serializes a message object of type '<StrInt-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StrInt-request>) istream)
  "Deserializes a message object of type '<StrInt-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StrInt-request>)))
  "Returns string type for a service object of type '<StrInt-request>"
  "framework/StrIntRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StrInt-request)))
  "Returns string type for a service object of type 'StrInt-request"
  "framework/StrIntRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StrInt-request>)))
  "Returns md5sum for a message object of type '<StrInt-request>"
  "3df8c623795dfc3d210f796f50fd2d3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StrInt-request)))
  "Returns md5sum for a message object of type 'StrInt-request"
  "3df8c623795dfc3d210f796f50fd2d3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StrInt-request>)))
  "Returns full string definition for message of type '<StrInt-request>"
  (cl:format cl:nil "string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StrInt-request)))
  "Returns full string definition for message of type 'StrInt-request"
  (cl:format cl:nil "string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StrInt-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StrInt-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StrInt-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude StrInt-response.msg.html

(cl:defclass <StrInt-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0))
)

(cl:defclass StrInt-response (<StrInt-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StrInt-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StrInt-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name framework-srv:<StrInt-response> is deprecated: use framework-srv:StrInt-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <StrInt-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader framework-srv:data-val is deprecated.  Use framework-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StrInt-response>) ostream)
  "Serializes a message object of type '<StrInt-response>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StrInt-response>) istream)
  "Deserializes a message object of type '<StrInt-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StrInt-response>)))
  "Returns string type for a service object of type '<StrInt-response>"
  "framework/StrIntResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StrInt-response)))
  "Returns string type for a service object of type 'StrInt-response"
  "framework/StrIntResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StrInt-response>)))
  "Returns md5sum for a message object of type '<StrInt-response>"
  "3df8c623795dfc3d210f796f50fd2d3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StrInt-response)))
  "Returns md5sum for a message object of type 'StrInt-response"
  "3df8c623795dfc3d210f796f50fd2d3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StrInt-response>)))
  "Returns full string definition for message of type '<StrInt-response>"
  (cl:format cl:nil "int32 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StrInt-response)))
  "Returns full string definition for message of type 'StrInt-response"
  (cl:format cl:nil "int32 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StrInt-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StrInt-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StrInt-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StrInt)))
  'StrInt-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StrInt)))
  'StrInt-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StrInt)))
  "Returns string type for a service object of type '<StrInt>"
  "framework/StrInt")