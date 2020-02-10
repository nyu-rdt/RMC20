; Auto-generated. Do not edit!


(cl:in-package framework-srv)


;//! \htmlinclude IntStr-request.msg.html

(cl:defclass <IntStr-request> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass IntStr-request (<IntStr-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IntStr-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IntStr-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name framework-srv:<IntStr-request> is deprecated: use framework-srv:IntStr-request instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <IntStr-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader framework-srv:ID-val is deprecated.  Use framework-srv:ID instead.")
  (ID m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <IntStr-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader framework-srv:data-val is deprecated.  Use framework-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IntStr-request>) ostream)
  "Serializes a message object of type '<IntStr-request>"
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IntStr-request>) istream)
  "Deserializes a message object of type '<IntStr-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IntStr-request>)))
  "Returns string type for a service object of type '<IntStr-request>"
  "framework/IntStrRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntStr-request)))
  "Returns string type for a service object of type 'IntStr-request"
  "framework/IntStrRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IntStr-request>)))
  "Returns md5sum for a message object of type '<IntStr-request>"
  "e4e1a089454a32166064b947dbe98aff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IntStr-request)))
  "Returns md5sum for a message object of type 'IntStr-request"
  "e4e1a089454a32166064b947dbe98aff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IntStr-request>)))
  "Returns full string definition for message of type '<IntStr-request>"
  (cl:format cl:nil "int32 ID~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IntStr-request)))
  "Returns full string definition for message of type 'IntStr-request"
  (cl:format cl:nil "int32 ID~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IntStr-request>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IntStr-request>))
  "Converts a ROS message object to a list"
  (cl:list 'IntStr-request
    (cl:cons ':ID (ID msg))
    (cl:cons ':data (data msg))
))
;//! \htmlinclude IntStr-response.msg.html

(cl:defclass <IntStr-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass IntStr-response (<IntStr-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IntStr-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IntStr-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name framework-srv:<IntStr-response> is deprecated: use framework-srv:IntStr-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <IntStr-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader framework-srv:success-val is deprecated.  Use framework-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IntStr-response>) ostream)
  "Serializes a message object of type '<IntStr-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IntStr-response>) istream)
  "Deserializes a message object of type '<IntStr-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IntStr-response>)))
  "Returns string type for a service object of type '<IntStr-response>"
  "framework/IntStrResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntStr-response)))
  "Returns string type for a service object of type 'IntStr-response"
  "framework/IntStrResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IntStr-response>)))
  "Returns md5sum for a message object of type '<IntStr-response>"
  "e4e1a089454a32166064b947dbe98aff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IntStr-response)))
  "Returns md5sum for a message object of type 'IntStr-response"
  "e4e1a089454a32166064b947dbe98aff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IntStr-response>)))
  "Returns full string definition for message of type '<IntStr-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IntStr-response)))
  "Returns full string definition for message of type 'IntStr-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IntStr-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IntStr-response>))
  "Converts a ROS message object to a list"
  (cl:list 'IntStr-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'IntStr)))
  'IntStr-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'IntStr)))
  'IntStr-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntStr)))
  "Returns string type for a service object of type '<IntStr>"
  "framework/IntStr")