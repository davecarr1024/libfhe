; Auto-generated. Do not edit!


(cl:in-package re2robotModel-msg)


;//! \htmlinclude ArbiterConfig.msg.html

(cl:defclass <ArbiterConfig> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (updateIntervalS
    :reader updateIntervalS
    :initarg :updateIntervalS
    :type cl:float
    :initform 0.0)
   (initialController
    :reader initialController
    :initarg :initialController
    :type cl:string
    :initform ""))
)

(cl:defclass ArbiterConfig (<ArbiterConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArbiterConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArbiterConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-msg:<ArbiterConfig> is deprecated: use re2robotModel-msg:ArbiterConfig instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <ArbiterConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:type-val is deprecated.  Use re2robotModel-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'updateIntervalS-val :lambda-list '(m))
(cl:defmethod updateIntervalS-val ((m <ArbiterConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:updateIntervalS-val is deprecated.  Use re2robotModel-msg:updateIntervalS instead.")
  (updateIntervalS m))

(cl:ensure-generic-function 'initialController-val :lambda-list '(m))
(cl:defmethod initialController-val ((m <ArbiterConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:initialController-val is deprecated.  Use re2robotModel-msg:initialController instead.")
  (initialController m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArbiterConfig>) ostream)
  "Serializes a message object of type '<ArbiterConfig>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'updateIntervalS))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'initialController))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'initialController))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArbiterConfig>) istream)
  "Deserializes a message object of type '<ArbiterConfig>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'updateIntervalS) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'initialController) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'initialController) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArbiterConfig>)))
  "Returns string type for a message object of type '<ArbiterConfig>"
  "re2robotModel/ArbiterConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArbiterConfig)))
  "Returns string type for a message object of type 'ArbiterConfig"
  "re2robotModel/ArbiterConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArbiterConfig>)))
  "Returns md5sum for a message object of type '<ArbiterConfig>"
  "16d15c6c4f109996d0d7cf619c2c571b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArbiterConfig)))
  "Returns md5sum for a message object of type 'ArbiterConfig"
  "16d15c6c4f109996d0d7cf619c2c571b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArbiterConfig>)))
  "Returns full string definition for message of type '<ArbiterConfig>"
  (cl:format cl:nil "#common~%string type~%float32 updateIntervalS~%~%#control arbiter~%string initialController~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArbiterConfig)))
  "Returns full string definition for message of type 'ArbiterConfig"
  (cl:format cl:nil "#common~%string type~%float32 updateIntervalS~%~%#control arbiter~%string initialController~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArbiterConfig>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
     4
     4 (cl:length (cl:slot-value msg 'initialController))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArbiterConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'ArbiterConfig
    (cl:cons ':type (type msg))
    (cl:cons ':updateIntervalS (updateIntervalS msg))
    (cl:cons ':initialController (initialController msg))
))
