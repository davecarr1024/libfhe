; Auto-generated. Do not edit!


(cl:in-package re2robotModel-msg)


;//! \htmlinclude TransmissionConfig.msg.html

(cl:defclass <TransmissionConfig> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (drive
    :reader drive
    :initarg :drive
    :type cl:string
    :initform "")
   (joint
    :reader joint
    :initarg :joint
    :type cl:string
    :initform "")
   (timerIntervalS
    :reader timerIntervalS
    :initarg :timerIntervalS
    :type cl:float
    :initform 0.0))
)

(cl:defclass TransmissionConfig (<TransmissionConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransmissionConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransmissionConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-msg:<TransmissionConfig> is deprecated: use re2robotModel-msg:TransmissionConfig instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <TransmissionConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:type-val is deprecated.  Use re2robotModel-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'drive-val :lambda-list '(m))
(cl:defmethod drive-val ((m <TransmissionConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:drive-val is deprecated.  Use re2robotModel-msg:drive instead.")
  (drive m))

(cl:ensure-generic-function 'joint-val :lambda-list '(m))
(cl:defmethod joint-val ((m <TransmissionConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:joint-val is deprecated.  Use re2robotModel-msg:joint instead.")
  (joint m))

(cl:ensure-generic-function 'timerIntervalS-val :lambda-list '(m))
(cl:defmethod timerIntervalS-val ((m <TransmissionConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:timerIntervalS-val is deprecated.  Use re2robotModel-msg:timerIntervalS instead.")
  (timerIntervalS m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransmissionConfig>) ostream)
  "Serializes a message object of type '<TransmissionConfig>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'drive))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'drive))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'joint))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'timerIntervalS))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransmissionConfig>) istream)
  "Deserializes a message object of type '<TransmissionConfig>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drive) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'drive) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'joint) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'joint) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timerIntervalS) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransmissionConfig>)))
  "Returns string type for a message object of type '<TransmissionConfig>"
  "re2robotModel/TransmissionConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransmissionConfig)))
  "Returns string type for a message object of type 'TransmissionConfig"
  "re2robotModel/TransmissionConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransmissionConfig>)))
  "Returns md5sum for a message object of type '<TransmissionConfig>"
  "6841c6455f157038b84358a2ff1b61d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransmissionConfig)))
  "Returns md5sum for a message object of type 'TransmissionConfig"
  "6841c6455f157038b84358a2ff1b61d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransmissionConfig>)))
  "Returns full string definition for message of type '<TransmissionConfig>"
  (cl:format cl:nil "string type~%string drive~%string joint~%float32 timerIntervalS~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransmissionConfig)))
  "Returns full string definition for message of type 'TransmissionConfig"
  (cl:format cl:nil "string type~%string drive~%string joint~%float32 timerIntervalS~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransmissionConfig>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:length (cl:slot-value msg 'drive))
     4 (cl:length (cl:slot-value msg 'joint))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransmissionConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'TransmissionConfig
    (cl:cons ':type (type msg))
    (cl:cons ':drive (drive msg))
    (cl:cons ':joint (joint msg))
    (cl:cons ':timerIntervalS (timerIntervalS msg))
))
