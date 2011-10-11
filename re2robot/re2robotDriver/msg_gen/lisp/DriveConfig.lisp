; Auto-generated. Do not edit!


(cl:in-package re2robotDriver-msg)


;//! \htmlinclude DriveConfig.msg.html

(cl:defclass <DriveConfig> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (updateIntervalS
    :reader updateIntervalS
    :initarg :updateIntervalS
    :type cl:float
    :initform 0.0)
   (cmdTimeoutS
    :reader cmdTimeoutS
    :initarg :cmdTimeoutS
    :type cl:float
    :initform 0.0)
   (maxVelocity
    :reader maxVelocity
    :initarg :maxVelocity
    :type cl:float
    :initform 0.0)
   (minPosition
    :reader minPosition
    :initarg :minPosition
    :type cl:float
    :initform 0.0)
   (maxPosition
    :reader maxPosition
    :initarg :maxPosition
    :type cl:float
    :initform 0.0)
   (initialPosition
    :reader initialPosition
    :initarg :initialPosition
    :type cl:float
    :initform 0.0)
   (positionPGain
    :reader positionPGain
    :initarg :positionPGain
    :type cl:float
    :initform 0.0)
   (positionDGain
    :reader positionDGain
    :initarg :positionDGain
    :type cl:float
    :initform 0.0)
   (velocityTimeToTargetS
    :reader velocityTimeToTargetS
    :initarg :velocityTimeToTargetS
    :type cl:float
    :initform 0.0))
)

(cl:defclass DriveConfig (<DriveConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DriveConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DriveConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotDriver-msg:<DriveConfig> is deprecated: use re2robotDriver-msg:DriveConfig instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <DriveConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:name-val is deprecated.  Use re2robotDriver-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <DriveConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:type-val is deprecated.  Use re2robotDriver-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'updateIntervalS-val :lambda-list '(m))
(cl:defmethod updateIntervalS-val ((m <DriveConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:updateIntervalS-val is deprecated.  Use re2robotDriver-msg:updateIntervalS instead.")
  (updateIntervalS m))

(cl:ensure-generic-function 'cmdTimeoutS-val :lambda-list '(m))
(cl:defmethod cmdTimeoutS-val ((m <DriveConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:cmdTimeoutS-val is deprecated.  Use re2robotDriver-msg:cmdTimeoutS instead.")
  (cmdTimeoutS m))

(cl:ensure-generic-function 'maxVelocity-val :lambda-list '(m))
(cl:defmethod maxVelocity-val ((m <DriveConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:maxVelocity-val is deprecated.  Use re2robotDriver-msg:maxVelocity instead.")
  (maxVelocity m))

(cl:ensure-generic-function 'minPosition-val :lambda-list '(m))
(cl:defmethod minPosition-val ((m <DriveConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:minPosition-val is deprecated.  Use re2robotDriver-msg:minPosition instead.")
  (minPosition m))

(cl:ensure-generic-function 'maxPosition-val :lambda-list '(m))
(cl:defmethod maxPosition-val ((m <DriveConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:maxPosition-val is deprecated.  Use re2robotDriver-msg:maxPosition instead.")
  (maxPosition m))

(cl:ensure-generic-function 'initialPosition-val :lambda-list '(m))
(cl:defmethod initialPosition-val ((m <DriveConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:initialPosition-val is deprecated.  Use re2robotDriver-msg:initialPosition instead.")
  (initialPosition m))

(cl:ensure-generic-function 'positionPGain-val :lambda-list '(m))
(cl:defmethod positionPGain-val ((m <DriveConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:positionPGain-val is deprecated.  Use re2robotDriver-msg:positionPGain instead.")
  (positionPGain m))

(cl:ensure-generic-function 'positionDGain-val :lambda-list '(m))
(cl:defmethod positionDGain-val ((m <DriveConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:positionDGain-val is deprecated.  Use re2robotDriver-msg:positionDGain instead.")
  (positionDGain m))

(cl:ensure-generic-function 'velocityTimeToTargetS-val :lambda-list '(m))
(cl:defmethod velocityTimeToTargetS-val ((m <DriveConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:velocityTimeToTargetS-val is deprecated.  Use re2robotDriver-msg:velocityTimeToTargetS instead.")
  (velocityTimeToTargetS m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DriveConfig>) ostream)
  "Serializes a message object of type '<DriveConfig>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cmdTimeoutS))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'maxVelocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'minPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'maxPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initialPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'positionPGain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'positionDGain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocityTimeToTargetS))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DriveConfig>) istream)
  "Deserializes a message object of type '<DriveConfig>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cmdTimeoutS) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxVelocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'minPosition) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxPosition) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initialPosition) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'positionPGain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'positionDGain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocityTimeToTargetS) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DriveConfig>)))
  "Returns string type for a message object of type '<DriveConfig>"
  "re2robotDriver/DriveConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DriveConfig)))
  "Returns string type for a message object of type 'DriveConfig"
  "re2robotDriver/DriveConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DriveConfig>)))
  "Returns md5sum for a message object of type '<DriveConfig>"
  "7739876033e4c5d57efaa789aba3f14f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DriveConfig)))
  "Returns md5sum for a message object of type 'DriveConfig"
  "7739876033e4c5d57efaa789aba3f14f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DriveConfig>)))
  "Returns full string definition for message of type '<DriveConfig>"
  (cl:format cl:nil "#Drive~%string name~%string type~%float32 updateIntervalS~%float32 cmdTimeoutS~%float32 maxVelocity~%float32 minPosition~%float32 maxPosition~%~%#SimDrive~%float32 initialPosition~%float32 positionPGain~%float32 positionDGain~%float32 velocityTimeToTargetS~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DriveConfig)))
  "Returns full string definition for message of type 'DriveConfig"
  (cl:format cl:nil "#Drive~%string name~%string type~%float32 updateIntervalS~%float32 cmdTimeoutS~%float32 maxVelocity~%float32 minPosition~%float32 maxPosition~%~%#SimDrive~%float32 initialPosition~%float32 positionPGain~%float32 positionDGain~%float32 velocityTimeToTargetS~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DriveConfig>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'type))
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DriveConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'DriveConfig
    (cl:cons ':name (name msg))
    (cl:cons ':type (type msg))
    (cl:cons ':updateIntervalS (updateIntervalS msg))
    (cl:cons ':cmdTimeoutS (cmdTimeoutS msg))
    (cl:cons ':maxVelocity (maxVelocity msg))
    (cl:cons ':minPosition (minPosition msg))
    (cl:cons ':maxPosition (maxPosition msg))
    (cl:cons ':initialPosition (initialPosition msg))
    (cl:cons ':positionPGain (positionPGain msg))
    (cl:cons ':positionDGain (positionDGain msg))
    (cl:cons ':velocityTimeToTargetS (velocityTimeToTargetS msg))
))
