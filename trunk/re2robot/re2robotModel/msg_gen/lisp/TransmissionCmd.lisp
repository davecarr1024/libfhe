; Auto-generated. Do not edit!


(cl:in-package re2robotModel-msg)


;//! \htmlinclude TransmissionCmd.msg.html

(cl:defclass <TransmissionCmd> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (cmd
    :reader cmd
    :initarg :cmd
    :type cl:float
    :initform 0.0))
)

(cl:defclass TransmissionCmd (<TransmissionCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransmissionCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransmissionCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-msg:<TransmissionCmd> is deprecated: use re2robotModel-msg:TransmissionCmd instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <TransmissionCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:mode-val is deprecated.  Use re2robotModel-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <TransmissionCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:cmd-val is deprecated.  Use re2robotModel-msg:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TransmissionCmd>)))
    "Constants for message type '<TransmissionCmd>"
  '((:MODE_EFFORT . 1)
    (:MODE_VELOCITY . 2)
    (:MODE_POSITION . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TransmissionCmd)))
    "Constants for message type 'TransmissionCmd"
  '((:MODE_EFFORT . 1)
    (:MODE_VELOCITY . 2)
    (:MODE_POSITION . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransmissionCmd>) ostream)
  "Serializes a message object of type '<TransmissionCmd>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransmissionCmd>) istream)
  "Deserializes a message object of type '<TransmissionCmd>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cmd) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransmissionCmd>)))
  "Returns string type for a message object of type '<TransmissionCmd>"
  "re2robotModel/TransmissionCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransmissionCmd)))
  "Returns string type for a message object of type 'TransmissionCmd"
  "re2robotModel/TransmissionCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransmissionCmd>)))
  "Returns md5sum for a message object of type '<TransmissionCmd>"
  "3af6dabffd29e01e37500e9a7f5f0bdd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransmissionCmd)))
  "Returns md5sum for a message object of type 'TransmissionCmd"
  "3af6dabffd29e01e37500e9a7f5f0bdd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransmissionCmd>)))
  "Returns full string definition for message of type '<TransmissionCmd>"
  (cl:format cl:nil "uint8 MODE_EFFORT = 1~%uint8 MODE_VELOCITY = 2~%uint8 MODE_POSITION = 3~%~%uint8 mode~%float32 cmd~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransmissionCmd)))
  "Returns full string definition for message of type 'TransmissionCmd"
  (cl:format cl:nil "uint8 MODE_EFFORT = 1~%uint8 MODE_VELOCITY = 2~%uint8 MODE_POSITION = 3~%~%uint8 mode~%float32 cmd~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransmissionCmd>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransmissionCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'TransmissionCmd
    (cl:cons ':mode (mode msg))
    (cl:cons ':cmd (cmd msg))
))
