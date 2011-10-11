; Auto-generated. Do not edit!


(cl:in-package re2robotModel-msg)


;//! \htmlinclude JointCmd.msg.html

(cl:defclass <JointCmd> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (cmd
    :reader cmd
    :initarg :cmd
    :type cl:float
    :initform 0.0)
   (controller
    :reader controller
    :initarg :controller
    :type cl:string
    :initform ""))
)

(cl:defclass JointCmd (<JointCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-msg:<JointCmd> is deprecated: use re2robotModel-msg:JointCmd instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <JointCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:mode-val is deprecated.  Use re2robotModel-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <JointCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:cmd-val is deprecated.  Use re2robotModel-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'controller-val :lambda-list '(m))
(cl:defmethod controller-val ((m <JointCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:controller-val is deprecated.  Use re2robotModel-msg:controller instead.")
  (controller m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<JointCmd>)))
    "Constants for message type '<JointCmd>"
  '((:MODE_EFFORT . 1)
    (:MODE_VELOCITY . 2)
    (:MODE_POSITION . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'JointCmd)))
    "Constants for message type 'JointCmd"
  '((:MODE_EFFORT . 1)
    (:MODE_VELOCITY . 2)
    (:MODE_POSITION . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointCmd>) ostream)
  "Serializes a message object of type '<JointCmd>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'controller))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'controller))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointCmd>) istream)
  "Deserializes a message object of type '<JointCmd>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cmd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'controller) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'controller) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointCmd>)))
  "Returns string type for a message object of type '<JointCmd>"
  "re2robotModel/JointCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointCmd)))
  "Returns string type for a message object of type 'JointCmd"
  "re2robotModel/JointCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointCmd>)))
  "Returns md5sum for a message object of type '<JointCmd>"
  "0da0207f4feb073f159111e615f666b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointCmd)))
  "Returns md5sum for a message object of type 'JointCmd"
  "0da0207f4feb073f159111e615f666b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointCmd>)))
  "Returns full string definition for message of type '<JointCmd>"
  (cl:format cl:nil "uint8 MODE_EFFORT = 1~%uint8 MODE_VELOCITY = 2~%uint8 MODE_POSITION = 3~%~%uint8 mode~%float32 cmd~%string controller~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointCmd)))
  "Returns full string definition for message of type 'JointCmd"
  (cl:format cl:nil "uint8 MODE_EFFORT = 1~%uint8 MODE_VELOCITY = 2~%uint8 MODE_POSITION = 3~%~%uint8 mode~%float32 cmd~%string controller~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointCmd>))
  (cl:+ 0
     1
     4
     4 (cl:length (cl:slot-value msg 'controller))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'JointCmd
    (cl:cons ':mode (mode msg))
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':controller (controller msg))
))
