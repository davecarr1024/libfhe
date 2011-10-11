; Auto-generated. Do not edit!


(cl:in-package re2robotModel-msg)


;//! \htmlinclude ControlCmd.msg.html

(cl:defclass <ControlCmd> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (controller
    :reader controller
    :initarg :controller
    :type cl:string
    :initform ""))
)

(cl:defclass ControlCmd (<ControlCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-msg:<ControlCmd> is deprecated: use re2robotModel-msg:ControlCmd instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <ControlCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:mode-val is deprecated.  Use re2robotModel-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'controller-val :lambda-list '(m))
(cl:defmethod controller-val ((m <ControlCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:controller-val is deprecated.  Use re2robotModel-msg:controller instead.")
  (controller m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControlCmd>)))
    "Constants for message type '<ControlCmd>"
  '((:MODE_ADD_CONTROLLER . 1)
    (:MODE_REMOVE_CONTROLLER . 2)
    (:MODE_SET_ACTIVE_CONTROLLER . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControlCmd)))
    "Constants for message type 'ControlCmd"
  '((:MODE_ADD_CONTROLLER . 1)
    (:MODE_REMOVE_CONTROLLER . 2)
    (:MODE_SET_ACTIVE_CONTROLLER . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlCmd>) ostream)
  "Serializes a message object of type '<ControlCmd>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'controller))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'controller))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlCmd>) istream)
  "Deserializes a message object of type '<ControlCmd>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlCmd>)))
  "Returns string type for a message object of type '<ControlCmd>"
  "re2robotModel/ControlCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlCmd)))
  "Returns string type for a message object of type 'ControlCmd"
  "re2robotModel/ControlCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlCmd>)))
  "Returns md5sum for a message object of type '<ControlCmd>"
  "fe80797f03ec01b6161834d44aa4929d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlCmd)))
  "Returns md5sum for a message object of type 'ControlCmd"
  "fe80797f03ec01b6161834d44aa4929d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlCmd>)))
  "Returns full string definition for message of type '<ControlCmd>"
  (cl:format cl:nil "uint8 MODE_ADD_CONTROLLER = 1~%uint8 MODE_REMOVE_CONTROLLER = 2~%uint8 MODE_SET_ACTIVE_CONTROLLER = 3~%~%uint8 mode~%string controller~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlCmd)))
  "Returns full string definition for message of type 'ControlCmd"
  (cl:format cl:nil "uint8 MODE_ADD_CONTROLLER = 1~%uint8 MODE_REMOVE_CONTROLLER = 2~%uint8 MODE_SET_ACTIVE_CONTROLLER = 3~%~%uint8 mode~%string controller~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlCmd>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'controller))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlCmd
    (cl:cons ':mode (mode msg))
    (cl:cons ':controller (controller msg))
))
