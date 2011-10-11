; Auto-generated. Do not edit!


(cl:in-package re2robotModel-msg)


;//! \htmlinclude ControlArbiterState.msg.html

(cl:defclass <ControlArbiterState> (roslisp-msg-protocol:ros-message)
  ((activeController
    :reader activeController
    :initarg :activeController
    :type cl:string
    :initform "")
   (controllers
    :reader controllers
    :initarg :controllers
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass ControlArbiterState (<ControlArbiterState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlArbiterState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlArbiterState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-msg:<ControlArbiterState> is deprecated: use re2robotModel-msg:ControlArbiterState instead.")))

(cl:ensure-generic-function 'activeController-val :lambda-list '(m))
(cl:defmethod activeController-val ((m <ControlArbiterState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:activeController-val is deprecated.  Use re2robotModel-msg:activeController instead.")
  (activeController m))

(cl:ensure-generic-function 'controllers-val :lambda-list '(m))
(cl:defmethod controllers-val ((m <ControlArbiterState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:controllers-val is deprecated.  Use re2robotModel-msg:controllers instead.")
  (controllers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlArbiterState>) ostream)
  "Serializes a message object of type '<ControlArbiterState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'activeController))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'activeController))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'controllers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'controllers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlArbiterState>) istream)
  "Deserializes a message object of type '<ControlArbiterState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'activeController) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'activeController) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'controllers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'controllers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlArbiterState>)))
  "Returns string type for a message object of type '<ControlArbiterState>"
  "re2robotModel/ControlArbiterState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlArbiterState)))
  "Returns string type for a message object of type 'ControlArbiterState"
  "re2robotModel/ControlArbiterState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlArbiterState>)))
  "Returns md5sum for a message object of type '<ControlArbiterState>"
  "c5ef3b0c73d38edb82013574bb8e3755")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlArbiterState)))
  "Returns md5sum for a message object of type 'ControlArbiterState"
  "c5ef3b0c73d38edb82013574bb8e3755")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlArbiterState>)))
  "Returns full string definition for message of type '<ControlArbiterState>"
  (cl:format cl:nil "string activeController~%string[] controllers~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlArbiterState)))
  "Returns full string definition for message of type 'ControlArbiterState"
  (cl:format cl:nil "string activeController~%string[] controllers~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlArbiterState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'activeController))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'controllers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlArbiterState>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlArbiterState
    (cl:cons ':activeController (activeController msg))
    (cl:cons ':controllers (controllers msg))
))
