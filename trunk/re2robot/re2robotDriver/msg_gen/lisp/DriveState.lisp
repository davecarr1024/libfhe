; Auto-generated. Do not edit!


(cl:in-package re2robotDriver-msg)


;//! \htmlinclude DriveState.msg.html

(cl:defclass <DriveState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (effort
    :reader effort
    :initarg :effort
    :type cl:float
    :initform 0.0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0))
)

(cl:defclass DriveState (<DriveState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DriveState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DriveState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotDriver-msg:<DriveState> is deprecated: use re2robotDriver-msg:DriveState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DriveState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:header-val is deprecated.  Use re2robotDriver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'effort-val :lambda-list '(m))
(cl:defmethod effort-val ((m <DriveState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:effort-val is deprecated.  Use re2robotDriver-msg:effort instead.")
  (effort m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <DriveState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:velocity-val is deprecated.  Use re2robotDriver-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <DriveState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:position-val is deprecated.  Use re2robotDriver-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DriveState>) ostream)
  "Serializes a message object of type '<DriveState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'effort))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DriveState>) istream)
  "Deserializes a message object of type '<DriveState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'effort) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DriveState>)))
  "Returns string type for a message object of type '<DriveState>"
  "re2robotDriver/DriveState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DriveState)))
  "Returns string type for a message object of type 'DriveState"
  "re2robotDriver/DriveState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DriveState>)))
  "Returns md5sum for a message object of type '<DriveState>"
  "909dc10c76ab46e18fbc3e3be61d280d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DriveState)))
  "Returns md5sum for a message object of type 'DriveState"
  "909dc10c76ab46e18fbc3e3be61d280d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DriveState>)))
  "Returns full string definition for message of type '<DriveState>"
  (cl:format cl:nil "Header header~%float32 effort~%float32 velocity~%float32 position~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DriveState)))
  "Returns full string definition for message of type 'DriveState"
  (cl:format cl:nil "Header header~%float32 effort~%float32 velocity~%float32 position~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DriveState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DriveState>))
  "Converts a ROS message object to a list"
  (cl:list 'DriveState
    (cl:cons ':header (header msg))
    (cl:cons ':effort (effort msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':position (position msg))
))
