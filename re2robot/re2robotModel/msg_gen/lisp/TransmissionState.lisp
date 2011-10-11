; Auto-generated. Do not edit!


(cl:in-package re2robotModel-msg)


;//! \htmlinclude TransmissionState.msg.html

(cl:defclass <TransmissionState> (roslisp-msg-protocol:ros-message)
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

(cl:defclass TransmissionState (<TransmissionState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransmissionState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransmissionState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-msg:<TransmissionState> is deprecated: use re2robotModel-msg:TransmissionState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TransmissionState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:header-val is deprecated.  Use re2robotModel-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'effort-val :lambda-list '(m))
(cl:defmethod effort-val ((m <TransmissionState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:effort-val is deprecated.  Use re2robotModel-msg:effort instead.")
  (effort m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <TransmissionState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:velocity-val is deprecated.  Use re2robotModel-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <TransmissionState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:position-val is deprecated.  Use re2robotModel-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransmissionState>) ostream)
  "Serializes a message object of type '<TransmissionState>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransmissionState>) istream)
  "Deserializes a message object of type '<TransmissionState>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransmissionState>)))
  "Returns string type for a message object of type '<TransmissionState>"
  "re2robotModel/TransmissionState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransmissionState)))
  "Returns string type for a message object of type 'TransmissionState"
  "re2robotModel/TransmissionState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransmissionState>)))
  "Returns md5sum for a message object of type '<TransmissionState>"
  "909dc10c76ab46e18fbc3e3be61d280d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransmissionState)))
  "Returns md5sum for a message object of type 'TransmissionState"
  "909dc10c76ab46e18fbc3e3be61d280d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransmissionState>)))
  "Returns full string definition for message of type '<TransmissionState>"
  (cl:format cl:nil "Header header~%float32 effort~%float32 velocity~%float32 position~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransmissionState)))
  "Returns full string definition for message of type 'TransmissionState"
  (cl:format cl:nil "Header header~%float32 effort~%float32 velocity~%float32 position~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransmissionState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransmissionState>))
  "Converts a ROS message object to a list"
  (cl:list 'TransmissionState
    (cl:cons ':header (header msg))
    (cl:cons ':effort (effort msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':position (position msg))
))
