; Auto-generated. Do not edit!


(cl:in-package re2robotDriver-msg)


;//! \htmlinclude DriverConfig.msg.html

(cl:defclass <DriverConfig> (roslisp-msg-protocol:ros-message)
  ((drives
    :reader drives
    :initarg :drives
    :type (cl:vector re2robotDriver-msg:DriveConfig)
   :initform (cl:make-array 0 :element-type 're2robotDriver-msg:DriveConfig :initial-element (cl:make-instance 're2robotDriver-msg:DriveConfig))))
)

(cl:defclass DriverConfig (<DriverConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DriverConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DriverConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotDriver-msg:<DriverConfig> is deprecated: use re2robotDriver-msg:DriverConfig instead.")))

(cl:ensure-generic-function 'drives-val :lambda-list '(m))
(cl:defmethod drives-val ((m <DriverConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-msg:drives-val is deprecated.  Use re2robotDriver-msg:drives instead.")
  (drives m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DriverConfig>) ostream)
  "Serializes a message object of type '<DriverConfig>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'drives))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'drives))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DriverConfig>) istream)
  "Deserializes a message object of type '<DriverConfig>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'drives) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'drives)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 're2robotDriver-msg:DriveConfig))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DriverConfig>)))
  "Returns string type for a message object of type '<DriverConfig>"
  "re2robotDriver/DriverConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DriverConfig)))
  "Returns string type for a message object of type 'DriverConfig"
  "re2robotDriver/DriverConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DriverConfig>)))
  "Returns md5sum for a message object of type '<DriverConfig>"
  "eeb8e90567be2610059b20481b8a260e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DriverConfig)))
  "Returns md5sum for a message object of type 'DriverConfig"
  "eeb8e90567be2610059b20481b8a260e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DriverConfig>)))
  "Returns full string definition for message of type '<DriverConfig>"
  (cl:format cl:nil "DriveConfig[] drives~%================================================================================~%MSG: re2robotDriver/DriveConfig~%#Drive~%string name~%string type~%float32 updateIntervalS~%float32 cmdTimeoutS~%float32 maxVelocity~%float32 minPosition~%float32 maxPosition~%~%#SimDrive~%float32 initialPosition~%float32 positionPGain~%float32 positionDGain~%float32 velocityTimeToTargetS~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DriverConfig)))
  "Returns full string definition for message of type 'DriverConfig"
  (cl:format cl:nil "DriveConfig[] drives~%================================================================================~%MSG: re2robotDriver/DriveConfig~%#Drive~%string name~%string type~%float32 updateIntervalS~%float32 cmdTimeoutS~%float32 maxVelocity~%float32 minPosition~%float32 maxPosition~%~%#SimDrive~%float32 initialPosition~%float32 positionPGain~%float32 positionDGain~%float32 velocityTimeToTargetS~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DriverConfig>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'drives) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DriverConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'DriverConfig
    (cl:cons ':drives (drives msg))
))
