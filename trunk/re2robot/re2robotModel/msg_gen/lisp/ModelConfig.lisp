; Auto-generated. Do not edit!


(cl:in-package re2robotModel-msg)


;//! \htmlinclude ModelConfig.msg.html

(cl:defclass <ModelConfig> (roslisp-msg-protocol:ros-message)
  ((transmissions
    :reader transmissions
    :initarg :transmissions
    :type (cl:vector re2robotModel-msg:TransmissionConfig)
   :initform (cl:make-array 0 :element-type 're2robotModel-msg:TransmissionConfig :initial-element (cl:make-instance 're2robotModel-msg:TransmissionConfig)))
   (arbiters
    :reader arbiters
    :initarg :arbiters
    :type (cl:vector re2robotModel-msg:ArbiterConfig)
   :initform (cl:make-array 0 :element-type 're2robotModel-msg:ArbiterConfig :initial-element (cl:make-instance 're2robotModel-msg:ArbiterConfig))))
)

(cl:defclass ModelConfig (<ModelConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModelConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModelConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-msg:<ModelConfig> is deprecated: use re2robotModel-msg:ModelConfig instead.")))

(cl:ensure-generic-function 'transmissions-val :lambda-list '(m))
(cl:defmethod transmissions-val ((m <ModelConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:transmissions-val is deprecated.  Use re2robotModel-msg:transmissions instead.")
  (transmissions m))

(cl:ensure-generic-function 'arbiters-val :lambda-list '(m))
(cl:defmethod arbiters-val ((m <ModelConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-msg:arbiters-val is deprecated.  Use re2robotModel-msg:arbiters instead.")
  (arbiters m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModelConfig>) ostream)
  "Serializes a message object of type '<ModelConfig>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'transmissions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'transmissions))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'arbiters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'arbiters))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModelConfig>) istream)
  "Deserializes a message object of type '<ModelConfig>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'transmissions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'transmissions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 're2robotModel-msg:TransmissionConfig))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'arbiters) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'arbiters)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 're2robotModel-msg:ArbiterConfig))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModelConfig>)))
  "Returns string type for a message object of type '<ModelConfig>"
  "re2robotModel/ModelConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelConfig)))
  "Returns string type for a message object of type 'ModelConfig"
  "re2robotModel/ModelConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModelConfig>)))
  "Returns md5sum for a message object of type '<ModelConfig>"
  "417ae4eb253d437dfccedc437013e3aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModelConfig)))
  "Returns md5sum for a message object of type 'ModelConfig"
  "417ae4eb253d437dfccedc437013e3aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModelConfig>)))
  "Returns full string definition for message of type '<ModelConfig>"
  (cl:format cl:nil "TransmissionConfig[] transmissions~%ArbiterConfig[] arbiters~%================================================================================~%MSG: re2robotModel/TransmissionConfig~%string type~%string drive~%string joint~%float32 timerIntervalS~%================================================================================~%MSG: re2robotModel/ArbiterConfig~%#common~%string type~%float32 updateIntervalS~%~%#control arbiter~%string initialController~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModelConfig)))
  "Returns full string definition for message of type 'ModelConfig"
  (cl:format cl:nil "TransmissionConfig[] transmissions~%ArbiterConfig[] arbiters~%================================================================================~%MSG: re2robotModel/TransmissionConfig~%string type~%string drive~%string joint~%float32 timerIntervalS~%================================================================================~%MSG: re2robotModel/ArbiterConfig~%#common~%string type~%float32 updateIntervalS~%~%#control arbiter~%string initialController~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModelConfig>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'transmissions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'arbiters) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModelConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'ModelConfig
    (cl:cons ':transmissions (transmissions msg))
    (cl:cons ':arbiters (arbiters msg))
))
