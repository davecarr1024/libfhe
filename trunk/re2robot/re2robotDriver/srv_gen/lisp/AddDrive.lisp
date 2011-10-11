; Auto-generated. Do not edit!


(cl:in-package re2robotDriver-srv)


;//! \htmlinclude AddDrive-request.msg.html

(cl:defclass <AddDrive-request> (roslisp-msg-protocol:ros-message)
  ((config
    :reader config
    :initarg :config
    :type re2robotDriver-msg:DriveConfig
    :initform (cl:make-instance 're2robotDriver-msg:DriveConfig)))
)

(cl:defclass AddDrive-request (<AddDrive-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddDrive-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddDrive-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotDriver-srv:<AddDrive-request> is deprecated: use re2robotDriver-srv:AddDrive-request instead.")))

(cl:ensure-generic-function 'config-val :lambda-list '(m))
(cl:defmethod config-val ((m <AddDrive-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-srv:config-val is deprecated.  Use re2robotDriver-srv:config instead.")
  (config m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddDrive-request>) ostream)
  "Serializes a message object of type '<AddDrive-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'config) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddDrive-request>) istream)
  "Deserializes a message object of type '<AddDrive-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'config) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddDrive-request>)))
  "Returns string type for a service object of type '<AddDrive-request>"
  "re2robotDriver/AddDriveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddDrive-request)))
  "Returns string type for a service object of type 'AddDrive-request"
  "re2robotDriver/AddDriveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddDrive-request>)))
  "Returns md5sum for a message object of type '<AddDrive-request>"
  "70c564ffc604af2a9251b0bf0a28375f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddDrive-request)))
  "Returns md5sum for a message object of type 'AddDrive-request"
  "70c564ffc604af2a9251b0bf0a28375f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddDrive-request>)))
  "Returns full string definition for message of type '<AddDrive-request>"
  (cl:format cl:nil "DriveConfig config~%~%================================================================================~%MSG: re2robotDriver/DriveConfig~%#Drive~%string name~%string type~%float32 updateIntervalS~%float32 cmdTimeoutS~%float32 maxVelocity~%float32 minPosition~%float32 maxPosition~%~%#SimDrive~%float32 initialPosition~%float32 positionPGain~%float32 positionDGain~%float32 velocityTimeToTargetS~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddDrive-request)))
  "Returns full string definition for message of type 'AddDrive-request"
  (cl:format cl:nil "DriveConfig config~%~%================================================================================~%MSG: re2robotDriver/DriveConfig~%#Drive~%string name~%string type~%float32 updateIntervalS~%float32 cmdTimeoutS~%float32 maxVelocity~%float32 minPosition~%float32 maxPosition~%~%#SimDrive~%float32 initialPosition~%float32 positionPGain~%float32 positionDGain~%float32 velocityTimeToTargetS~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddDrive-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'config))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddDrive-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddDrive-request
    (cl:cons ':config (config msg))
))
;//! \htmlinclude AddDrive-response.msg.html

(cl:defclass <AddDrive-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AddDrive-response (<AddDrive-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddDrive-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddDrive-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotDriver-srv:<AddDrive-response> is deprecated: use re2robotDriver-srv:AddDrive-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AddDrive-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotDriver-srv:success-val is deprecated.  Use re2robotDriver-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddDrive-response>) ostream)
  "Serializes a message object of type '<AddDrive-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddDrive-response>) istream)
  "Deserializes a message object of type '<AddDrive-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddDrive-response>)))
  "Returns string type for a service object of type '<AddDrive-response>"
  "re2robotDriver/AddDriveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddDrive-response)))
  "Returns string type for a service object of type 'AddDrive-response"
  "re2robotDriver/AddDriveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddDrive-response>)))
  "Returns md5sum for a message object of type '<AddDrive-response>"
  "70c564ffc604af2a9251b0bf0a28375f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddDrive-response)))
  "Returns md5sum for a message object of type 'AddDrive-response"
  "70c564ffc604af2a9251b0bf0a28375f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddDrive-response>)))
  "Returns full string definition for message of type '<AddDrive-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddDrive-response)))
  "Returns full string definition for message of type 'AddDrive-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddDrive-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddDrive-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddDrive-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddDrive)))
  'AddDrive-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddDrive)))
  'AddDrive-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddDrive)))
  "Returns string type for a service object of type '<AddDrive>"
  "re2robotDriver/AddDrive")
