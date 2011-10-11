; Auto-generated. Do not edit!


(cl:in-package re2robotModel-srv)


;//! \htmlinclude AddArbiter-request.msg.html

(cl:defclass <AddArbiter-request> (roslisp-msg-protocol:ros-message)
  ((config
    :reader config
    :initarg :config
    :type re2robotModel-msg:ArbiterConfig
    :initform (cl:make-instance 're2robotModel-msg:ArbiterConfig)))
)

(cl:defclass AddArbiter-request (<AddArbiter-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddArbiter-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddArbiter-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-srv:<AddArbiter-request> is deprecated: use re2robotModel-srv:AddArbiter-request instead.")))

(cl:ensure-generic-function 'config-val :lambda-list '(m))
(cl:defmethod config-val ((m <AddArbiter-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-srv:config-val is deprecated.  Use re2robotModel-srv:config instead.")
  (config m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddArbiter-request>) ostream)
  "Serializes a message object of type '<AddArbiter-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'config) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddArbiter-request>) istream)
  "Deserializes a message object of type '<AddArbiter-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'config) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddArbiter-request>)))
  "Returns string type for a service object of type '<AddArbiter-request>"
  "re2robotModel/AddArbiterRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddArbiter-request)))
  "Returns string type for a service object of type 'AddArbiter-request"
  "re2robotModel/AddArbiterRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddArbiter-request>)))
  "Returns md5sum for a message object of type '<AddArbiter-request>"
  "0ed5e0636f291e4517e8f4f37018f3dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddArbiter-request)))
  "Returns md5sum for a message object of type 'AddArbiter-request"
  "0ed5e0636f291e4517e8f4f37018f3dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddArbiter-request>)))
  "Returns full string definition for message of type '<AddArbiter-request>"
  (cl:format cl:nil "ArbiterConfig config~%~%================================================================================~%MSG: re2robotModel/ArbiterConfig~%#common~%string type~%float32 updateIntervalS~%~%#control arbiter~%string initialController~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddArbiter-request)))
  "Returns full string definition for message of type 'AddArbiter-request"
  (cl:format cl:nil "ArbiterConfig config~%~%================================================================================~%MSG: re2robotModel/ArbiterConfig~%#common~%string type~%float32 updateIntervalS~%~%#control arbiter~%string initialController~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddArbiter-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'config))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddArbiter-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddArbiter-request
    (cl:cons ':config (config msg))
))
;//! \htmlinclude AddArbiter-response.msg.html

(cl:defclass <AddArbiter-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AddArbiter-response (<AddArbiter-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddArbiter-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddArbiter-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-srv:<AddArbiter-response> is deprecated: use re2robotModel-srv:AddArbiter-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AddArbiter-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-srv:success-val is deprecated.  Use re2robotModel-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddArbiter-response>) ostream)
  "Serializes a message object of type '<AddArbiter-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddArbiter-response>) istream)
  "Deserializes a message object of type '<AddArbiter-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddArbiter-response>)))
  "Returns string type for a service object of type '<AddArbiter-response>"
  "re2robotModel/AddArbiterResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddArbiter-response)))
  "Returns string type for a service object of type 'AddArbiter-response"
  "re2robotModel/AddArbiterResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddArbiter-response>)))
  "Returns md5sum for a message object of type '<AddArbiter-response>"
  "0ed5e0636f291e4517e8f4f37018f3dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddArbiter-response)))
  "Returns md5sum for a message object of type 'AddArbiter-response"
  "0ed5e0636f291e4517e8f4f37018f3dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddArbiter-response>)))
  "Returns full string definition for message of type '<AddArbiter-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddArbiter-response)))
  "Returns full string definition for message of type 'AddArbiter-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddArbiter-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddArbiter-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddArbiter-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddArbiter)))
  'AddArbiter-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddArbiter)))
  'AddArbiter-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddArbiter)))
  "Returns string type for a service object of type '<AddArbiter>"
  "re2robotModel/AddArbiter")
