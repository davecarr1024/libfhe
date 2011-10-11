; Auto-generated. Do not edit!


(cl:in-package re2robotModel-srv)


;//! \htmlinclude AddTransmission-request.msg.html

(cl:defclass <AddTransmission-request> (roslisp-msg-protocol:ros-message)
  ((config
    :reader config
    :initarg :config
    :type re2robotModel-msg:TransmissionConfig
    :initform (cl:make-instance 're2robotModel-msg:TransmissionConfig)))
)

(cl:defclass AddTransmission-request (<AddTransmission-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddTransmission-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddTransmission-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-srv:<AddTransmission-request> is deprecated: use re2robotModel-srv:AddTransmission-request instead.")))

(cl:ensure-generic-function 'config-val :lambda-list '(m))
(cl:defmethod config-val ((m <AddTransmission-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-srv:config-val is deprecated.  Use re2robotModel-srv:config instead.")
  (config m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddTransmission-request>) ostream)
  "Serializes a message object of type '<AddTransmission-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'config) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddTransmission-request>) istream)
  "Deserializes a message object of type '<AddTransmission-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'config) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddTransmission-request>)))
  "Returns string type for a service object of type '<AddTransmission-request>"
  "re2robotModel/AddTransmissionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddTransmission-request)))
  "Returns string type for a service object of type 'AddTransmission-request"
  "re2robotModel/AddTransmissionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddTransmission-request>)))
  "Returns md5sum for a message object of type '<AddTransmission-request>"
  "d23cfae343317ef728ad2d22a93edf01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddTransmission-request)))
  "Returns md5sum for a message object of type 'AddTransmission-request"
  "d23cfae343317ef728ad2d22a93edf01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddTransmission-request>)))
  "Returns full string definition for message of type '<AddTransmission-request>"
  (cl:format cl:nil "TransmissionConfig config~%~%================================================================================~%MSG: re2robotModel/TransmissionConfig~%string type~%string drive~%string joint~%float32 timerIntervalS~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddTransmission-request)))
  "Returns full string definition for message of type 'AddTransmission-request"
  (cl:format cl:nil "TransmissionConfig config~%~%================================================================================~%MSG: re2robotModel/TransmissionConfig~%string type~%string drive~%string joint~%float32 timerIntervalS~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddTransmission-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'config))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddTransmission-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddTransmission-request
    (cl:cons ':config (config msg))
))
;//! \htmlinclude AddTransmission-response.msg.html

(cl:defclass <AddTransmission-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AddTransmission-response (<AddTransmission-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddTransmission-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddTransmission-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-srv:<AddTransmission-response> is deprecated: use re2robotModel-srv:AddTransmission-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AddTransmission-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-srv:success-val is deprecated.  Use re2robotModel-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddTransmission-response>) ostream)
  "Serializes a message object of type '<AddTransmission-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddTransmission-response>) istream)
  "Deserializes a message object of type '<AddTransmission-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddTransmission-response>)))
  "Returns string type for a service object of type '<AddTransmission-response>"
  "re2robotModel/AddTransmissionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddTransmission-response)))
  "Returns string type for a service object of type 'AddTransmission-response"
  "re2robotModel/AddTransmissionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddTransmission-response>)))
  "Returns md5sum for a message object of type '<AddTransmission-response>"
  "d23cfae343317ef728ad2d22a93edf01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddTransmission-response)))
  "Returns md5sum for a message object of type 'AddTransmission-response"
  "d23cfae343317ef728ad2d22a93edf01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddTransmission-response>)))
  "Returns full string definition for message of type '<AddTransmission-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddTransmission-response)))
  "Returns full string definition for message of type 'AddTransmission-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddTransmission-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddTransmission-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddTransmission-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddTransmission)))
  'AddTransmission-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddTransmission)))
  'AddTransmission-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddTransmission)))
  "Returns string type for a service object of type '<AddTransmission>"
  "re2robotModel/AddTransmission")
