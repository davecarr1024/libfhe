; Auto-generated. Do not edit!


(cl:in-package re2robotModel-srv)


;//! \htmlinclude RemoveArbiter-request.msg.html

(cl:defclass <RemoveArbiter-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform ""))
)

(cl:defclass RemoveArbiter-request (<RemoveArbiter-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RemoveArbiter-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RemoveArbiter-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-srv:<RemoveArbiter-request> is deprecated: use re2robotModel-srv:RemoveArbiter-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <RemoveArbiter-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-srv:type-val is deprecated.  Use re2robotModel-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RemoveArbiter-request>) ostream)
  "Serializes a message object of type '<RemoveArbiter-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RemoveArbiter-request>) istream)
  "Deserializes a message object of type '<RemoveArbiter-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RemoveArbiter-request>)))
  "Returns string type for a service object of type '<RemoveArbiter-request>"
  "re2robotModel/RemoveArbiterRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RemoveArbiter-request)))
  "Returns string type for a service object of type 'RemoveArbiter-request"
  "re2robotModel/RemoveArbiterRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RemoveArbiter-request>)))
  "Returns md5sum for a message object of type '<RemoveArbiter-request>"
  "6f79030b38c16e3be13c8c4e4b634640")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RemoveArbiter-request)))
  "Returns md5sum for a message object of type 'RemoveArbiter-request"
  "6f79030b38c16e3be13c8c4e4b634640")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RemoveArbiter-request>)))
  "Returns full string definition for message of type '<RemoveArbiter-request>"
  (cl:format cl:nil "string type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RemoveArbiter-request)))
  "Returns full string definition for message of type 'RemoveArbiter-request"
  (cl:format cl:nil "string type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RemoveArbiter-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RemoveArbiter-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RemoveArbiter-request
    (cl:cons ':type (type msg))
))
;//! \htmlinclude RemoveArbiter-response.msg.html

(cl:defclass <RemoveArbiter-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RemoveArbiter-response (<RemoveArbiter-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RemoveArbiter-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RemoveArbiter-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-srv:<RemoveArbiter-response> is deprecated: use re2robotModel-srv:RemoveArbiter-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RemoveArbiter-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-srv:success-val is deprecated.  Use re2robotModel-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RemoveArbiter-response>) ostream)
  "Serializes a message object of type '<RemoveArbiter-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RemoveArbiter-response>) istream)
  "Deserializes a message object of type '<RemoveArbiter-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RemoveArbiter-response>)))
  "Returns string type for a service object of type '<RemoveArbiter-response>"
  "re2robotModel/RemoveArbiterResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RemoveArbiter-response)))
  "Returns string type for a service object of type 'RemoveArbiter-response"
  "re2robotModel/RemoveArbiterResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RemoveArbiter-response>)))
  "Returns md5sum for a message object of type '<RemoveArbiter-response>"
  "6f79030b38c16e3be13c8c4e4b634640")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RemoveArbiter-response)))
  "Returns md5sum for a message object of type 'RemoveArbiter-response"
  "6f79030b38c16e3be13c8c4e4b634640")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RemoveArbiter-response>)))
  "Returns full string definition for message of type '<RemoveArbiter-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RemoveArbiter-response)))
  "Returns full string definition for message of type 'RemoveArbiter-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RemoveArbiter-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RemoveArbiter-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RemoveArbiter-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RemoveArbiter)))
  'RemoveArbiter-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RemoveArbiter)))
  'RemoveArbiter-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RemoveArbiter)))
  "Returns string type for a service object of type '<RemoveArbiter>"
  "re2robotModel/RemoveArbiter")
