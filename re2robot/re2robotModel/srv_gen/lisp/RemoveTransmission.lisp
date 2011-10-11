; Auto-generated. Do not edit!


(cl:in-package re2robotModel-srv)


;//! \htmlinclude RemoveTransmission-request.msg.html

(cl:defclass <RemoveTransmission-request> (roslisp-msg-protocol:ros-message)
  ((joint
    :reader joint
    :initarg :joint
    :type cl:string
    :initform ""))
)

(cl:defclass RemoveTransmission-request (<RemoveTransmission-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RemoveTransmission-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RemoveTransmission-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-srv:<RemoveTransmission-request> is deprecated: use re2robotModel-srv:RemoveTransmission-request instead.")))

(cl:ensure-generic-function 'joint-val :lambda-list '(m))
(cl:defmethod joint-val ((m <RemoveTransmission-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-srv:joint-val is deprecated.  Use re2robotModel-srv:joint instead.")
  (joint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RemoveTransmission-request>) ostream)
  "Serializes a message object of type '<RemoveTransmission-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'joint))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RemoveTransmission-request>) istream)
  "Deserializes a message object of type '<RemoveTransmission-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'joint) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'joint) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RemoveTransmission-request>)))
  "Returns string type for a service object of type '<RemoveTransmission-request>"
  "re2robotModel/RemoveTransmissionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RemoveTransmission-request)))
  "Returns string type for a service object of type 'RemoveTransmission-request"
  "re2robotModel/RemoveTransmissionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RemoveTransmission-request>)))
  "Returns md5sum for a message object of type '<RemoveTransmission-request>"
  "296066438feec30c8905d1fe1042a90b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RemoveTransmission-request)))
  "Returns md5sum for a message object of type 'RemoveTransmission-request"
  "296066438feec30c8905d1fe1042a90b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RemoveTransmission-request>)))
  "Returns full string definition for message of type '<RemoveTransmission-request>"
  (cl:format cl:nil "string joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RemoveTransmission-request)))
  "Returns full string definition for message of type 'RemoveTransmission-request"
  (cl:format cl:nil "string joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RemoveTransmission-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'joint))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RemoveTransmission-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RemoveTransmission-request
    (cl:cons ':joint (joint msg))
))
;//! \htmlinclude RemoveTransmission-response.msg.html

(cl:defclass <RemoveTransmission-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RemoveTransmission-response (<RemoveTransmission-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RemoveTransmission-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RemoveTransmission-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re2robotModel-srv:<RemoveTransmission-response> is deprecated: use re2robotModel-srv:RemoveTransmission-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RemoveTransmission-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re2robotModel-srv:success-val is deprecated.  Use re2robotModel-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RemoveTransmission-response>) ostream)
  "Serializes a message object of type '<RemoveTransmission-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RemoveTransmission-response>) istream)
  "Deserializes a message object of type '<RemoveTransmission-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RemoveTransmission-response>)))
  "Returns string type for a service object of type '<RemoveTransmission-response>"
  "re2robotModel/RemoveTransmissionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RemoveTransmission-response)))
  "Returns string type for a service object of type 'RemoveTransmission-response"
  "re2robotModel/RemoveTransmissionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RemoveTransmission-response>)))
  "Returns md5sum for a message object of type '<RemoveTransmission-response>"
  "296066438feec30c8905d1fe1042a90b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RemoveTransmission-response)))
  "Returns md5sum for a message object of type 'RemoveTransmission-response"
  "296066438feec30c8905d1fe1042a90b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RemoveTransmission-response>)))
  "Returns full string definition for message of type '<RemoveTransmission-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RemoveTransmission-response)))
  "Returns full string definition for message of type 'RemoveTransmission-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RemoveTransmission-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RemoveTransmission-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RemoveTransmission-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RemoveTransmission)))
  'RemoveTransmission-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RemoveTransmission)))
  'RemoveTransmission-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RemoveTransmission)))
  "Returns string type for a service object of type '<RemoveTransmission>"
  "re2robotModel/RemoveTransmission")
