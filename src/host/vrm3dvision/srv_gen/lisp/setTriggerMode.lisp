; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude setTriggerMode-request.msg.html

(cl:defclass <setTriggerMode-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0))
)

(cl:defclass setTriggerMode-request (<setTriggerMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setTriggerMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setTriggerMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setTriggerMode-request> is deprecated: use vrm3dvision-srv:setTriggerMode-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <setTriggerMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:type-val is deprecated.  Use vrm3dvision-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<setTriggerMode-request>)))
    "Constants for message type '<setTriggerMode-request>"
  '((:INTERNAL_TRIGGER . 1)
    (:EXTERNAL_TRIGGER . 2)
    (:SOFT_TRIGGER . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'setTriggerMode-request)))
    "Constants for message type 'setTriggerMode-request"
  '((:INTERNAL_TRIGGER . 1)
    (:EXTERNAL_TRIGGER . 2)
    (:SOFT_TRIGGER . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setTriggerMode-request>) ostream)
  "Serializes a message object of type '<setTriggerMode-request>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setTriggerMode-request>) istream)
  "Deserializes a message object of type '<setTriggerMode-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setTriggerMode-request>)))
  "Returns string type for a service object of type '<setTriggerMode-request>"
  "vrm3dvision/setTriggerModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setTriggerMode-request)))
  "Returns string type for a service object of type 'setTriggerMode-request"
  "vrm3dvision/setTriggerModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setTriggerMode-request>)))
  "Returns md5sum for a message object of type '<setTriggerMode-request>"
  "f9d7feb10898ffb84ffd482f5c4c9653")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setTriggerMode-request)))
  "Returns md5sum for a message object of type 'setTriggerMode-request"
  "f9d7feb10898ffb84ffd482f5c4c9653")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setTriggerMode-request>)))
  "Returns full string definition for message of type '<setTriggerMode-request>"
  (cl:format cl:nil "int32 INTERNAL_TRIGGER = 1~%int32 EXTERNAL_TRIGGER = 2~%int32 SOFT_TRIGGER = 3~%~%int32 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setTriggerMode-request)))
  "Returns full string definition for message of type 'setTriggerMode-request"
  (cl:format cl:nil "int32 INTERNAL_TRIGGER = 1~%int32 EXTERNAL_TRIGGER = 2~%int32 SOFT_TRIGGER = 3~%~%int32 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setTriggerMode-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setTriggerMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setTriggerMode-request
    (cl:cons ':type (type msg))
))
;//! \htmlinclude setTriggerMode-response.msg.html

(cl:defclass <setTriggerMode-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setTriggerMode-response (<setTriggerMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setTriggerMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setTriggerMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setTriggerMode-response> is deprecated: use vrm3dvision-srv:setTriggerMode-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <setTriggerMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setTriggerMode-response>) ostream)
  "Serializes a message object of type '<setTriggerMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setTriggerMode-response>) istream)
  "Deserializes a message object of type '<setTriggerMode-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setTriggerMode-response>)))
  "Returns string type for a service object of type '<setTriggerMode-response>"
  "vrm3dvision/setTriggerModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setTriggerMode-response)))
  "Returns string type for a service object of type 'setTriggerMode-response"
  "vrm3dvision/setTriggerModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setTriggerMode-response>)))
  "Returns md5sum for a message object of type '<setTriggerMode-response>"
  "f9d7feb10898ffb84ffd482f5c4c9653")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setTriggerMode-response)))
  "Returns md5sum for a message object of type 'setTriggerMode-response"
  "f9d7feb10898ffb84ffd482f5c4c9653")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setTriggerMode-response>)))
  "Returns full string definition for message of type '<setTriggerMode-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setTriggerMode-response)))
  "Returns full string definition for message of type 'setTriggerMode-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setTriggerMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setTriggerMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setTriggerMode-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setTriggerMode)))
  'setTriggerMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setTriggerMode)))
  'setTriggerMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setTriggerMode)))
  "Returns string type for a service object of type '<setTriggerMode>"
  "vrm3dvision/setTriggerMode")