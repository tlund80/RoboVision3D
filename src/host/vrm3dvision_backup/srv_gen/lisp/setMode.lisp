; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude setMode-request.msg.html

(cl:defclass <setMode-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0))
)

(cl:defclass setMode-request (<setMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setMode-request> is deprecated: use vrm3dvision-srv:setMode-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <setMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:type-val is deprecated.  Use vrm3dvision-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<setMode-request>)))
    "Constants for message type '<setMode-request>"
  '((:STREAMING . 10)
    (:PATTERN . 11)
    (:HDR . 12)
    (:RANDOM_DOT_PATTERN . 13))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'setMode-request)))
    "Constants for message type 'setMode-request"
  '((:STREAMING . 10)
    (:PATTERN . 11)
    (:HDR . 12)
    (:RANDOM_DOT_PATTERN . 13))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setMode-request>) ostream)
  "Serializes a message object of type '<setMode-request>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setMode-request>) istream)
  "Deserializes a message object of type '<setMode-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setMode-request>)))
  "Returns string type for a service object of type '<setMode-request>"
  "vrm3dvision/setModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setMode-request)))
  "Returns string type for a service object of type 'setMode-request"
  "vrm3dvision/setModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setMode-request>)))
  "Returns md5sum for a message object of type '<setMode-request>"
  "301d52529482d266024fa09b9a664426")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setMode-request)))
  "Returns md5sum for a message object of type 'setMode-request"
  "301d52529482d266024fa09b9a664426")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setMode-request>)))
  "Returns full string definition for message of type '<setMode-request>"
  (cl:format cl:nil "int32 STREAMING = 10~%int32 PATTERN = 11~%int32 HDR = 12~%int32 RANDOM_DOT_PATTERN = 13~%~%int32 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setMode-request)))
  "Returns full string definition for message of type 'setMode-request"
  (cl:format cl:nil "int32 STREAMING = 10~%int32 PATTERN = 11~%int32 HDR = 12~%int32 RANDOM_DOT_PATTERN = 13~%~%int32 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setMode-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setMode-request
    (cl:cons ':type (type msg))
))
;//! \htmlinclude setMode-response.msg.html

(cl:defclass <setMode-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setMode-response (<setMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setMode-response> is deprecated: use vrm3dvision-srv:setMode-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <setMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setMode-response>) ostream)
  "Serializes a message object of type '<setMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setMode-response>) istream)
  "Deserializes a message object of type '<setMode-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setMode-response>)))
  "Returns string type for a service object of type '<setMode-response>"
  "vrm3dvision/setModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setMode-response)))
  "Returns string type for a service object of type 'setMode-response"
  "vrm3dvision/setModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setMode-response>)))
  "Returns md5sum for a message object of type '<setMode-response>"
  "301d52529482d266024fa09b9a664426")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setMode-response)))
  "Returns md5sum for a message object of type 'setMode-response"
  "301d52529482d266024fa09b9a664426")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setMode-response>)))
  "Returns full string definition for message of type '<setMode-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setMode-response)))
  "Returns full string definition for message of type 'setMode-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setMode-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setMode)))
  'setMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setMode)))
  'setMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setMode)))
  "Returns string type for a service object of type '<setMode>"
  "vrm3dvision/setMode")