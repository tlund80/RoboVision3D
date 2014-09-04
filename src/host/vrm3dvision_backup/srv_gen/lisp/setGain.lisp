; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude setGain-request.msg.html

(cl:defclass <setGain-request> (roslisp-msg-protocol:ros-message)
  ((camera
    :reader camera
    :initarg :camera
    :type cl:fixnum
    :initform 0)
   (gain
    :reader gain
    :initarg :gain
    :type cl:float
    :initform 0.0))
)

(cl:defclass setGain-request (<setGain-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setGain-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setGain-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setGain-request> is deprecated: use vrm3dvision-srv:setGain-request instead.")))

(cl:ensure-generic-function 'camera-val :lambda-list '(m))
(cl:defmethod camera-val ((m <setGain-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:camera-val is deprecated.  Use vrm3dvision-srv:camera instead.")
  (camera m))

(cl:ensure-generic-function 'gain-val :lambda-list '(m))
(cl:defmethod gain-val ((m <setGain-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:gain-val is deprecated.  Use vrm3dvision-srv:gain instead.")
  (gain m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<setGain-request>)))
    "Constants for message type '<setGain-request>"
  '((:LEFT_CAMERA . 1)
    (:RIGHT_CAMERA . 2)
    (:COLOR_CAMERA . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'setGain-request)))
    "Constants for message type 'setGain-request"
  '((:LEFT_CAMERA . 1)
    (:RIGHT_CAMERA . 2)
    (:COLOR_CAMERA . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setGain-request>) ostream)
  "Serializes a message object of type '<setGain-request>"
  (cl:let* ((signed (cl:slot-value msg 'camera)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setGain-request>) istream)
  "Deserializes a message object of type '<setGain-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'camera) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gain) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setGain-request>)))
  "Returns string type for a service object of type '<setGain-request>"
  "vrm3dvision/setGainRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setGain-request)))
  "Returns string type for a service object of type 'setGain-request"
  "vrm3dvision/setGainRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setGain-request>)))
  "Returns md5sum for a message object of type '<setGain-request>"
  "ed8bcecf8b13e55a92dc371339e8422c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setGain-request)))
  "Returns md5sum for a message object of type 'setGain-request"
  "ed8bcecf8b13e55a92dc371339e8422c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setGain-request>)))
  "Returns full string definition for message of type '<setGain-request>"
  (cl:format cl:nil "int8 LEFT_CAMERA = 1~%int8 RIGHT_CAMERA = 2~%int8 COLOR_CAMERA = 4~%~%int8 camera~%~%float32 gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setGain-request)))
  "Returns full string definition for message of type 'setGain-request"
  (cl:format cl:nil "int8 LEFT_CAMERA = 1~%int8 RIGHT_CAMERA = 2~%int8 COLOR_CAMERA = 4~%~%int8 camera~%~%float32 gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setGain-request>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setGain-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setGain-request
    (cl:cons ':camera (camera msg))
    (cl:cons ':gain (gain msg))
))
;//! \htmlinclude setGain-response.msg.html

(cl:defclass <setGain-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setGain-response (<setGain-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setGain-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setGain-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setGain-response> is deprecated: use vrm3dvision-srv:setGain-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <setGain-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setGain-response>) ostream)
  "Serializes a message object of type '<setGain-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setGain-response>) istream)
  "Deserializes a message object of type '<setGain-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setGain-response>)))
  "Returns string type for a service object of type '<setGain-response>"
  "vrm3dvision/setGainResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setGain-response)))
  "Returns string type for a service object of type 'setGain-response"
  "vrm3dvision/setGainResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setGain-response>)))
  "Returns md5sum for a message object of type '<setGain-response>"
  "ed8bcecf8b13e55a92dc371339e8422c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setGain-response)))
  "Returns md5sum for a message object of type 'setGain-response"
  "ed8bcecf8b13e55a92dc371339e8422c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setGain-response>)))
  "Returns full string definition for message of type '<setGain-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setGain-response)))
  "Returns full string definition for message of type 'setGain-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setGain-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setGain-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setGain-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setGain)))
  'setGain-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setGain)))
  'setGain-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setGain)))
  "Returns string type for a service object of type '<setGain>"
  "vrm3dvision/setGain")