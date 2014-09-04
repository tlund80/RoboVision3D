; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude setExposure-request.msg.html

(cl:defclass <setExposure-request> (roslisp-msg-protocol:ros-message)
  ((camera
    :reader camera
    :initarg :camera
    :type cl:integer
    :initform 0)
   (exposure
    :reader exposure
    :initarg :exposure
    :type cl:string
    :initform ""))
)

(cl:defclass setExposure-request (<setExposure-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setExposure-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setExposure-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setExposure-request> is deprecated: use vrm3dvision-srv:setExposure-request instead.")))

(cl:ensure-generic-function 'camera-val :lambda-list '(m))
(cl:defmethod camera-val ((m <setExposure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:camera-val is deprecated.  Use vrm3dvision-srv:camera instead.")
  (camera m))

(cl:ensure-generic-function 'exposure-val :lambda-list '(m))
(cl:defmethod exposure-val ((m <setExposure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:exposure-val is deprecated.  Use vrm3dvision-srv:exposure instead.")
  (exposure m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<setExposure-request>)))
    "Constants for message type '<setExposure-request>"
  '((:LEFT_CAMERA . 1)
    (:RIGHT_CAMERA . 2)
    (:LEFT_AND_RIGHT_CAMERA . 3)
    (:COLOR_CAMERA . 4)
    (:ALL_CAMERAS . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'setExposure-request)))
    "Constants for message type 'setExposure-request"
  '((:LEFT_CAMERA . 1)
    (:RIGHT_CAMERA . 2)
    (:LEFT_AND_RIGHT_CAMERA . 3)
    (:COLOR_CAMERA . 4)
    (:ALL_CAMERAS . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setExposure-request>) ostream)
  "Serializes a message object of type '<setExposure-request>"
  (cl:let* ((signed (cl:slot-value msg 'camera)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'exposure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'exposure))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setExposure-request>) istream)
  "Deserializes a message object of type '<setExposure-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'camera) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'exposure) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'exposure) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setExposure-request>)))
  "Returns string type for a service object of type '<setExposure-request>"
  "vrm3dvision/setExposureRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setExposure-request)))
  "Returns string type for a service object of type 'setExposure-request"
  "vrm3dvision/setExposureRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setExposure-request>)))
  "Returns md5sum for a message object of type '<setExposure-request>"
  "5ba508c10b2f392acb59bcd971e154d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setExposure-request)))
  "Returns md5sum for a message object of type 'setExposure-request"
  "5ba508c10b2f392acb59bcd971e154d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setExposure-request>)))
  "Returns full string definition for message of type '<setExposure-request>"
  (cl:format cl:nil "int32 LEFT_CAMERA = 1~%int32 RIGHT_CAMERA = 2~%int32 LEFT_AND_RIGHT_CAMERA = 3~%int32 COLOR_CAMERA = 4~%int32 ALL_CAMERAS = 7~%~%int32 camera~%string exposure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setExposure-request)))
  "Returns full string definition for message of type 'setExposure-request"
  (cl:format cl:nil "int32 LEFT_CAMERA = 1~%int32 RIGHT_CAMERA = 2~%int32 LEFT_AND_RIGHT_CAMERA = 3~%int32 COLOR_CAMERA = 4~%int32 ALL_CAMERAS = 7~%~%int32 camera~%string exposure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setExposure-request>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'exposure))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setExposure-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setExposure-request
    (cl:cons ':camera (camera msg))
    (cl:cons ':exposure (exposure msg))
))
;//! \htmlinclude setExposure-response.msg.html

(cl:defclass <setExposure-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setExposure-response (<setExposure-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setExposure-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setExposure-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setExposure-response> is deprecated: use vrm3dvision-srv:setExposure-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <setExposure-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setExposure-response>) ostream)
  "Serializes a message object of type '<setExposure-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setExposure-response>) istream)
  "Deserializes a message object of type '<setExposure-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setExposure-response>)))
  "Returns string type for a service object of type '<setExposure-response>"
  "vrm3dvision/setExposureResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setExposure-response)))
  "Returns string type for a service object of type 'setExposure-response"
  "vrm3dvision/setExposureResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setExposure-response>)))
  "Returns md5sum for a message object of type '<setExposure-response>"
  "5ba508c10b2f392acb59bcd971e154d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setExposure-response)))
  "Returns md5sum for a message object of type 'setExposure-response"
  "5ba508c10b2f392acb59bcd971e154d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setExposure-response>)))
  "Returns full string definition for message of type '<setExposure-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setExposure-response)))
  "Returns full string definition for message of type 'setExposure-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setExposure-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setExposure-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setExposure-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setExposure)))
  'setExposure-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setExposure)))
  'setExposure-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setExposure)))
  "Returns string type for a service object of type '<setExposure>"
  "vrm3dvision/setExposure")