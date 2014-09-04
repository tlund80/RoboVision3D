; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude stopCamera-request.msg.html

(cl:defclass <stopCamera-request> (roslisp-msg-protocol:ros-message)
  ((camera
    :reader camera
    :initarg :camera
    :type cl:fixnum
    :initform 0))
)

(cl:defclass stopCamera-request (<stopCamera-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <stopCamera-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'stopCamera-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<stopCamera-request> is deprecated: use vrm3dvision-srv:stopCamera-request instead.")))

(cl:ensure-generic-function 'camera-val :lambda-list '(m))
(cl:defmethod camera-val ((m <stopCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:camera-val is deprecated.  Use vrm3dvision-srv:camera instead.")
  (camera m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<stopCamera-request>)))
    "Constants for message type '<stopCamera-request>"
  '((:LEFT_CAMERA . 1)
    (:RIGHT_CAMERA . 2)
    (:COLOR_CAMERA . 4)
    (:ALL_CAMERAS . 8))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'stopCamera-request)))
    "Constants for message type 'stopCamera-request"
  '((:LEFT_CAMERA . 1)
    (:RIGHT_CAMERA . 2)
    (:COLOR_CAMERA . 4)
    (:ALL_CAMERAS . 8))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <stopCamera-request>) ostream)
  "Serializes a message object of type '<stopCamera-request>"
  (cl:let* ((signed (cl:slot-value msg 'camera)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <stopCamera-request>) istream)
  "Deserializes a message object of type '<stopCamera-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'camera) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<stopCamera-request>)))
  "Returns string type for a service object of type '<stopCamera-request>"
  "vrm3dvision/stopCameraRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'stopCamera-request)))
  "Returns string type for a service object of type 'stopCamera-request"
  "vrm3dvision/stopCameraRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<stopCamera-request>)))
  "Returns md5sum for a message object of type '<stopCamera-request>"
  "7ab4d71c90564909e360a8b8d9fc499c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'stopCamera-request)))
  "Returns md5sum for a message object of type 'stopCamera-request"
  "7ab4d71c90564909e360a8b8d9fc499c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<stopCamera-request>)))
  "Returns full string definition for message of type '<stopCamera-request>"
  (cl:format cl:nil "int8 LEFT_CAMERA = 1~%int8 RIGHT_CAMERA = 2~%int8 COLOR_CAMERA = 4~%int8 ALL_CAMERAS = 8~%~%int8 camera~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'stopCamera-request)))
  "Returns full string definition for message of type 'stopCamera-request"
  (cl:format cl:nil "int8 LEFT_CAMERA = 1~%int8 RIGHT_CAMERA = 2~%int8 COLOR_CAMERA = 4~%int8 ALL_CAMERAS = 8~%~%int8 camera~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <stopCamera-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <stopCamera-request>))
  "Converts a ROS message object to a list"
  (cl:list 'stopCamera-request
    (cl:cons ':camera (camera msg))
))
;//! \htmlinclude stopCamera-response.msg.html

(cl:defclass <stopCamera-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass stopCamera-response (<stopCamera-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <stopCamera-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'stopCamera-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<stopCamera-response> is deprecated: use vrm3dvision-srv:stopCamera-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <stopCamera-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <stopCamera-response>) ostream)
  "Serializes a message object of type '<stopCamera-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <stopCamera-response>) istream)
  "Deserializes a message object of type '<stopCamera-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<stopCamera-response>)))
  "Returns string type for a service object of type '<stopCamera-response>"
  "vrm3dvision/stopCameraResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'stopCamera-response)))
  "Returns string type for a service object of type 'stopCamera-response"
  "vrm3dvision/stopCameraResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<stopCamera-response>)))
  "Returns md5sum for a message object of type '<stopCamera-response>"
  "7ab4d71c90564909e360a8b8d9fc499c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'stopCamera-response)))
  "Returns md5sum for a message object of type 'stopCamera-response"
  "7ab4d71c90564909e360a8b8d9fc499c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<stopCamera-response>)))
  "Returns full string definition for message of type '<stopCamera-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'stopCamera-response)))
  "Returns full string definition for message of type 'stopCamera-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <stopCamera-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <stopCamera-response>))
  "Converts a ROS message object to a list"
  (cl:list 'stopCamera-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'stopCamera)))
  'stopCamera-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'stopCamera)))
  'stopCamera-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'stopCamera)))
  "Returns string type for a service object of type '<stopCamera>"
  "vrm3dvision/stopCamera")