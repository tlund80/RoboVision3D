; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude startCamera-request.msg.html

(cl:defclass <startCamera-request> (roslisp-msg-protocol:ros-message)
  ((camera
    :reader camera
    :initarg :camera
    :type cl:fixnum
    :initform 0))
)

(cl:defclass startCamera-request (<startCamera-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <startCamera-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'startCamera-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<startCamera-request> is deprecated: use vrm3dvision-srv:startCamera-request instead.")))

(cl:ensure-generic-function 'camera-val :lambda-list '(m))
(cl:defmethod camera-val ((m <startCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:camera-val is deprecated.  Use vrm3dvision-srv:camera instead.")
  (camera m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<startCamera-request>)))
    "Constants for message type '<startCamera-request>"
  '((:LEFT_CAMERA . 1)
    (:RIGHT_CAMERA . 2)
    (:COLOR_CAMERA . 4)
    (:ALL_CAMERAS . 8))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'startCamera-request)))
    "Constants for message type 'startCamera-request"
  '((:LEFT_CAMERA . 1)
    (:RIGHT_CAMERA . 2)
    (:COLOR_CAMERA . 4)
    (:ALL_CAMERAS . 8))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <startCamera-request>) ostream)
  "Serializes a message object of type '<startCamera-request>"
  (cl:let* ((signed (cl:slot-value msg 'camera)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <startCamera-request>) istream)
  "Deserializes a message object of type '<startCamera-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'camera) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<startCamera-request>)))
  "Returns string type for a service object of type '<startCamera-request>"
  "vrm3dvision/startCameraRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'startCamera-request)))
  "Returns string type for a service object of type 'startCamera-request"
  "vrm3dvision/startCameraRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<startCamera-request>)))
  "Returns md5sum for a message object of type '<startCamera-request>"
  "7ab4d71c90564909e360a8b8d9fc499c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'startCamera-request)))
  "Returns md5sum for a message object of type 'startCamera-request"
  "7ab4d71c90564909e360a8b8d9fc499c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<startCamera-request>)))
  "Returns full string definition for message of type '<startCamera-request>"
  (cl:format cl:nil "int8 LEFT_CAMERA = 1~%int8 RIGHT_CAMERA = 2~%int8 COLOR_CAMERA = 4~%int8 ALL_CAMERAS = 8~%~%int8 camera~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'startCamera-request)))
  "Returns full string definition for message of type 'startCamera-request"
  (cl:format cl:nil "int8 LEFT_CAMERA = 1~%int8 RIGHT_CAMERA = 2~%int8 COLOR_CAMERA = 4~%int8 ALL_CAMERAS = 8~%~%int8 camera~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <startCamera-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <startCamera-request>))
  "Converts a ROS message object to a list"
  (cl:list 'startCamera-request
    (cl:cons ':camera (camera msg))
))
;//! \htmlinclude startCamera-response.msg.html

(cl:defclass <startCamera-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass startCamera-response (<startCamera-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <startCamera-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'startCamera-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<startCamera-response> is deprecated: use vrm3dvision-srv:startCamera-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <startCamera-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <startCamera-response>) ostream)
  "Serializes a message object of type '<startCamera-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <startCamera-response>) istream)
  "Deserializes a message object of type '<startCamera-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<startCamera-response>)))
  "Returns string type for a service object of type '<startCamera-response>"
  "vrm3dvision/startCameraResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'startCamera-response)))
  "Returns string type for a service object of type 'startCamera-response"
  "vrm3dvision/startCameraResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<startCamera-response>)))
  "Returns md5sum for a message object of type '<startCamera-response>"
  "7ab4d71c90564909e360a8b8d9fc499c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'startCamera-response)))
  "Returns md5sum for a message object of type 'startCamera-response"
  "7ab4d71c90564909e360a8b8d9fc499c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<startCamera-response>)))
  "Returns full string definition for message of type '<startCamera-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'startCamera-response)))
  "Returns full string definition for message of type 'startCamera-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <startCamera-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <startCamera-response>))
  "Converts a ROS message object to a list"
  (cl:list 'startCamera-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'startCamera)))
  'startCamera-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'startCamera)))
  'startCamera-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'startCamera)))
  "Returns string type for a service object of type '<startCamera>"
  "vrm3dvision/startCamera")