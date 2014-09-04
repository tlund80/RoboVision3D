; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude setActiveCameras-request.msg.html

(cl:defclass <setActiveCameras-request> (roslisp-msg-protocol:ros-message)
  ((cameras
    :reader cameras
    :initarg :cameras
    :type cl:integer
    :initform 0))
)

(cl:defclass setActiveCameras-request (<setActiveCameras-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setActiveCameras-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setActiveCameras-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setActiveCameras-request> is deprecated: use vrm3dvision-srv:setActiveCameras-request instead.")))

(cl:ensure-generic-function 'cameras-val :lambda-list '(m))
(cl:defmethod cameras-val ((m <setActiveCameras-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:cameras-val is deprecated.  Use vrm3dvision-srv:cameras instead.")
  (cameras m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<setActiveCameras-request>)))
    "Constants for message type '<setActiveCameras-request>"
  '((:NONE . 0)
    (:LEFT_CENTER_RIGHT . 1)
    (:LEFT_RIGHT . 2)
    (:LEFT_CENTER . 3)
    (:CENTER_RIGHT . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'setActiveCameras-request)))
    "Constants for message type 'setActiveCameras-request"
  '((:NONE . 0)
    (:LEFT_CENTER_RIGHT . 1)
    (:LEFT_RIGHT . 2)
    (:LEFT_CENTER . 3)
    (:CENTER_RIGHT . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setActiveCameras-request>) ostream)
  "Serializes a message object of type '<setActiveCameras-request>"
  (cl:let* ((signed (cl:slot-value msg 'cameras)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setActiveCameras-request>) istream)
  "Deserializes a message object of type '<setActiveCameras-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cameras) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setActiveCameras-request>)))
  "Returns string type for a service object of type '<setActiveCameras-request>"
  "vrm3dvision/setActiveCamerasRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setActiveCameras-request)))
  "Returns string type for a service object of type 'setActiveCameras-request"
  "vrm3dvision/setActiveCamerasRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setActiveCameras-request>)))
  "Returns md5sum for a message object of type '<setActiveCameras-request>"
  "e1e2698fca1f66ed6e3d47581a4b30bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setActiveCameras-request)))
  "Returns md5sum for a message object of type 'setActiveCameras-request"
  "e1e2698fca1f66ed6e3d47581a4b30bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setActiveCameras-request>)))
  "Returns full string definition for message of type '<setActiveCameras-request>"
  (cl:format cl:nil "int32 NONE = 0~%int32 LEFT_CENTER_RIGHT = 1~%int32 LEFT_RIGHT = 2~%int32 LEFT_CENTER = 3~%int32 CENTER_RIGHT = 4~%~%int32 cameras~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setActiveCameras-request)))
  "Returns full string definition for message of type 'setActiveCameras-request"
  (cl:format cl:nil "int32 NONE = 0~%int32 LEFT_CENTER_RIGHT = 1~%int32 LEFT_RIGHT = 2~%int32 LEFT_CENTER = 3~%int32 CENTER_RIGHT = 4~%~%int32 cameras~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setActiveCameras-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setActiveCameras-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setActiveCameras-request
    (cl:cons ':cameras (cameras msg))
))
;//! \htmlinclude setActiveCameras-response.msg.html

(cl:defclass <setActiveCameras-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setActiveCameras-response (<setActiveCameras-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setActiveCameras-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setActiveCameras-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setActiveCameras-response> is deprecated: use vrm3dvision-srv:setActiveCameras-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <setActiveCameras-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setActiveCameras-response>) ostream)
  "Serializes a message object of type '<setActiveCameras-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setActiveCameras-response>) istream)
  "Deserializes a message object of type '<setActiveCameras-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setActiveCameras-response>)))
  "Returns string type for a service object of type '<setActiveCameras-response>"
  "vrm3dvision/setActiveCamerasResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setActiveCameras-response)))
  "Returns string type for a service object of type 'setActiveCameras-response"
  "vrm3dvision/setActiveCamerasResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setActiveCameras-response>)))
  "Returns md5sum for a message object of type '<setActiveCameras-response>"
  "e1e2698fca1f66ed6e3d47581a4b30bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setActiveCameras-response)))
  "Returns md5sum for a message object of type 'setActiveCameras-response"
  "e1e2698fca1f66ed6e3d47581a4b30bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setActiveCameras-response>)))
  "Returns full string definition for message of type '<setActiveCameras-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setActiveCameras-response)))
  "Returns full string definition for message of type 'setActiveCameras-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setActiveCameras-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setActiveCameras-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setActiveCameras-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setActiveCameras)))
  'setActiveCameras-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setActiveCameras)))
  'setActiveCameras-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setActiveCameras)))
  "Returns string type for a service object of type '<setActiveCameras>"
  "vrm3dvision/setActiveCameras")