; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude setPattern-request.msg.html

(cl:defclass <setPattern-request> (roslisp-msg-protocol:ros-message)
  ((partial_view
    :reader partial_view
    :initarg :partial_view
    :type cl:boolean
    :initform cl:nil)
   (resolution
    :reader resolution
    :initarg :resolution
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (use_simple_occlusion
    :reader use_simple_occlusion
    :initarg :use_simple_occlusion
    :type cl:boolean
    :initform cl:nil)
   (add_point_colors
    :reader add_point_colors
    :initarg :add_point_colors
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setPattern-request (<setPattern-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setPattern-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setPattern-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setPattern-request> is deprecated: use vrm3dvision-srv:setPattern-request instead.")))

(cl:ensure-generic-function 'partial_view-val :lambda-list '(m))
(cl:defmethod partial_view-val ((m <setPattern-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:partial_view-val is deprecated.  Use vrm3dvision-srv:partial_view instead.")
  (partial_view m))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <setPattern-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:resolution-val is deprecated.  Use vrm3dvision-srv:resolution instead.")
  (resolution m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <setPattern-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:type-val is deprecated.  Use vrm3dvision-srv:type instead.")
  (type m))

(cl:ensure-generic-function 'use_simple_occlusion-val :lambda-list '(m))
(cl:defmethod use_simple_occlusion-val ((m <setPattern-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:use_simple_occlusion-val is deprecated.  Use vrm3dvision-srv:use_simple_occlusion instead.")
  (use_simple_occlusion m))

(cl:ensure-generic-function 'add_point_colors-val :lambda-list '(m))
(cl:defmethod add_point_colors-val ((m <setPattern-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:add_point_colors-val is deprecated.  Use vrm3dvision-srv:add_point_colors instead.")
  (add_point_colors m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<setPattern-request>)))
    "Constants for message type '<setPattern-request>"
  '((:LARGE_GAP_GC . 0)
    (:CONVENTIONAL_GC . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'setPattern-request)))
    "Constants for message type 'setPattern-request"
  '((:LARGE_GAP_GC . 0)
    (:CONVENTIONAL_GC . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setPattern-request>) ostream)
  "Serializes a message object of type '<setPattern-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'partial_view) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'resolution)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_simple_occlusion) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'add_point_colors) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setPattern-request>) istream)
  "Deserializes a message object of type '<setPattern-request>"
    (cl:setf (cl:slot-value msg 'partial_view) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'resolution) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'use_simple_occlusion) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'add_point_colors) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setPattern-request>)))
  "Returns string type for a service object of type '<setPattern-request>"
  "vrm3dvision/setPatternRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setPattern-request)))
  "Returns string type for a service object of type 'setPattern-request"
  "vrm3dvision/setPatternRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setPattern-request>)))
  "Returns md5sum for a message object of type '<setPattern-request>"
  "2251a41e841e515d865157e5f0247c6a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setPattern-request)))
  "Returns md5sum for a message object of type 'setPattern-request"
  "2251a41e841e515d865157e5f0247c6a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setPattern-request>)))
  "Returns full string definition for message of type '<setPattern-request>"
  (cl:format cl:nil "int32 LARGE_GAP_GC = 0~%int32 CONVENTIONAL_GC = 1~%~%bool partial_view~%int32 resolution~%int32 type~%bool use_simple_occlusion~%bool add_point_colors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setPattern-request)))
  "Returns full string definition for message of type 'setPattern-request"
  (cl:format cl:nil "int32 LARGE_GAP_GC = 0~%int32 CONVENTIONAL_GC = 1~%~%bool partial_view~%int32 resolution~%int32 type~%bool use_simple_occlusion~%bool add_point_colors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setPattern-request>))
  (cl:+ 0
     1
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setPattern-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setPattern-request
    (cl:cons ':partial_view (partial_view msg))
    (cl:cons ':resolution (resolution msg))
    (cl:cons ':type (type msg))
    (cl:cons ':use_simple_occlusion (use_simple_occlusion msg))
    (cl:cons ':add_point_colors (add_point_colors msg))
))
;//! \htmlinclude setPattern-response.msg.html

(cl:defclass <setPattern-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setPattern-response (<setPattern-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setPattern-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setPattern-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setPattern-response> is deprecated: use vrm3dvision-srv:setPattern-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <setPattern-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setPattern-response>) ostream)
  "Serializes a message object of type '<setPattern-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setPattern-response>) istream)
  "Deserializes a message object of type '<setPattern-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setPattern-response>)))
  "Returns string type for a service object of type '<setPattern-response>"
  "vrm3dvision/setPatternResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setPattern-response)))
  "Returns string type for a service object of type 'setPattern-response"
  "vrm3dvision/setPatternResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setPattern-response>)))
  "Returns md5sum for a message object of type '<setPattern-response>"
  "2251a41e841e515d865157e5f0247c6a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setPattern-response)))
  "Returns md5sum for a message object of type 'setPattern-response"
  "2251a41e841e515d865157e5f0247c6a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setPattern-response>)))
  "Returns full string definition for message of type '<setPattern-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setPattern-response)))
  "Returns full string definition for message of type 'setPattern-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setPattern-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setPattern-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setPattern-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setPattern)))
  'setPattern-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setPattern)))
  'setPattern-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setPattern)))
  "Returns string type for a service object of type '<setPattern>"
  "vrm3dvision/setPattern")