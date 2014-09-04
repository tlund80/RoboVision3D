; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude setFrameRate-request.msg.html

(cl:defclass <setFrameRate-request> (roslisp-msg-protocol:ros-message)
  ((frame_rate
    :reader frame_rate
    :initarg :frame_rate
    :type cl:float
    :initform 0.0))
)

(cl:defclass setFrameRate-request (<setFrameRate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setFrameRate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setFrameRate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setFrameRate-request> is deprecated: use vrm3dvision-srv:setFrameRate-request instead.")))

(cl:ensure-generic-function 'frame_rate-val :lambda-list '(m))
(cl:defmethod frame_rate-val ((m <setFrameRate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:frame_rate-val is deprecated.  Use vrm3dvision-srv:frame_rate instead.")
  (frame_rate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setFrameRate-request>) ostream)
  "Serializes a message object of type '<setFrameRate-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'frame_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setFrameRate-request>) istream)
  "Deserializes a message object of type '<setFrameRate-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frame_rate) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setFrameRate-request>)))
  "Returns string type for a service object of type '<setFrameRate-request>"
  "vrm3dvision/setFrameRateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setFrameRate-request)))
  "Returns string type for a service object of type 'setFrameRate-request"
  "vrm3dvision/setFrameRateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setFrameRate-request>)))
  "Returns md5sum for a message object of type '<setFrameRate-request>"
  "e095c38142cda2c7e70004dbdc099796")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setFrameRate-request)))
  "Returns md5sum for a message object of type 'setFrameRate-request"
  "e095c38142cda2c7e70004dbdc099796")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setFrameRate-request>)))
  "Returns full string definition for message of type '<setFrameRate-request>"
  (cl:format cl:nil "float32 frame_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setFrameRate-request)))
  "Returns full string definition for message of type 'setFrameRate-request"
  (cl:format cl:nil "float32 frame_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setFrameRate-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setFrameRate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setFrameRate-request
    (cl:cons ':frame_rate (frame_rate msg))
))
;//! \htmlinclude setFrameRate-response.msg.html

(cl:defclass <setFrameRate-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setFrameRate-response (<setFrameRate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setFrameRate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setFrameRate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<setFrameRate-response> is deprecated: use vrm3dvision-srv:setFrameRate-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <setFrameRate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setFrameRate-response>) ostream)
  "Serializes a message object of type '<setFrameRate-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setFrameRate-response>) istream)
  "Deserializes a message object of type '<setFrameRate-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setFrameRate-response>)))
  "Returns string type for a service object of type '<setFrameRate-response>"
  "vrm3dvision/setFrameRateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setFrameRate-response)))
  "Returns string type for a service object of type 'setFrameRate-response"
  "vrm3dvision/setFrameRateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setFrameRate-response>)))
  "Returns md5sum for a message object of type '<setFrameRate-response>"
  "e095c38142cda2c7e70004dbdc099796")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setFrameRate-response)))
  "Returns md5sum for a message object of type 'setFrameRate-response"
  "e095c38142cda2c7e70004dbdc099796")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setFrameRate-response>)))
  "Returns full string definition for message of type '<setFrameRate-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setFrameRate-response)))
  "Returns full string definition for message of type 'setFrameRate-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setFrameRate-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setFrameRate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setFrameRate-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setFrameRate)))
  'setFrameRate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setFrameRate)))
  'setFrameRate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setFrameRate)))
  "Returns string type for a service object of type '<setFrameRate>"
  "vrm3dvision/setFrameRate")