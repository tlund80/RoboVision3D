; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude triggerCamera-request.msg.html

(cl:defclass <triggerCamera-request> (roslisp-msg-protocol:ros-message)
  ((trigger
    :reader trigger
    :initarg :trigger
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass triggerCamera-request (<triggerCamera-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <triggerCamera-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'triggerCamera-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<triggerCamera-request> is deprecated: use vrm3dvision-srv:triggerCamera-request instead.")))

(cl:ensure-generic-function 'trigger-val :lambda-list '(m))
(cl:defmethod trigger-val ((m <triggerCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:trigger-val is deprecated.  Use vrm3dvision-srv:trigger instead.")
  (trigger m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <triggerCamera-request>) ostream)
  "Serializes a message object of type '<triggerCamera-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'trigger) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <triggerCamera-request>) istream)
  "Deserializes a message object of type '<triggerCamera-request>"
    (cl:setf (cl:slot-value msg 'trigger) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<triggerCamera-request>)))
  "Returns string type for a service object of type '<triggerCamera-request>"
  "vrm3dvision/triggerCameraRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'triggerCamera-request)))
  "Returns string type for a service object of type 'triggerCamera-request"
  "vrm3dvision/triggerCameraRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<triggerCamera-request>)))
  "Returns md5sum for a message object of type '<triggerCamera-request>"
  "ac73de7c264c9f54be33ec7cca98a512")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'triggerCamera-request)))
  "Returns md5sum for a message object of type 'triggerCamera-request"
  "ac73de7c264c9f54be33ec7cca98a512")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<triggerCamera-request>)))
  "Returns full string definition for message of type '<triggerCamera-request>"
  (cl:format cl:nil "bool trigger~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'triggerCamera-request)))
  "Returns full string definition for message of type 'triggerCamera-request"
  (cl:format cl:nil "bool trigger~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <triggerCamera-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <triggerCamera-request>))
  "Converts a ROS message object to a list"
  (cl:list 'triggerCamera-request
    (cl:cons ':trigger (trigger msg))
))
;//! \htmlinclude triggerCamera-response.msg.html

(cl:defclass <triggerCamera-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass triggerCamera-response (<triggerCamera-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <triggerCamera-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'triggerCamera-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<triggerCamera-response> is deprecated: use vrm3dvision-srv:triggerCamera-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <triggerCamera-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <triggerCamera-response>) ostream)
  "Serializes a message object of type '<triggerCamera-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <triggerCamera-response>) istream)
  "Deserializes a message object of type '<triggerCamera-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<triggerCamera-response>)))
  "Returns string type for a service object of type '<triggerCamera-response>"
  "vrm3dvision/triggerCameraResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'triggerCamera-response)))
  "Returns string type for a service object of type 'triggerCamera-response"
  "vrm3dvision/triggerCameraResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<triggerCamera-response>)))
  "Returns md5sum for a message object of type '<triggerCamera-response>"
  "ac73de7c264c9f54be33ec7cca98a512")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'triggerCamera-response)))
  "Returns md5sum for a message object of type 'triggerCamera-response"
  "ac73de7c264c9f54be33ec7cca98a512")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<triggerCamera-response>)))
  "Returns full string definition for message of type '<triggerCamera-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'triggerCamera-response)))
  "Returns full string definition for message of type 'triggerCamera-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <triggerCamera-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <triggerCamera-response>))
  "Converts a ROS message object to a list"
  (cl:list 'triggerCamera-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'triggerCamera)))
  'triggerCamera-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'triggerCamera)))
  'triggerCamera-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'triggerCamera)))
  "Returns string type for a service object of type '<triggerCamera>"
  "vrm3dvision/triggerCamera")