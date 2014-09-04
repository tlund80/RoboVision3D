; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude computeBestExposure-request.msg.html

(cl:defclass <computeBestExposure-request> (roslisp-msg-protocol:ros-message)
  ((model_name
    :reader model_name
    :initarg :model_name
    :type cl:string
    :initform "")
   (exposures
    :reader exposures
    :initarg :exposures
    :type cl:string
    :initform ""))
)

(cl:defclass computeBestExposure-request (<computeBestExposure-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <computeBestExposure-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'computeBestExposure-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<computeBestExposure-request> is deprecated: use vrm3dvision-srv:computeBestExposure-request instead.")))

(cl:ensure-generic-function 'model_name-val :lambda-list '(m))
(cl:defmethod model_name-val ((m <computeBestExposure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:model_name-val is deprecated.  Use vrm3dvision-srv:model_name instead.")
  (model_name m))

(cl:ensure-generic-function 'exposures-val :lambda-list '(m))
(cl:defmethod exposures-val ((m <computeBestExposure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:exposures-val is deprecated.  Use vrm3dvision-srv:exposures instead.")
  (exposures m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <computeBestExposure-request>) ostream)
  "Serializes a message object of type '<computeBestExposure-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'exposures))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'exposures))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <computeBestExposure-request>) istream)
  "Deserializes a message object of type '<computeBestExposure-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'exposures) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'exposures) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<computeBestExposure-request>)))
  "Returns string type for a service object of type '<computeBestExposure-request>"
  "vrm3dvision/computeBestExposureRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'computeBestExposure-request)))
  "Returns string type for a service object of type 'computeBestExposure-request"
  "vrm3dvision/computeBestExposureRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<computeBestExposure-request>)))
  "Returns md5sum for a message object of type '<computeBestExposure-request>"
  "2dda0f20eb5007c51638f441a7489240")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'computeBestExposure-request)))
  "Returns md5sum for a message object of type 'computeBestExposure-request"
  "2dda0f20eb5007c51638f441a7489240")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<computeBestExposure-request>)))
  "Returns full string definition for message of type '<computeBestExposure-request>"
  (cl:format cl:nil "string model_name~%string exposures~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'computeBestExposure-request)))
  "Returns full string definition for message of type 'computeBestExposure-request"
  (cl:format cl:nil "string model_name~%string exposures~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <computeBestExposure-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model_name))
     4 (cl:length (cl:slot-value msg 'exposures))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <computeBestExposure-request>))
  "Converts a ROS message object to a list"
  (cl:list 'computeBestExposure-request
    (cl:cons ':model_name (model_name msg))
    (cl:cons ':exposures (exposures msg))
))
;//! \htmlinclude computeBestExposure-response.msg.html

(cl:defclass <computeBestExposure-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass computeBestExposure-response (<computeBestExposure-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <computeBestExposure-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'computeBestExposure-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<computeBestExposure-response> is deprecated: use vrm3dvision-srv:computeBestExposure-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <computeBestExposure-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <computeBestExposure-response>) ostream)
  "Serializes a message object of type '<computeBestExposure-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <computeBestExposure-response>) istream)
  "Deserializes a message object of type '<computeBestExposure-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<computeBestExposure-response>)))
  "Returns string type for a service object of type '<computeBestExposure-response>"
  "vrm3dvision/computeBestExposureResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'computeBestExposure-response)))
  "Returns string type for a service object of type 'computeBestExposure-response"
  "vrm3dvision/computeBestExposureResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<computeBestExposure-response>)))
  "Returns md5sum for a message object of type '<computeBestExposure-response>"
  "2dda0f20eb5007c51638f441a7489240")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'computeBestExposure-response)))
  "Returns md5sum for a message object of type 'computeBestExposure-response"
  "2dda0f20eb5007c51638f441a7489240")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<computeBestExposure-response>)))
  "Returns full string definition for message of type '<computeBestExposure-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'computeBestExposure-response)))
  "Returns full string definition for message of type 'computeBestExposure-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <computeBestExposure-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <computeBestExposure-response>))
  "Converts a ROS message object to a list"
  (cl:list 'computeBestExposure-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'computeBestExposure)))
  'computeBestExposure-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'computeBestExposure)))
  'computeBestExposure-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'computeBestExposure)))
  "Returns string type for a service object of type '<computeBestExposure>"
  "vrm3dvision/computeBestExposure")