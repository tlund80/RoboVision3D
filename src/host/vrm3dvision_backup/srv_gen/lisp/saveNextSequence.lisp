; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude saveNextSequence-request.msg.html

(cl:defclass <saveNextSequence-request> (roslisp-msg-protocol:ros-message)
  ((folder_name
    :reader folder_name
    :initarg :folder_name
    :type cl:string
    :initform ""))
)

(cl:defclass saveNextSequence-request (<saveNextSequence-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <saveNextSequence-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'saveNextSequence-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<saveNextSequence-request> is deprecated: use vrm3dvision-srv:saveNextSequence-request instead.")))

(cl:ensure-generic-function 'folder_name-val :lambda-list '(m))
(cl:defmethod folder_name-val ((m <saveNextSequence-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:folder_name-val is deprecated.  Use vrm3dvision-srv:folder_name instead.")
  (folder_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <saveNextSequence-request>) ostream)
  "Serializes a message object of type '<saveNextSequence-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'folder_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'folder_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <saveNextSequence-request>) istream)
  "Deserializes a message object of type '<saveNextSequence-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'folder_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'folder_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<saveNextSequence-request>)))
  "Returns string type for a service object of type '<saveNextSequence-request>"
  "vrm3dvision/saveNextSequenceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveNextSequence-request)))
  "Returns string type for a service object of type 'saveNextSequence-request"
  "vrm3dvision/saveNextSequenceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<saveNextSequence-request>)))
  "Returns md5sum for a message object of type '<saveNextSequence-request>"
  "c9fdd0b297419b23b31a4fee9a64a8ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'saveNextSequence-request)))
  "Returns md5sum for a message object of type 'saveNextSequence-request"
  "c9fdd0b297419b23b31a4fee9a64a8ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<saveNextSequence-request>)))
  "Returns full string definition for message of type '<saveNextSequence-request>"
  (cl:format cl:nil "string folder_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'saveNextSequence-request)))
  "Returns full string definition for message of type 'saveNextSequence-request"
  (cl:format cl:nil "string folder_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <saveNextSequence-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'folder_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <saveNextSequence-request>))
  "Converts a ROS message object to a list"
  (cl:list 'saveNextSequence-request
    (cl:cons ':folder_name (folder_name msg))
))
;//! \htmlinclude saveNextSequence-response.msg.html

(cl:defclass <saveNextSequence-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass saveNextSequence-response (<saveNextSequence-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <saveNextSequence-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'saveNextSequence-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<saveNextSequence-response> is deprecated: use vrm3dvision-srv:saveNextSequence-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <saveNextSequence-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <saveNextSequence-response>) ostream)
  "Serializes a message object of type '<saveNextSequence-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <saveNextSequence-response>) istream)
  "Deserializes a message object of type '<saveNextSequence-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<saveNextSequence-response>)))
  "Returns string type for a service object of type '<saveNextSequence-response>"
  "vrm3dvision/saveNextSequenceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveNextSequence-response)))
  "Returns string type for a service object of type 'saveNextSequence-response"
  "vrm3dvision/saveNextSequenceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<saveNextSequence-response>)))
  "Returns md5sum for a message object of type '<saveNextSequence-response>"
  "c9fdd0b297419b23b31a4fee9a64a8ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'saveNextSequence-response)))
  "Returns md5sum for a message object of type 'saveNextSequence-response"
  "c9fdd0b297419b23b31a4fee9a64a8ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<saveNextSequence-response>)))
  "Returns full string definition for message of type '<saveNextSequence-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'saveNextSequence-response)))
  "Returns full string definition for message of type 'saveNextSequence-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <saveNextSequence-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <saveNextSequence-response>))
  "Converts a ROS message object to a list"
  (cl:list 'saveNextSequence-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'saveNextSequence)))
  'saveNextSequence-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'saveNextSequence)))
  'saveNextSequence-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveNextSequence)))
  "Returns string type for a service object of type '<saveNextSequence>"
  "vrm3dvision/saveNextSequence")