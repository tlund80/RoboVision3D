; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude acquireSequenceOnRobot-request.msg.html

(cl:defclass <acquireSequenceOnRobot-request> (roslisp-msg-protocol:ros-message)
  ((model
    :reader model
    :initarg :model
    :type cl:string
    :initform "")
   (method
    :reader method
    :initarg :method
    :type cl:string
    :initform "")
   (exposure
    :reader exposure
    :initarg :exposure
    :type cl:string
    :initform "")
   (save_path
    :reader save_path
    :initarg :save_path
    :type cl:string
    :initform ""))
)

(cl:defclass acquireSequenceOnRobot-request (<acquireSequenceOnRobot-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <acquireSequenceOnRobot-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'acquireSequenceOnRobot-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<acquireSequenceOnRobot-request> is deprecated: use vrm3dvision-srv:acquireSequenceOnRobot-request instead.")))

(cl:ensure-generic-function 'model-val :lambda-list '(m))
(cl:defmethod model-val ((m <acquireSequenceOnRobot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:model-val is deprecated.  Use vrm3dvision-srv:model instead.")
  (model m))

(cl:ensure-generic-function 'method-val :lambda-list '(m))
(cl:defmethod method-val ((m <acquireSequenceOnRobot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:method-val is deprecated.  Use vrm3dvision-srv:method instead.")
  (method m))

(cl:ensure-generic-function 'exposure-val :lambda-list '(m))
(cl:defmethod exposure-val ((m <acquireSequenceOnRobot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:exposure-val is deprecated.  Use vrm3dvision-srv:exposure instead.")
  (exposure m))

(cl:ensure-generic-function 'save_path-val :lambda-list '(m))
(cl:defmethod save_path-val ((m <acquireSequenceOnRobot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:save_path-val is deprecated.  Use vrm3dvision-srv:save_path instead.")
  (save_path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <acquireSequenceOnRobot-request>) ostream)
  "Serializes a message object of type '<acquireSequenceOnRobot-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'method))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'method))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'exposure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'exposure))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'save_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'save_path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <acquireSequenceOnRobot-request>) istream)
  "Deserializes a message object of type '<acquireSequenceOnRobot-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'method) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'method) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'exposure) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'exposure) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'save_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'save_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<acquireSequenceOnRobot-request>)))
  "Returns string type for a service object of type '<acquireSequenceOnRobot-request>"
  "vrm3dvision/acquireSequenceOnRobotRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'acquireSequenceOnRobot-request)))
  "Returns string type for a service object of type 'acquireSequenceOnRobot-request"
  "vrm3dvision/acquireSequenceOnRobotRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<acquireSequenceOnRobot-request>)))
  "Returns md5sum for a message object of type '<acquireSequenceOnRobot-request>"
  "b9667e0169bca17c2158a3394d7309f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'acquireSequenceOnRobot-request)))
  "Returns md5sum for a message object of type 'acquireSequenceOnRobot-request"
  "b9667e0169bca17c2158a3394d7309f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<acquireSequenceOnRobot-request>)))
  "Returns full string definition for message of type '<acquireSequenceOnRobot-request>"
  (cl:format cl:nil "string model~%string method~%string exposure~%string save_path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'acquireSequenceOnRobot-request)))
  "Returns full string definition for message of type 'acquireSequenceOnRobot-request"
  (cl:format cl:nil "string model~%string method~%string exposure~%string save_path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <acquireSequenceOnRobot-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model))
     4 (cl:length (cl:slot-value msg 'method))
     4 (cl:length (cl:slot-value msg 'exposure))
     4 (cl:length (cl:slot-value msg 'save_path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <acquireSequenceOnRobot-request>))
  "Converts a ROS message object to a list"
  (cl:list 'acquireSequenceOnRobot-request
    (cl:cons ':model (model msg))
    (cl:cons ':method (method msg))
    (cl:cons ':exposure (exposure msg))
    (cl:cons ':save_path (save_path msg))
))
;//! \htmlinclude acquireSequenceOnRobot-response.msg.html

(cl:defclass <acquireSequenceOnRobot-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass acquireSequenceOnRobot-response (<acquireSequenceOnRobot-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <acquireSequenceOnRobot-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'acquireSequenceOnRobot-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<acquireSequenceOnRobot-response> is deprecated: use vrm3dvision-srv:acquireSequenceOnRobot-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <acquireSequenceOnRobot-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <acquireSequenceOnRobot-response>) ostream)
  "Serializes a message object of type '<acquireSequenceOnRobot-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <acquireSequenceOnRobot-response>) istream)
  "Deserializes a message object of type '<acquireSequenceOnRobot-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<acquireSequenceOnRobot-response>)))
  "Returns string type for a service object of type '<acquireSequenceOnRobot-response>"
  "vrm3dvision/acquireSequenceOnRobotResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'acquireSequenceOnRobot-response)))
  "Returns string type for a service object of type 'acquireSequenceOnRobot-response"
  "vrm3dvision/acquireSequenceOnRobotResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<acquireSequenceOnRobot-response>)))
  "Returns md5sum for a message object of type '<acquireSequenceOnRobot-response>"
  "b9667e0169bca17c2158a3394d7309f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'acquireSequenceOnRobot-response)))
  "Returns md5sum for a message object of type 'acquireSequenceOnRobot-response"
  "b9667e0169bca17c2158a3394d7309f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<acquireSequenceOnRobot-response>)))
  "Returns full string definition for message of type '<acquireSequenceOnRobot-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'acquireSequenceOnRobot-response)))
  "Returns full string definition for message of type 'acquireSequenceOnRobot-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <acquireSequenceOnRobot-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <acquireSequenceOnRobot-response>))
  "Converts a ROS message object to a list"
  (cl:list 'acquireSequenceOnRobot-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'acquireSequenceOnRobot)))
  'acquireSequenceOnRobot-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'acquireSequenceOnRobot)))
  'acquireSequenceOnRobot-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'acquireSequenceOnRobot)))
  "Returns string type for a service object of type '<acquireSequenceOnRobot>"
  "vrm3dvision/acquireSequenceOnRobot")