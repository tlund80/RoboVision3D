; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude acquirePoseEstimate-request.msg.html

(cl:defclass <acquirePoseEstimate-request> (roslisp-msg-protocol:ros-message)
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
   (manual_trigger
    :reader manual_trigger
    :initarg :manual_trigger
    :type cl:boolean
    :initform cl:nil)
   (smp
    :reader smp
    :initarg :smp
    :type vrm3dvision-msg:SurfaceModelParams
    :initform (cl:make-instance 'vrm3dvision-msg:SurfaceModelParams))
   (app
    :reader app
    :initarg :app
    :type vrm3dvision-msg:AlignmentPrerejectiveParams
    :initform (cl:make-instance 'vrm3dvision-msg:AlignmentPrerejectiveParams)))
)

(cl:defclass acquirePoseEstimate-request (<acquirePoseEstimate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <acquirePoseEstimate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'acquirePoseEstimate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<acquirePoseEstimate-request> is deprecated: use vrm3dvision-srv:acquirePoseEstimate-request instead.")))

(cl:ensure-generic-function 'model-val :lambda-list '(m))
(cl:defmethod model-val ((m <acquirePoseEstimate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:model-val is deprecated.  Use vrm3dvision-srv:model instead.")
  (model m))

(cl:ensure-generic-function 'method-val :lambda-list '(m))
(cl:defmethod method-val ((m <acquirePoseEstimate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:method-val is deprecated.  Use vrm3dvision-srv:method instead.")
  (method m))

(cl:ensure-generic-function 'exposure-val :lambda-list '(m))
(cl:defmethod exposure-val ((m <acquirePoseEstimate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:exposure-val is deprecated.  Use vrm3dvision-srv:exposure instead.")
  (exposure m))

(cl:ensure-generic-function 'manual_trigger-val :lambda-list '(m))
(cl:defmethod manual_trigger-val ((m <acquirePoseEstimate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:manual_trigger-val is deprecated.  Use vrm3dvision-srv:manual_trigger instead.")
  (manual_trigger m))

(cl:ensure-generic-function 'smp-val :lambda-list '(m))
(cl:defmethod smp-val ((m <acquirePoseEstimate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:smp-val is deprecated.  Use vrm3dvision-srv:smp instead.")
  (smp m))

(cl:ensure-generic-function 'app-val :lambda-list '(m))
(cl:defmethod app-val ((m <acquirePoseEstimate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:app-val is deprecated.  Use vrm3dvision-srv:app instead.")
  (app m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <acquirePoseEstimate-request>) ostream)
  "Serializes a message object of type '<acquirePoseEstimate-request>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'manual_trigger) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'smp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'app) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <acquirePoseEstimate-request>) istream)
  "Deserializes a message object of type '<acquirePoseEstimate-request>"
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
    (cl:setf (cl:slot-value msg 'manual_trigger) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'smp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'app) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<acquirePoseEstimate-request>)))
  "Returns string type for a service object of type '<acquirePoseEstimate-request>"
  "vrm3dvision/acquirePoseEstimateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'acquirePoseEstimate-request)))
  "Returns string type for a service object of type 'acquirePoseEstimate-request"
  "vrm3dvision/acquirePoseEstimateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<acquirePoseEstimate-request>)))
  "Returns md5sum for a message object of type '<acquirePoseEstimate-request>"
  "cf391b127fb3c1a20668508da59fe3f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'acquirePoseEstimate-request)))
  "Returns md5sum for a message object of type 'acquirePoseEstimate-request"
  "cf391b127fb3c1a20668508da59fe3f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<acquirePoseEstimate-request>)))
  "Returns full string definition for message of type '<acquirePoseEstimate-request>"
  (cl:format cl:nil "string model~%string method~%string exposure~%bool manual_trigger~%SurfaceModelParams smp~%AlignmentPrerejectiveParams app~%~%================================================================================~%MSG: vrm3dvision/SurfaceModelParams~%#### Common parameters~%float64 rel_sampling_distance~%~%#### Creation parameters~%int8 	model_invert_normals~%float64 pose_ref_rel_sampling_distance~%float64 feat_step_size_rel~%uint16 feat_angle_resolution~%~%#### Detection parameters~%~%## Approximate matching~%float64 key_point_fraction~%float64 min_score~%int8 	return_result_handle~%int8 	num_matches~%float64 max_overlap_dist_rel~%~%## Sparse pose refinement~%int8 	sparse_pose_refinement~%string 	score_type~%int8 	pose_ref_use_scene_normals~%~%## Dense pose refinement~%int8 	dense_pose_refinement~%uint16 	pose_ref_num_steps~%uint16 	pose_ref_sub_sampling~%float64 pose_ref_dist_threshold_rel~%float64 pose_ref_scoring_dist_rel~%~%## Evaluation~%float64 min_score_threshold~%bool 	reset_to_default~%~%~%~%================================================================================~%MSG: vrm3dvision/AlignmentPrerejectiveParams~%#### Common parameters~%float64 leaf_size~%float64 normal_radius_ratio_leaf~%float64 feature_radius_ratio_leaf~%uint32 	correspondence_randomness~%float64 similarity_threshold~%uint32 	max_iterations~%float64 max_correspondence_distance_ratio_leaf~%float64 inlier_fraction~%~%## ICP~%uint32 	icp_max_iterations~%float64 icp_max_correspondence_distance_ratio_leaf~%~%# Alignment Prerejective~%float64 min_score~%float64 min_score_threshold~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'acquirePoseEstimate-request)))
  "Returns full string definition for message of type 'acquirePoseEstimate-request"
  (cl:format cl:nil "string model~%string method~%string exposure~%bool manual_trigger~%SurfaceModelParams smp~%AlignmentPrerejectiveParams app~%~%================================================================================~%MSG: vrm3dvision/SurfaceModelParams~%#### Common parameters~%float64 rel_sampling_distance~%~%#### Creation parameters~%int8 	model_invert_normals~%float64 pose_ref_rel_sampling_distance~%float64 feat_step_size_rel~%uint16 feat_angle_resolution~%~%#### Detection parameters~%~%## Approximate matching~%float64 key_point_fraction~%float64 min_score~%int8 	return_result_handle~%int8 	num_matches~%float64 max_overlap_dist_rel~%~%## Sparse pose refinement~%int8 	sparse_pose_refinement~%string 	score_type~%int8 	pose_ref_use_scene_normals~%~%## Dense pose refinement~%int8 	dense_pose_refinement~%uint16 	pose_ref_num_steps~%uint16 	pose_ref_sub_sampling~%float64 pose_ref_dist_threshold_rel~%float64 pose_ref_scoring_dist_rel~%~%## Evaluation~%float64 min_score_threshold~%bool 	reset_to_default~%~%~%~%================================================================================~%MSG: vrm3dvision/AlignmentPrerejectiveParams~%#### Common parameters~%float64 leaf_size~%float64 normal_radius_ratio_leaf~%float64 feature_radius_ratio_leaf~%uint32 	correspondence_randomness~%float64 similarity_threshold~%uint32 	max_iterations~%float64 max_correspondence_distance_ratio_leaf~%float64 inlier_fraction~%~%## ICP~%uint32 	icp_max_iterations~%float64 icp_max_correspondence_distance_ratio_leaf~%~%# Alignment Prerejective~%float64 min_score~%float64 min_score_threshold~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <acquirePoseEstimate-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model))
     4 (cl:length (cl:slot-value msg 'method))
     4 (cl:length (cl:slot-value msg 'exposure))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'smp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'app))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <acquirePoseEstimate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'acquirePoseEstimate-request
    (cl:cons ':model (model msg))
    (cl:cons ':method (method msg))
    (cl:cons ':exposure (exposure msg))
    (cl:cons ':manual_trigger (manual_trigger msg))
    (cl:cons ':smp (smp msg))
    (cl:cons ':app (app msg))
))
;//! \htmlinclude acquirePoseEstimate-response.msg.html

(cl:defclass <acquirePoseEstimate-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass acquirePoseEstimate-response (<acquirePoseEstimate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <acquirePoseEstimate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'acquirePoseEstimate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<acquirePoseEstimate-response> is deprecated: use vrm3dvision-srv:acquirePoseEstimate-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <acquirePoseEstimate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <acquirePoseEstimate-response>) ostream)
  "Serializes a message object of type '<acquirePoseEstimate-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <acquirePoseEstimate-response>) istream)
  "Deserializes a message object of type '<acquirePoseEstimate-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<acquirePoseEstimate-response>)))
  "Returns string type for a service object of type '<acquirePoseEstimate-response>"
  "vrm3dvision/acquirePoseEstimateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'acquirePoseEstimate-response)))
  "Returns string type for a service object of type 'acquirePoseEstimate-response"
  "vrm3dvision/acquirePoseEstimateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<acquirePoseEstimate-response>)))
  "Returns md5sum for a message object of type '<acquirePoseEstimate-response>"
  "cf391b127fb3c1a20668508da59fe3f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'acquirePoseEstimate-response)))
  "Returns md5sum for a message object of type 'acquirePoseEstimate-response"
  "cf391b127fb3c1a20668508da59fe3f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<acquirePoseEstimate-response>)))
  "Returns full string definition for message of type '<acquirePoseEstimate-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'acquirePoseEstimate-response)))
  "Returns full string definition for message of type 'acquirePoseEstimate-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <acquirePoseEstimate-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <acquirePoseEstimate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'acquirePoseEstimate-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'acquirePoseEstimate)))
  'acquirePoseEstimate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'acquirePoseEstimate)))
  'acquirePoseEstimate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'acquirePoseEstimate)))
  "Returns string type for a service object of type '<acquirePoseEstimate>"
  "vrm3dvision/acquirePoseEstimate")