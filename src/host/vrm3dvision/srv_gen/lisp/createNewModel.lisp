; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-srv)


;//! \htmlinclude createNewModel-request.msg.html

(cl:defclass <createNewModel-request> (roslisp-msg-protocol:ros-message)
  ((model_name
    :reader model_name
    :initarg :model_name
    :type cl:string
    :initform "")
   (cad_path
    :reader cad_path
    :initarg :cad_path
    :type cl:string
    :initform "")
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

(cl:defclass createNewModel-request (<createNewModel-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <createNewModel-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'createNewModel-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<createNewModel-request> is deprecated: use vrm3dvision-srv:createNewModel-request instead.")))

(cl:ensure-generic-function 'model_name-val :lambda-list '(m))
(cl:defmethod model_name-val ((m <createNewModel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:model_name-val is deprecated.  Use vrm3dvision-srv:model_name instead.")
  (model_name m))

(cl:ensure-generic-function 'cad_path-val :lambda-list '(m))
(cl:defmethod cad_path-val ((m <createNewModel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:cad_path-val is deprecated.  Use vrm3dvision-srv:cad_path instead.")
  (cad_path m))

(cl:ensure-generic-function 'smp-val :lambda-list '(m))
(cl:defmethod smp-val ((m <createNewModel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:smp-val is deprecated.  Use vrm3dvision-srv:smp instead.")
  (smp m))

(cl:ensure-generic-function 'app-val :lambda-list '(m))
(cl:defmethod app-val ((m <createNewModel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:app-val is deprecated.  Use vrm3dvision-srv:app instead.")
  (app m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <createNewModel-request>) ostream)
  "Serializes a message object of type '<createNewModel-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cad_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cad_path))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'smp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'app) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <createNewModel-request>) istream)
  "Deserializes a message object of type '<createNewModel-request>"
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
      (cl:setf (cl:slot-value msg 'cad_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cad_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'smp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'app) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<createNewModel-request>)))
  "Returns string type for a service object of type '<createNewModel-request>"
  "vrm3dvision/createNewModelRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'createNewModel-request)))
  "Returns string type for a service object of type 'createNewModel-request"
  "vrm3dvision/createNewModelRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<createNewModel-request>)))
  "Returns md5sum for a message object of type '<createNewModel-request>"
  "5b06b70d26b98c2e4972576e2b4a1d1a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'createNewModel-request)))
  "Returns md5sum for a message object of type 'createNewModel-request"
  "5b06b70d26b98c2e4972576e2b4a1d1a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<createNewModel-request>)))
  "Returns full string definition for message of type '<createNewModel-request>"
  (cl:format cl:nil "string model_name~%string cad_path~%SurfaceModelParams smp~%AlignmentPrerejectiveParams app~%~%================================================================================~%MSG: vrm3dvision/SurfaceModelParams~%#### Common parameters~%float64 rel_sampling_distance~%~%#### Creation parameters~%int8 	model_invert_normals~%float64 pose_ref_rel_sampling_distance~%float64 feat_step_size_rel~%uint16 feat_angle_resolution~%~%#### Detection parameters~%~%## Approximate matching~%float64 key_point_fraction~%float64 min_score~%int8 	return_result_handle~%int8 	num_matches~%float64 max_overlap_dist_rel~%~%## Sparse pose refinement~%int8 	sparse_pose_refinement~%string 	score_type~%int8 	pose_ref_use_scene_normals~%~%## Dense pose refinement~%int8 	dense_pose_refinement~%uint16 	pose_ref_num_steps~%uint16 	pose_ref_sub_sampling~%float64 pose_ref_dist_threshold_rel~%float64 pose_ref_scoring_dist_rel~%~%## Evaluation~%float64 min_score_threshold~%bool 	reset_to_default~%~%~%~%================================================================================~%MSG: vrm3dvision/AlignmentPrerejectiveParams~%#### Common parameters~%float64 leaf_size~%float64 normal_radius_ratio_leaf~%float64 feature_radius_ratio_leaf~%uint32 	correspondence_randomness~%float64 similarity_threshold~%uint32 	max_iterations~%float64 max_correspondence_distance_ratio_leaf~%float64 inlier_fraction~%~%## ICP~%uint32 	icp_max_iterations~%float64 icp_max_correspondence_distance_ratio_leaf~%~%# Alignment Prerejective~%float64 min_score~%float64 min_score_threshold~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'createNewModel-request)))
  "Returns full string definition for message of type 'createNewModel-request"
  (cl:format cl:nil "string model_name~%string cad_path~%SurfaceModelParams smp~%AlignmentPrerejectiveParams app~%~%================================================================================~%MSG: vrm3dvision/SurfaceModelParams~%#### Common parameters~%float64 rel_sampling_distance~%~%#### Creation parameters~%int8 	model_invert_normals~%float64 pose_ref_rel_sampling_distance~%float64 feat_step_size_rel~%uint16 feat_angle_resolution~%~%#### Detection parameters~%~%## Approximate matching~%float64 key_point_fraction~%float64 min_score~%int8 	return_result_handle~%int8 	num_matches~%float64 max_overlap_dist_rel~%~%## Sparse pose refinement~%int8 	sparse_pose_refinement~%string 	score_type~%int8 	pose_ref_use_scene_normals~%~%## Dense pose refinement~%int8 	dense_pose_refinement~%uint16 	pose_ref_num_steps~%uint16 	pose_ref_sub_sampling~%float64 pose_ref_dist_threshold_rel~%float64 pose_ref_scoring_dist_rel~%~%## Evaluation~%float64 min_score_threshold~%bool 	reset_to_default~%~%~%~%================================================================================~%MSG: vrm3dvision/AlignmentPrerejectiveParams~%#### Common parameters~%float64 leaf_size~%float64 normal_radius_ratio_leaf~%float64 feature_radius_ratio_leaf~%uint32 	correspondence_randomness~%float64 similarity_threshold~%uint32 	max_iterations~%float64 max_correspondence_distance_ratio_leaf~%float64 inlier_fraction~%~%## ICP~%uint32 	icp_max_iterations~%float64 icp_max_correspondence_distance_ratio_leaf~%~%# Alignment Prerejective~%float64 min_score~%float64 min_score_threshold~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <createNewModel-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model_name))
     4 (cl:length (cl:slot-value msg 'cad_path))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'smp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'app))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <createNewModel-request>))
  "Converts a ROS message object to a list"
  (cl:list 'createNewModel-request
    (cl:cons ':model_name (model_name msg))
    (cl:cons ':cad_path (cad_path msg))
    (cl:cons ':smp (smp msg))
    (cl:cons ':app (app msg))
))
;//! \htmlinclude createNewModel-response.msg.html

(cl:defclass <createNewModel-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass createNewModel-response (<createNewModel-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <createNewModel-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'createNewModel-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-srv:<createNewModel-response> is deprecated: use vrm3dvision-srv:createNewModel-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <createNewModel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-srv:success-val is deprecated.  Use vrm3dvision-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <createNewModel-response>) ostream)
  "Serializes a message object of type '<createNewModel-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <createNewModel-response>) istream)
  "Deserializes a message object of type '<createNewModel-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<createNewModel-response>)))
  "Returns string type for a service object of type '<createNewModel-response>"
  "vrm3dvision/createNewModelResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'createNewModel-response)))
  "Returns string type for a service object of type 'createNewModel-response"
  "vrm3dvision/createNewModelResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<createNewModel-response>)))
  "Returns md5sum for a message object of type '<createNewModel-response>"
  "5b06b70d26b98c2e4972576e2b4a1d1a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'createNewModel-response)))
  "Returns md5sum for a message object of type 'createNewModel-response"
  "5b06b70d26b98c2e4972576e2b4a1d1a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<createNewModel-response>)))
  "Returns full string definition for message of type '<createNewModel-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'createNewModel-response)))
  "Returns full string definition for message of type 'createNewModel-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <createNewModel-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <createNewModel-response>))
  "Converts a ROS message object to a list"
  (cl:list 'createNewModel-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'createNewModel)))
  'createNewModel-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'createNewModel)))
  'createNewModel-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'createNewModel)))
  "Returns string type for a service object of type '<createNewModel>"
  "vrm3dvision/createNewModel")