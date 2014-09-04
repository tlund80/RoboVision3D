; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-msg)


;//! \htmlinclude SurfaceModelParams.msg.html

(cl:defclass <SurfaceModelParams> (roslisp-msg-protocol:ros-message)
  ((rel_sampling_distance
    :reader rel_sampling_distance
    :initarg :rel_sampling_distance
    :type cl:float
    :initform 0.0)
   (model_invert_normals
    :reader model_invert_normals
    :initarg :model_invert_normals
    :type cl:fixnum
    :initform 0)
   (pose_ref_rel_sampling_distance
    :reader pose_ref_rel_sampling_distance
    :initarg :pose_ref_rel_sampling_distance
    :type cl:float
    :initform 0.0)
   (feat_step_size_rel
    :reader feat_step_size_rel
    :initarg :feat_step_size_rel
    :type cl:float
    :initform 0.0)
   (feat_angle_resolution
    :reader feat_angle_resolution
    :initarg :feat_angle_resolution
    :type cl:fixnum
    :initform 0)
   (key_point_fraction
    :reader key_point_fraction
    :initarg :key_point_fraction
    :type cl:float
    :initform 0.0)
   (min_score
    :reader min_score
    :initarg :min_score
    :type cl:float
    :initform 0.0)
   (return_result_handle
    :reader return_result_handle
    :initarg :return_result_handle
    :type cl:fixnum
    :initform 0)
   (num_matches
    :reader num_matches
    :initarg :num_matches
    :type cl:fixnum
    :initform 0)
   (max_overlap_dist_rel
    :reader max_overlap_dist_rel
    :initarg :max_overlap_dist_rel
    :type cl:float
    :initform 0.0)
   (sparse_pose_refinement
    :reader sparse_pose_refinement
    :initarg :sparse_pose_refinement
    :type cl:fixnum
    :initform 0)
   (score_type
    :reader score_type
    :initarg :score_type
    :type cl:string
    :initform "")
   (pose_ref_use_scene_normals
    :reader pose_ref_use_scene_normals
    :initarg :pose_ref_use_scene_normals
    :type cl:fixnum
    :initform 0)
   (dense_pose_refinement
    :reader dense_pose_refinement
    :initarg :dense_pose_refinement
    :type cl:fixnum
    :initform 0)
   (pose_ref_num_steps
    :reader pose_ref_num_steps
    :initarg :pose_ref_num_steps
    :type cl:fixnum
    :initform 0)
   (pose_ref_sub_sampling
    :reader pose_ref_sub_sampling
    :initarg :pose_ref_sub_sampling
    :type cl:fixnum
    :initform 0)
   (pose_ref_dist_threshold_rel
    :reader pose_ref_dist_threshold_rel
    :initarg :pose_ref_dist_threshold_rel
    :type cl:float
    :initform 0.0)
   (pose_ref_scoring_dist_rel
    :reader pose_ref_scoring_dist_rel
    :initarg :pose_ref_scoring_dist_rel
    :type cl:float
    :initform 0.0)
   (min_score_threshold
    :reader min_score_threshold
    :initarg :min_score_threshold
    :type cl:float
    :initform 0.0)
   (reset_to_default
    :reader reset_to_default
    :initarg :reset_to_default
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SurfaceModelParams (<SurfaceModelParams>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SurfaceModelParams>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SurfaceModelParams)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-msg:<SurfaceModelParams> is deprecated: use vrm3dvision-msg:SurfaceModelParams instead.")))

(cl:ensure-generic-function 'rel_sampling_distance-val :lambda-list '(m))
(cl:defmethod rel_sampling_distance-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:rel_sampling_distance-val is deprecated.  Use vrm3dvision-msg:rel_sampling_distance instead.")
  (rel_sampling_distance m))

(cl:ensure-generic-function 'model_invert_normals-val :lambda-list '(m))
(cl:defmethod model_invert_normals-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:model_invert_normals-val is deprecated.  Use vrm3dvision-msg:model_invert_normals instead.")
  (model_invert_normals m))

(cl:ensure-generic-function 'pose_ref_rel_sampling_distance-val :lambda-list '(m))
(cl:defmethod pose_ref_rel_sampling_distance-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:pose_ref_rel_sampling_distance-val is deprecated.  Use vrm3dvision-msg:pose_ref_rel_sampling_distance instead.")
  (pose_ref_rel_sampling_distance m))

(cl:ensure-generic-function 'feat_step_size_rel-val :lambda-list '(m))
(cl:defmethod feat_step_size_rel-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:feat_step_size_rel-val is deprecated.  Use vrm3dvision-msg:feat_step_size_rel instead.")
  (feat_step_size_rel m))

(cl:ensure-generic-function 'feat_angle_resolution-val :lambda-list '(m))
(cl:defmethod feat_angle_resolution-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:feat_angle_resolution-val is deprecated.  Use vrm3dvision-msg:feat_angle_resolution instead.")
  (feat_angle_resolution m))

(cl:ensure-generic-function 'key_point_fraction-val :lambda-list '(m))
(cl:defmethod key_point_fraction-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:key_point_fraction-val is deprecated.  Use vrm3dvision-msg:key_point_fraction instead.")
  (key_point_fraction m))

(cl:ensure-generic-function 'min_score-val :lambda-list '(m))
(cl:defmethod min_score-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:min_score-val is deprecated.  Use vrm3dvision-msg:min_score instead.")
  (min_score m))

(cl:ensure-generic-function 'return_result_handle-val :lambda-list '(m))
(cl:defmethod return_result_handle-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:return_result_handle-val is deprecated.  Use vrm3dvision-msg:return_result_handle instead.")
  (return_result_handle m))

(cl:ensure-generic-function 'num_matches-val :lambda-list '(m))
(cl:defmethod num_matches-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:num_matches-val is deprecated.  Use vrm3dvision-msg:num_matches instead.")
  (num_matches m))

(cl:ensure-generic-function 'max_overlap_dist_rel-val :lambda-list '(m))
(cl:defmethod max_overlap_dist_rel-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:max_overlap_dist_rel-val is deprecated.  Use vrm3dvision-msg:max_overlap_dist_rel instead.")
  (max_overlap_dist_rel m))

(cl:ensure-generic-function 'sparse_pose_refinement-val :lambda-list '(m))
(cl:defmethod sparse_pose_refinement-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:sparse_pose_refinement-val is deprecated.  Use vrm3dvision-msg:sparse_pose_refinement instead.")
  (sparse_pose_refinement m))

(cl:ensure-generic-function 'score_type-val :lambda-list '(m))
(cl:defmethod score_type-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:score_type-val is deprecated.  Use vrm3dvision-msg:score_type instead.")
  (score_type m))

(cl:ensure-generic-function 'pose_ref_use_scene_normals-val :lambda-list '(m))
(cl:defmethod pose_ref_use_scene_normals-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:pose_ref_use_scene_normals-val is deprecated.  Use vrm3dvision-msg:pose_ref_use_scene_normals instead.")
  (pose_ref_use_scene_normals m))

(cl:ensure-generic-function 'dense_pose_refinement-val :lambda-list '(m))
(cl:defmethod dense_pose_refinement-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:dense_pose_refinement-val is deprecated.  Use vrm3dvision-msg:dense_pose_refinement instead.")
  (dense_pose_refinement m))

(cl:ensure-generic-function 'pose_ref_num_steps-val :lambda-list '(m))
(cl:defmethod pose_ref_num_steps-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:pose_ref_num_steps-val is deprecated.  Use vrm3dvision-msg:pose_ref_num_steps instead.")
  (pose_ref_num_steps m))

(cl:ensure-generic-function 'pose_ref_sub_sampling-val :lambda-list '(m))
(cl:defmethod pose_ref_sub_sampling-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:pose_ref_sub_sampling-val is deprecated.  Use vrm3dvision-msg:pose_ref_sub_sampling instead.")
  (pose_ref_sub_sampling m))

(cl:ensure-generic-function 'pose_ref_dist_threshold_rel-val :lambda-list '(m))
(cl:defmethod pose_ref_dist_threshold_rel-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:pose_ref_dist_threshold_rel-val is deprecated.  Use vrm3dvision-msg:pose_ref_dist_threshold_rel instead.")
  (pose_ref_dist_threshold_rel m))

(cl:ensure-generic-function 'pose_ref_scoring_dist_rel-val :lambda-list '(m))
(cl:defmethod pose_ref_scoring_dist_rel-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:pose_ref_scoring_dist_rel-val is deprecated.  Use vrm3dvision-msg:pose_ref_scoring_dist_rel instead.")
  (pose_ref_scoring_dist_rel m))

(cl:ensure-generic-function 'min_score_threshold-val :lambda-list '(m))
(cl:defmethod min_score_threshold-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:min_score_threshold-val is deprecated.  Use vrm3dvision-msg:min_score_threshold instead.")
  (min_score_threshold m))

(cl:ensure-generic-function 'reset_to_default-val :lambda-list '(m))
(cl:defmethod reset_to_default-val ((m <SurfaceModelParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:reset_to_default-val is deprecated.  Use vrm3dvision-msg:reset_to_default instead.")
  (reset_to_default m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SurfaceModelParams>) ostream)
  "Serializes a message object of type '<SurfaceModelParams>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rel_sampling_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'model_invert_normals)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pose_ref_rel_sampling_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'feat_step_size_rel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feat_angle_resolution)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'feat_angle_resolution)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'key_point_fraction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min_score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'return_result_handle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_matches)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_overlap_dist_rel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'sparse_pose_refinement)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'score_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'score_type))
  (cl:let* ((signed (cl:slot-value msg 'pose_ref_use_scene_normals)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'dense_pose_refinement)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pose_ref_num_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pose_ref_num_steps)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pose_ref_sub_sampling)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pose_ref_sub_sampling)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pose_ref_dist_threshold_rel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pose_ref_scoring_dist_rel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min_score_threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reset_to_default) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SurfaceModelParams>) istream)
  "Deserializes a message object of type '<SurfaceModelParams>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rel_sampling_distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_invert_normals) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pose_ref_rel_sampling_distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'feat_step_size_rel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feat_angle_resolution)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'feat_angle_resolution)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'key_point_fraction) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_score) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'return_result_handle) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_matches) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_overlap_dist_rel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sparse_pose_refinement) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'score_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'score_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pose_ref_use_scene_normals) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dense_pose_refinement) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pose_ref_num_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pose_ref_num_steps)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pose_ref_sub_sampling)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pose_ref_sub_sampling)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pose_ref_dist_threshold_rel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pose_ref_scoring_dist_rel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_score_threshold) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'reset_to_default) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SurfaceModelParams>)))
  "Returns string type for a message object of type '<SurfaceModelParams>"
  "vrm3dvision/SurfaceModelParams")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SurfaceModelParams)))
  "Returns string type for a message object of type 'SurfaceModelParams"
  "vrm3dvision/SurfaceModelParams")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SurfaceModelParams>)))
  "Returns md5sum for a message object of type '<SurfaceModelParams>"
  "6b2dd2126c4d039954cc2daa6a9a324e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SurfaceModelParams)))
  "Returns md5sum for a message object of type 'SurfaceModelParams"
  "6b2dd2126c4d039954cc2daa6a9a324e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SurfaceModelParams>)))
  "Returns full string definition for message of type '<SurfaceModelParams>"
  (cl:format cl:nil "#### Common parameters~%float64 rel_sampling_distance~%~%#### Creation parameters~%int8 	model_invert_normals~%float64 pose_ref_rel_sampling_distance~%float64 feat_step_size_rel~%uint16 feat_angle_resolution~%~%#### Detection parameters~%~%## Approximate matching~%float64 key_point_fraction~%float64 min_score~%int8 	return_result_handle~%int8 	num_matches~%float64 max_overlap_dist_rel~%~%## Sparse pose refinement~%int8 	sparse_pose_refinement~%string 	score_type~%int8 	pose_ref_use_scene_normals~%~%## Dense pose refinement~%int8 	dense_pose_refinement~%uint16 	pose_ref_num_steps~%uint16 	pose_ref_sub_sampling~%float64 pose_ref_dist_threshold_rel~%float64 pose_ref_scoring_dist_rel~%~%## Evaluation~%float64 min_score_threshold~%bool 	reset_to_default~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SurfaceModelParams)))
  "Returns full string definition for message of type 'SurfaceModelParams"
  (cl:format cl:nil "#### Common parameters~%float64 rel_sampling_distance~%~%#### Creation parameters~%int8 	model_invert_normals~%float64 pose_ref_rel_sampling_distance~%float64 feat_step_size_rel~%uint16 feat_angle_resolution~%~%#### Detection parameters~%~%## Approximate matching~%float64 key_point_fraction~%float64 min_score~%int8 	return_result_handle~%int8 	num_matches~%float64 max_overlap_dist_rel~%~%## Sparse pose refinement~%int8 	sparse_pose_refinement~%string 	score_type~%int8 	pose_ref_use_scene_normals~%~%## Dense pose refinement~%int8 	dense_pose_refinement~%uint16 	pose_ref_num_steps~%uint16 	pose_ref_sub_sampling~%float64 pose_ref_dist_threshold_rel~%float64 pose_ref_scoring_dist_rel~%~%## Evaluation~%float64 min_score_threshold~%bool 	reset_to_default~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SurfaceModelParams>))
  (cl:+ 0
     8
     1
     8
     8
     2
     8
     8
     1
     1
     8
     1
     4 (cl:length (cl:slot-value msg 'score_type))
     1
     1
     2
     2
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SurfaceModelParams>))
  "Converts a ROS message object to a list"
  (cl:list 'SurfaceModelParams
    (cl:cons ':rel_sampling_distance (rel_sampling_distance msg))
    (cl:cons ':model_invert_normals (model_invert_normals msg))
    (cl:cons ':pose_ref_rel_sampling_distance (pose_ref_rel_sampling_distance msg))
    (cl:cons ':feat_step_size_rel (feat_step_size_rel msg))
    (cl:cons ':feat_angle_resolution (feat_angle_resolution msg))
    (cl:cons ':key_point_fraction (key_point_fraction msg))
    (cl:cons ':min_score (min_score msg))
    (cl:cons ':return_result_handle (return_result_handle msg))
    (cl:cons ':num_matches (num_matches msg))
    (cl:cons ':max_overlap_dist_rel (max_overlap_dist_rel msg))
    (cl:cons ':sparse_pose_refinement (sparse_pose_refinement msg))
    (cl:cons ':score_type (score_type msg))
    (cl:cons ':pose_ref_use_scene_normals (pose_ref_use_scene_normals msg))
    (cl:cons ':dense_pose_refinement (dense_pose_refinement msg))
    (cl:cons ':pose_ref_num_steps (pose_ref_num_steps msg))
    (cl:cons ':pose_ref_sub_sampling (pose_ref_sub_sampling msg))
    (cl:cons ':pose_ref_dist_threshold_rel (pose_ref_dist_threshold_rel msg))
    (cl:cons ':pose_ref_scoring_dist_rel (pose_ref_scoring_dist_rel msg))
    (cl:cons ':min_score_threshold (min_score_threshold msg))
    (cl:cons ':reset_to_default (reset_to_default msg))
))
