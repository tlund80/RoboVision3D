; Auto-generated. Do not edit!


(cl:in-package vrm3dvision-msg)


;//! \htmlinclude AlignmentPrerejectiveParams.msg.html

(cl:defclass <AlignmentPrerejectiveParams> (roslisp-msg-protocol:ros-message)
  ((leaf_size
    :reader leaf_size
    :initarg :leaf_size
    :type cl:float
    :initform 0.0)
   (normal_radius_ratio_leaf
    :reader normal_radius_ratio_leaf
    :initarg :normal_radius_ratio_leaf
    :type cl:float
    :initform 0.0)
   (feature_radius_ratio_leaf
    :reader feature_radius_ratio_leaf
    :initarg :feature_radius_ratio_leaf
    :type cl:float
    :initform 0.0)
   (correspondence_randomness
    :reader correspondence_randomness
    :initarg :correspondence_randomness
    :type cl:integer
    :initform 0)
   (similarity_threshold
    :reader similarity_threshold
    :initarg :similarity_threshold
    :type cl:float
    :initform 0.0)
   (max_iterations
    :reader max_iterations
    :initarg :max_iterations
    :type cl:integer
    :initform 0)
   (max_correspondence_distance_ratio_leaf
    :reader max_correspondence_distance_ratio_leaf
    :initarg :max_correspondence_distance_ratio_leaf
    :type cl:float
    :initform 0.0)
   (inlier_fraction
    :reader inlier_fraction
    :initarg :inlier_fraction
    :type cl:float
    :initform 0.0)
   (icp_max_iterations
    :reader icp_max_iterations
    :initarg :icp_max_iterations
    :type cl:integer
    :initform 0)
   (icp_max_correspondence_distance_ratio_leaf
    :reader icp_max_correspondence_distance_ratio_leaf
    :initarg :icp_max_correspondence_distance_ratio_leaf
    :type cl:float
    :initform 0.0)
   (min_score
    :reader min_score
    :initarg :min_score
    :type cl:float
    :initform 0.0)
   (min_score_threshold
    :reader min_score_threshold
    :initarg :min_score_threshold
    :type cl:float
    :initform 0.0))
)

(cl:defclass AlignmentPrerejectiveParams (<AlignmentPrerejectiveParams>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AlignmentPrerejectiveParams>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AlignmentPrerejectiveParams)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrm3dvision-msg:<AlignmentPrerejectiveParams> is deprecated: use vrm3dvision-msg:AlignmentPrerejectiveParams instead.")))

(cl:ensure-generic-function 'leaf_size-val :lambda-list '(m))
(cl:defmethod leaf_size-val ((m <AlignmentPrerejectiveParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:leaf_size-val is deprecated.  Use vrm3dvision-msg:leaf_size instead.")
  (leaf_size m))

(cl:ensure-generic-function 'normal_radius_ratio_leaf-val :lambda-list '(m))
(cl:defmethod normal_radius_ratio_leaf-val ((m <AlignmentPrerejectiveParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:normal_radius_ratio_leaf-val is deprecated.  Use vrm3dvision-msg:normal_radius_ratio_leaf instead.")
  (normal_radius_ratio_leaf m))

(cl:ensure-generic-function 'feature_radius_ratio_leaf-val :lambda-list '(m))
(cl:defmethod feature_radius_ratio_leaf-val ((m <AlignmentPrerejectiveParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:feature_radius_ratio_leaf-val is deprecated.  Use vrm3dvision-msg:feature_radius_ratio_leaf instead.")
  (feature_radius_ratio_leaf m))

(cl:ensure-generic-function 'correspondence_randomness-val :lambda-list '(m))
(cl:defmethod correspondence_randomness-val ((m <AlignmentPrerejectiveParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:correspondence_randomness-val is deprecated.  Use vrm3dvision-msg:correspondence_randomness instead.")
  (correspondence_randomness m))

(cl:ensure-generic-function 'similarity_threshold-val :lambda-list '(m))
(cl:defmethod similarity_threshold-val ((m <AlignmentPrerejectiveParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:similarity_threshold-val is deprecated.  Use vrm3dvision-msg:similarity_threshold instead.")
  (similarity_threshold m))

(cl:ensure-generic-function 'max_iterations-val :lambda-list '(m))
(cl:defmethod max_iterations-val ((m <AlignmentPrerejectiveParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:max_iterations-val is deprecated.  Use vrm3dvision-msg:max_iterations instead.")
  (max_iterations m))

(cl:ensure-generic-function 'max_correspondence_distance_ratio_leaf-val :lambda-list '(m))
(cl:defmethod max_correspondence_distance_ratio_leaf-val ((m <AlignmentPrerejectiveParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:max_correspondence_distance_ratio_leaf-val is deprecated.  Use vrm3dvision-msg:max_correspondence_distance_ratio_leaf instead.")
  (max_correspondence_distance_ratio_leaf m))

(cl:ensure-generic-function 'inlier_fraction-val :lambda-list '(m))
(cl:defmethod inlier_fraction-val ((m <AlignmentPrerejectiveParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:inlier_fraction-val is deprecated.  Use vrm3dvision-msg:inlier_fraction instead.")
  (inlier_fraction m))

(cl:ensure-generic-function 'icp_max_iterations-val :lambda-list '(m))
(cl:defmethod icp_max_iterations-val ((m <AlignmentPrerejectiveParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:icp_max_iterations-val is deprecated.  Use vrm3dvision-msg:icp_max_iterations instead.")
  (icp_max_iterations m))

(cl:ensure-generic-function 'icp_max_correspondence_distance_ratio_leaf-val :lambda-list '(m))
(cl:defmethod icp_max_correspondence_distance_ratio_leaf-val ((m <AlignmentPrerejectiveParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:icp_max_correspondence_distance_ratio_leaf-val is deprecated.  Use vrm3dvision-msg:icp_max_correspondence_distance_ratio_leaf instead.")
  (icp_max_correspondence_distance_ratio_leaf m))

(cl:ensure-generic-function 'min_score-val :lambda-list '(m))
(cl:defmethod min_score-val ((m <AlignmentPrerejectiveParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:min_score-val is deprecated.  Use vrm3dvision-msg:min_score instead.")
  (min_score m))

(cl:ensure-generic-function 'min_score_threshold-val :lambda-list '(m))
(cl:defmethod min_score_threshold-val ((m <AlignmentPrerejectiveParams>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrm3dvision-msg:min_score_threshold-val is deprecated.  Use vrm3dvision-msg:min_score_threshold instead.")
  (min_score_threshold m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AlignmentPrerejectiveParams>) ostream)
  "Serializes a message object of type '<AlignmentPrerejectiveParams>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'leaf_size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'normal_radius_ratio_leaf))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'feature_radius_ratio_leaf))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'correspondence_randomness)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'correspondence_randomness)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'correspondence_randomness)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'correspondence_randomness)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'similarity_threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'max_iterations)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'max_iterations)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'max_iterations)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'max_iterations)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_correspondence_distance_ratio_leaf))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'inlier_fraction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'icp_max_iterations)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'icp_max_iterations)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'icp_max_iterations)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'icp_max_iterations)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'icp_max_correspondence_distance_ratio_leaf))))
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
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min_score_threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AlignmentPrerejectiveParams>) istream)
  "Deserializes a message object of type '<AlignmentPrerejectiveParams>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'leaf_size) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'normal_radius_ratio_leaf) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'feature_radius_ratio_leaf) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'correspondence_randomness)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'correspondence_randomness)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'correspondence_randomness)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'correspondence_randomness)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'similarity_threshold) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'max_iterations)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'max_iterations)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'max_iterations)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'max_iterations)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_correspondence_distance_ratio_leaf) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'inlier_fraction) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'icp_max_iterations)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'icp_max_iterations)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'icp_max_iterations)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'icp_max_iterations)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'icp_max_correspondence_distance_ratio_leaf) (roslisp-utils:decode-double-float-bits bits)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AlignmentPrerejectiveParams>)))
  "Returns string type for a message object of type '<AlignmentPrerejectiveParams>"
  "vrm3dvision/AlignmentPrerejectiveParams")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AlignmentPrerejectiveParams)))
  "Returns string type for a message object of type 'AlignmentPrerejectiveParams"
  "vrm3dvision/AlignmentPrerejectiveParams")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AlignmentPrerejectiveParams>)))
  "Returns md5sum for a message object of type '<AlignmentPrerejectiveParams>"
  "a20f28cde22dcefbeefd241cf356edb6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AlignmentPrerejectiveParams)))
  "Returns md5sum for a message object of type 'AlignmentPrerejectiveParams"
  "a20f28cde22dcefbeefd241cf356edb6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AlignmentPrerejectiveParams>)))
  "Returns full string definition for message of type '<AlignmentPrerejectiveParams>"
  (cl:format cl:nil "#### Common parameters~%float64 leaf_size~%float64 normal_radius_ratio_leaf~%float64 feature_radius_ratio_leaf~%uint32 	correspondence_randomness~%float64 similarity_threshold~%uint32 	max_iterations~%float64 max_correspondence_distance_ratio_leaf~%float64 inlier_fraction~%~%## ICP~%uint32 	icp_max_iterations~%float64 icp_max_correspondence_distance_ratio_leaf~%~%# Alignment Prerejective~%float64 min_score~%float64 min_score_threshold~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AlignmentPrerejectiveParams)))
  "Returns full string definition for message of type 'AlignmentPrerejectiveParams"
  (cl:format cl:nil "#### Common parameters~%float64 leaf_size~%float64 normal_radius_ratio_leaf~%float64 feature_radius_ratio_leaf~%uint32 	correspondence_randomness~%float64 similarity_threshold~%uint32 	max_iterations~%float64 max_correspondence_distance_ratio_leaf~%float64 inlier_fraction~%~%## ICP~%uint32 	icp_max_iterations~%float64 icp_max_correspondence_distance_ratio_leaf~%~%# Alignment Prerejective~%float64 min_score~%float64 min_score_threshold~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AlignmentPrerejectiveParams>))
  (cl:+ 0
     8
     8
     8
     4
     8
     4
     8
     8
     4
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AlignmentPrerejectiveParams>))
  "Converts a ROS message object to a list"
  (cl:list 'AlignmentPrerejectiveParams
    (cl:cons ':leaf_size (leaf_size msg))
    (cl:cons ':normal_radius_ratio_leaf (normal_radius_ratio_leaf msg))
    (cl:cons ':feature_radius_ratio_leaf (feature_radius_ratio_leaf msg))
    (cl:cons ':correspondence_randomness (correspondence_randomness msg))
    (cl:cons ':similarity_threshold (similarity_threshold msg))
    (cl:cons ':max_iterations (max_iterations msg))
    (cl:cons ':max_correspondence_distance_ratio_leaf (max_correspondence_distance_ratio_leaf msg))
    (cl:cons ':inlier_fraction (inlier_fraction msg))
    (cl:cons ':icp_max_iterations (icp_max_iterations msg))
    (cl:cons ':icp_max_correspondence_distance_ratio_leaf (icp_max_correspondence_distance_ratio_leaf msg))
    (cl:cons ':min_score (min_score msg))
    (cl:cons ':min_score_threshold (min_score_threshold msg))
))
