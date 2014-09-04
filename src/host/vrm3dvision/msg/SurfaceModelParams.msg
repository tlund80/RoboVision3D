#### Common parameters
float64 rel_sampling_distance

#### Creation parameters
int8 	model_invert_normals
float64 pose_ref_rel_sampling_distance
float64 feat_step_size_rel
uint16 feat_angle_resolution

#### Detection parameters

## Approximate matching
float64 key_point_fraction
float64 min_score
int8 	return_result_handle
int8 	num_matches
float64 max_overlap_dist_rel

## Sparse pose refinement
int8 	sparse_pose_refinement
string 	score_type
int8 	pose_ref_use_scene_normals

## Dense pose refinement
int8 	dense_pose_refinement
uint16 	pose_ref_num_steps
uint16 	pose_ref_sub_sampling
float64 pose_ref_dist_threshold_rel
float64 pose_ref_scoring_dist_rel

## Evaluation
float64 min_score_threshold
bool 	reset_to_default


