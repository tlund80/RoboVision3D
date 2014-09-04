/********************************************************************************************************************
 *
 * \file                object_model.cpp
 * \author              Kent Hansen (kenh@teknologisk.dk)
 * \date                2014-03-18
 * \version             1.0
 * \brief               Object model class
 *
*********************************************************************************************************************/

#include "object_model.hpp"

namespace vrm3dvision {

ObjectModel::ObjectModel(int tesselation_level, int resolution, float sphere_radius, float field_of_view, int view_buffer_size) :
		num_views_(0),
		tesselation_level_(tesselation_level),
		resolution_(resolution),
		sphere_radius_(sphere_radius),
		field_of_view_(field_of_view),
		view_buffer_size_(view_buffer_size),
		single_expo_(5.0),
		single_edges_(1),
		double_edges_(1),
		double_expo_low_(3.0),
		double_expo_high_(10.0)
{
	surface_model_params_.rel_sampling_distance = 0.03;
	resetParamsDefault();
}

ObjectModel::~ObjectModel()
{
}

bool ObjectModel::initFromModel(const std::string& model_name)
{
	// Load model information
	if (model_name.at(0) != '/') // use default path
	{
		model_folder_ = getPathToModels() + model_name;
	}
	else // Use complete path to model
	{
		model_folder_ = model_name;
	}

	std::string model_header = model_folder_ + "/model_data.txt";
	std::string model_mesh = model_folder_ + "/model_mesh.stl";
	std::ifstream file(model_header.c_str());
	if (file.is_open())
	{
		file >> *this;
	}
	else
	{
		ROS_WARN_STREAM("Unable to open model data file from path: " << model_header);
		return false;
	}

	// Load mesh model
	if (!pcl::io::loadPolygonFile(model_mesh, model_mesh_))
	{
		ROS_WARN_STREAM("Unable to load mesh model");
		return false;
	}

	// Load pose estimation parameters
	loadParameters();

	// Init view set
	loadViewScore();

	return true;
}



bool ObjectModel::loadRawClouds()
{
	if((int)raw_clouds_.size() == num_views_)
	{
		return true;
	}
	else
	{
		raw_clouds_.clear();
		for (int i = 0; i < num_views_; i++)
		{
			std::stringstream ss;
			ss << model_folder_ << "/view_" << i << ".pcd";
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

			if ( boost::filesystem::exists( ss.str() ) )
			{
				pcl::io::loadPCDFile(ss.str(), *temp_cloud);
				raw_clouds_.push_back(temp_cloud);
			}
			else
			{
				ROS_WARN_STREAM("Failed to load raw cloud with path: " << ss.str());
				return false;
			}
		}
	}
	return true;
}



bool ObjectModel::createNewModelFromCad(const std::string& model_name, const std::string& cad_path)
{
	model_name_ = model_name;
	model_folder_ = getPathToModels() + model_name;

	if(!modelFolderExists())
	{
		if(loadCadModel(cad_path, model_mesh_))
		{
			ROS_INFO_STREAM("Creating new object model \"" << model_name << "\" from CAD file: " << cad_path);
			// Create multiple views of the model
			std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> views_xyz;
			std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;

			vtkSmartPointer< vtkPolyData > triangles_out_vtk;
			pcl::VTKUtils::mesh2vtk(model_mesh_, triangles_out_vtk);

			pcl::apps::RenderViewsTesselatedSphere rvts;
			rvts.addModelFromPolyData(triangles_out_vtk);
			rvts.setTesselationLevel(tesselation_level_);
			rvts.setResolution(resolution_);
			rvts.setRadiusSphere(sphere_radius_);
			rvts.setViewAngle(field_of_view_);

			rvts.generateViews();
			rvts.getViews(views_xyz);
			rvts.getPoses(poses);

			num_views_ = views_xyz.size();

			for (size_t i = 0; i < views_xyz.size(); i++)
			{
				// Copy view pose
				poses_.push_back(poses[i]);

				// Add raw view to list
				raw_clouds_.push_back(views_xyz[i]);
			}
			saveModel(cad_path);

			// Init view set
			initViewSet();

		}
		else
		{
			ROS_WARN_STREAM("Failed to load CAD model from path: " << cad_path);
			return false;
		}
	}
	else
	{
		ROS_WARN_STREAM("Model folder already exists. Loading data...");
		initFromModel(model_name);

	}

	return true;
}

std::vector<int>& ObjectModel::getViewOrder()
{
	view_idx.clear();
	for (view_set_itr = view_set.begin(); view_set_itr != view_set.end(); view_set_itr++)
	{
		view_idx.push_back(view_set_itr->second);
	}
	return view_idx;
}

float ObjectModel::getFitness(int index)
{
	for (view_set_itr = view_set.begin(); view_set_itr != view_set.end(); view_set_itr++)
	{
		if (index == view_set_itr->second)
			break;
	}
	return float(view_set_itr->first) / float(view_buffer.size());
}


void ObjectModel::addSuccededView(int index)
{
	for (view_set_itr = view_set.begin(); view_set_itr != view_set.end(); view_set_itr++)
	{
		if (index == view_set_itr->second)
			break;
	}

	if (view_set_itr != view_set.end())
	{
		std::pair<int, int> tmp_pair(view_set_itr->first+1, view_set_itr->second);
		view_set.erase(view_set_itr);
		view_set.insert(tmp_pair);
	}

	for (view_set_itr = view_set.begin(); view_set_itr != view_set.end(); view_set_itr++)
	{
		if (view_buffer.at(0) == view_set_itr->second)
			break;
	}

	if (view_set_itr != view_set.end())
	{
		std::pair<int, int> tmp_pair(view_set_itr->first-1, view_set_itr->second);
		view_set.erase(view_set_itr);
		view_set.insert(tmp_pair);
	}

	view_buffer.push_back(index);
}



bool ObjectModel::updateParameters(const SurfaceModelParams& smp, const AlignmentPrerejectiveParams& app)
{
	return (updateSurfaceModelParams(smp) && updateAlignmentPrerejectiveParams(app));
}

bool ObjectModel::updateSurfaceModelParams(const SurfaceModelParams& new_params)
{
	bool ret = true;

	if(new_params.reset_to_default)
	{
		resetSurfaceModelParamsDefault();
	}
	else
	{
		if(new_params.rel_sampling_distance > 0.0 && new_params.rel_sampling_distance < 1.0)
		{
			surface_model_params_.rel_sampling_distance = new_params.rel_sampling_distance;
		}
		else if(new_params.rel_sampling_distance != 0.0)
		{
			ROS_WARN_STREAM("Error in updateSurfaceModelParams(): Value outside range: 0.0 < rel_sampling_distance < 1.0");
			ret = false;
		}

		if(new_params.model_invert_normals != 0)
		{
			surface_model_params_.model_invert_normals = new_params.model_invert_normals;
		}

		if(new_params.pose_ref_rel_sampling_distance > 0.0 && new_params.pose_ref_rel_sampling_distance < 1.0)
		{
			surface_model_params_.pose_ref_rel_sampling_distance = new_params.pose_ref_rel_sampling_distance;
		}
		else if(new_params.pose_ref_rel_sampling_distance != 0.0)
		{
			ROS_WARN_STREAM("Error in updateSurfaceModelParams(): Value outside range: 0.0 < pose_ref_rel_sampling_distance < 1.0");
			ret = false;
		}

		if(new_params.feat_step_size_rel > 0.0 && new_params.feat_step_size_rel < 1.0)
		{
			surface_model_params_.feat_step_size_rel = new_params.feat_step_size_rel;
		}
		else if(new_params.feat_step_size_rel != 0.0)
		{
			ROS_WARN_STREAM("Error in updateSurfaceModelParams(): Value outside range: 0.0 < feat_step_size_rel < 1.0");
			ret = false;
		}

		if(new_params.feat_angle_resolution > 1)
		{
			surface_model_params_.feat_angle_resolution = new_params.feat_angle_resolution;
		}

		if(new_params.key_point_fraction > 0.0 && new_params.key_point_fraction <= 1.0)
		{
			surface_model_params_.key_point_fraction = new_params.key_point_fraction;
		}
		else if(new_params.key_point_fraction != 0.0)
		{
			ROS_WARN_STREAM("Error in updateSurfaceModelParams(): Value outside range: 0.0 < key_point_fraction <= 1.0");
			ret = false;
		}

		if(new_params.min_score > 0.0)
		{
			surface_model_params_.min_score = new_params.min_score;
		}

		if(new_params.return_result_handle != 0)
		{
			surface_model_params_.return_result_handle = new_params.return_result_handle;
		}

		if(new_params.num_matches > 0)
		{
			surface_model_params_.num_matches = new_params.num_matches;
		}

		if(new_params.max_overlap_dist_rel > 0.0 && new_params.max_overlap_dist_rel <= 1.0)
		{
			surface_model_params_.max_overlap_dist_rel = new_params.max_overlap_dist_rel;
		}
		else if(new_params.max_overlap_dist_rel != 0.0)
		{
			ROS_WARN_STREAM("Error in updateSurfaceModelParams(): Value outside range: 0.0 < max_overlap_dist_rel <= 1.0");
			ret = false;
		}

		if(new_params.sparse_pose_refinement != 0)
		{
			surface_model_params_.sparse_pose_refinement = new_params.sparse_pose_refinement;
		}

		if(!new_params.score_type.empty())
		{
			surface_model_params_.score_type = new_params.score_type;
		}

		if(new_params.pose_ref_use_scene_normals != 0)
		{
			surface_model_params_.pose_ref_use_scene_normals = new_params.pose_ref_use_scene_normals;
		}

		if(new_params.dense_pose_refinement != 0)
		{
			surface_model_params_.dense_pose_refinement = new_params.dense_pose_refinement;
		}

		if(new_params.pose_ref_num_steps > 0)
		{
			surface_model_params_.pose_ref_num_steps = new_params.pose_ref_num_steps;
		}

		if(new_params.pose_ref_sub_sampling > 0)
		{
			surface_model_params_.pose_ref_sub_sampling = new_params.pose_ref_sub_sampling;
		}

		if(new_params.pose_ref_dist_threshold_rel > 0.0)
		{
			surface_model_params_.pose_ref_dist_threshold_rel = new_params.pose_ref_dist_threshold_rel;
		}

		if(new_params.pose_ref_scoring_dist_rel > 0.0)
		{
			surface_model_params_.pose_ref_scoring_dist_rel = new_params.pose_ref_scoring_dist_rel;
		}

		if(new_params.min_score_threshold > 0.0)
		{
			surface_model_params_.min_score_threshold = new_params.min_score_threshold;
		}
	}

	saveSurfaceModelParams();
	return ret;
}



bool ObjectModel::updateAlignmentPrerejectiveParams(const AlignmentPrerejectiveParams& new_params)
{
	bool ret = true;
	if(new_params.leaf_size > 0.0)
	{
		alignment_prerejective_params_.leaf_size = new_params.leaf_size;
	}
	if(new_params.normal_radius_ratio_leaf > 0.0)
	{
		alignment_prerejective_params_.normal_radius_ratio_leaf = new_params.normal_radius_ratio_leaf;
	}
	if(new_params.feature_radius_ratio_leaf > 0.0)
	{
		alignment_prerejective_params_.feature_radius_ratio_leaf = new_params.feature_radius_ratio_leaf;
	}
	if(new_params.correspondence_randomness > 0)
	{
		alignment_prerejective_params_.correspondence_randomness = new_params.correspondence_randomness;
	}
	if(new_params.similarity_threshold > 0.0 && new_params.similarity_threshold <= 1.0)
	{
		alignment_prerejective_params_.similarity_threshold = new_params.similarity_threshold;
	}
	if(new_params.max_iterations > 0)
	{
		alignment_prerejective_params_.max_iterations = new_params.max_iterations;
	}
	if(new_params.max_correspondence_distance_ratio_leaf > 0.0)
	{
		alignment_prerejective_params_.max_correspondence_distance_ratio_leaf = new_params.max_correspondence_distance_ratio_leaf;
	}
	if(new_params.inlier_fraction > 0.0 && new_params.inlier_fraction <= 1.0)
	{
		alignment_prerejective_params_.inlier_fraction = new_params.inlier_fraction;
	}
	if(new_params.icp_max_iterations > 0.0)
	{
		alignment_prerejective_params_.icp_max_iterations = new_params.icp_max_iterations;
	}
	if(new_params.icp_max_correspondence_distance_ratio_leaf > 0.0)
	{
		alignment_prerejective_params_.icp_max_correspondence_distance_ratio_leaf = new_params.icp_max_correspondence_distance_ratio_leaf;
	}
	if(new_params.min_score > 0.0 && new_params.min_score <= 100.0)
	{
		alignment_prerejective_params_.min_score = new_params.min_score;
	}
	if(new_params.min_score_threshold > 0.0 && new_params.min_score_threshold <= 100.0)
	{
		alignment_prerejective_params_.min_score_threshold = new_params.min_score_threshold;
	}

	saveAlignmentPrerejectiveParams();

	return ret;
}



bool ObjectModel::saveViewScore()
{
	std::string file_path(model_folder_ + "/view_score.txt");
	std::ofstream os(file_path.c_str());

	if (os.is_open())
	{
		os << view_buffer.size() << " ";

		for (size_t i = 0; i < view_buffer.size(); i++)
		{
			os << view_buffer.at(i) << " ";
		}
		os.close();
		return true;
	}
	else
	{
		ROS_WARN_STREAM("Failed to create file for surface model parameters");
		return false;
	}
}

bool ObjectModel::initViewSet()
{
	view_set.clear();
	for (int i = 0; i < num_views_; i++)
	{
		std::pair<int, int> tmp(0,i);
		view_set.insert(tmp);
	}

	// Init circular buffer
	view_buffer.resize(view_buffer_size_);
	for (int i = 0; i < view_buffer_size_; i++)
	{
		view_buffer.push_back(-1);
	}

	return true;
}

bool ObjectModel::loadViewScore()
{
	bool ret = false;
	std::string file_path(model_folder_ + "/view_score.txt");
	std::ifstream is(file_path.c_str());

	initViewSet();

	if (is.is_open())
	{
		is >> view_buffer_size_;
		view_buffer.resize(view_buffer_size_);

		for (int i = 0; i < view_buffer_size_; i++)
		{
			int tmp;
			is >> tmp;
			addSuccededView(tmp);
		}

		is.close();

		ret = true;
	}

	return ret;
}



void ObjectModel::resetParamsDefault()
{
	resetSurfaceModelParamsDefault();
	resetAlignmentPrerejectiveParamsDefault();
}


void ObjectModel::resetSurfaceModelParamsDefault()
{
	surface_model_params_.model_invert_normals = -1;
	surface_model_params_.pose_ref_rel_sampling_distance = 0.01;
	surface_model_params_.feat_step_size_rel = surface_model_params_.rel_sampling_distance;
	surface_model_params_.feat_angle_resolution = 30.0;
	surface_model_params_.key_point_fraction = 0.2;
	surface_model_params_.min_score = 0.5;
	surface_model_params_.return_result_handle = 1;
	surface_model_params_.num_matches = 1;
	surface_model_params_.max_overlap_dist_rel = 0.5;
	surface_model_params_.sparse_pose_refinement = 1;
	surface_model_params_.score_type = "model_point_fraction";
	surface_model_params_.pose_ref_use_scene_normals = -1;
	surface_model_params_.dense_pose_refinement = 1;
	surface_model_params_.pose_ref_num_steps = 5;
	surface_model_params_.pose_ref_sub_sampling = 2;
	surface_model_params_.pose_ref_dist_threshold_rel = 0.1;
	surface_model_params_.pose_ref_scoring_dist_rel = 0.005;
	surface_model_params_.min_score_threshold = 0.8;
	surface_model_params_.reset_to_default = false;
}



void ObjectModel::resetAlignmentPrerejectiveParamsDefault()
{
	alignment_prerejective_params_.leaf_size = 0.002;
	alignment_prerejective_params_.normal_radius_ratio_leaf = 2.5;
	alignment_prerejective_params_.feature_radius_ratio_leaf = 5.0;
	alignment_prerejective_params_.correspondence_randomness = 5;
	alignment_prerejective_params_.similarity_threshold = 0.6;
	alignment_prerejective_params_.max_iterations = 100;
	alignment_prerejective_params_.max_correspondence_distance_ratio_leaf = 1.5;
	alignment_prerejective_params_.inlier_fraction = 0.5;
	alignment_prerejective_params_.icp_max_iterations = 200;
	alignment_prerejective_params_.icp_max_correspondence_distance_ratio_leaf = 2.25;
	alignment_prerejective_params_.min_score = 50;
	alignment_prerejective_params_.min_score_threshold = 58;
}



bool ObjectModel::saveParameters()
{
	return (saveSurfaceModelParams() && saveAlignmentPrerejectiveParams());
}



bool ObjectModel::saveSurfaceModelParams()
{
	std::string file_path(model_folder_ + "/surface_model_params.txt");
	std::ofstream os(file_path.c_str());

	if (os.is_open())
	{
		os << 	surface_model_params_.rel_sampling_distance << " " <<
				surface_model_params_.model_invert_normals << " " <<
				surface_model_params_.pose_ref_rel_sampling_distance << " " <<
				surface_model_params_.feat_step_size_rel << " " <<
				surface_model_params_.feat_angle_resolution << " " <<
				surface_model_params_.key_point_fraction << " " <<
				surface_model_params_.min_score << " " <<
				surface_model_params_.return_result_handle << " " <<
				surface_model_params_.num_matches  << " " <<
				surface_model_params_.max_overlap_dist_rel  << " " <<
				surface_model_params_.sparse_pose_refinement  << " " <<
				surface_model_params_.score_type  << " " <<
				surface_model_params_.pose_ref_use_scene_normals  << " " <<
				surface_model_params_.dense_pose_refinement  << " " <<
				surface_model_params_.pose_ref_num_steps  << " " <<
				surface_model_params_.pose_ref_sub_sampling  << " " <<
				surface_model_params_.pose_ref_dist_threshold_rel  << " " <<
				surface_model_params_.pose_ref_scoring_dist_rel;
		os.close();
		return true;
	}
	else
	{
		ROS_WARN_STREAM("Failed to create file for surface model parameters");
		return false;
	}
}



bool ObjectModel::saveAlignmentPrerejectiveParams()
{
	std::string file_path(model_folder_ + "/alignment_prerejective_params.txt");
	std::ofstream os(file_path.c_str());

	if (os.is_open())
	{
		os << 	alignment_prerejective_params_.leaf_size << " " <<
				alignment_prerejective_params_.normal_radius_ratio_leaf << " " <<
				alignment_prerejective_params_.feature_radius_ratio_leaf << " " <<
				alignment_prerejective_params_.correspondence_randomness << " " <<
				alignment_prerejective_params_.similarity_threshold << " " <<
				alignment_prerejective_params_.max_iterations << " " <<
				alignment_prerejective_params_.max_correspondence_distance_ratio_leaf << " " <<
				alignment_prerejective_params_.inlier_fraction << " " <<
				alignment_prerejective_params_.icp_max_iterations << " " <<
				alignment_prerejective_params_.icp_max_correspondence_distance_ratio_leaf << " " <<
				alignment_prerejective_params_.min_score << " " <<
				alignment_prerejective_params_.min_score_threshold;
		os.close();
		return true;
	}
	else
	{
		ROS_WARN_STREAM("Failed to create file for alignment prerejective parameters");
		return false;
	}
}



bool ObjectModel::loadParameters()
{
	return (loadSurfaceModelParams() && loadAlignmentPrerejectiveParams());
}



bool ObjectModel::loadSurfaceModelParams()
{
	std::string file_path(model_folder_ + "/surface_model_params.txt");
	std::ifstream is(file_path.c_str());

	if (is.is_open())
	{
		is >>	surface_model_params_.rel_sampling_distance;
		is >>	surface_model_params_.model_invert_normals;
		is >>	surface_model_params_.pose_ref_rel_sampling_distance;
		is >>	surface_model_params_.feat_step_size_rel;
		is >>	surface_model_params_.feat_angle_resolution;
		is >>	surface_model_params_.key_point_fraction;
		is >>	surface_model_params_.min_score;
		is >>	surface_model_params_.return_result_handle;
		is >>	surface_model_params_.num_matches;
		is >>	surface_model_params_.max_overlap_dist_rel;
		is >>	surface_model_params_.sparse_pose_refinement;
		is >>	surface_model_params_.score_type;
		is >>	surface_model_params_.pose_ref_use_scene_normals;
		is >>	surface_model_params_.dense_pose_refinement;
		is >>	surface_model_params_.pose_ref_num_steps;
		is >>	surface_model_params_.pose_ref_sub_sampling;
		is >>	surface_model_params_.pose_ref_dist_threshold_rel;
		is >>	surface_model_params_.pose_ref_scoring_dist_rel;
		is.close();
		return true;
	}
	else
	{
		ROS_WARN_STREAM("Failed to load file for surface model parameters");
		return false;
	}
}



bool ObjectModel::loadAlignmentPrerejectiveParams()
{
	std::string file_path(model_folder_ + "/alignment_prerejective_params.txt");
	std::ifstream is(file_path.c_str());

	if (is.is_open())
	{
		is >> 	alignment_prerejective_params_.leaf_size;
		is >> 	alignment_prerejective_params_.normal_radius_ratio_leaf;
		is >> 	alignment_prerejective_params_.feature_radius_ratio_leaf;
		is >> 	alignment_prerejective_params_.correspondence_randomness;
		is >> 	alignment_prerejective_params_.similarity_threshold;
		is >> 	alignment_prerejective_params_.max_iterations;
		is >> 	alignment_prerejective_params_.max_correspondence_distance_ratio_leaf;
		is >> 	alignment_prerejective_params_.inlier_fraction;
		is >> 	alignment_prerejective_params_.icp_max_iterations;
		is >> 	alignment_prerejective_params_.icp_max_correspondence_distance_ratio_leaf;
		is >> 	alignment_prerejective_params_.min_score;
		is >> 	alignment_prerejective_params_.min_score_threshold;
		is.close();
		return true;
	}
	else
	{
		ROS_WARN_STREAM("Failed to load file for alignment prerejective parameters");
		return false;
	}
}



bool ObjectModel::loadCadModel(const std::string& cad_path, pcl::PolygonMesh& mesh)
{
	bool ret = false;
	std::string filetype = cad_path.substr(cad_path.find_last_of(".") + 1);
	if (!boost::filesystem::exists(cad_path))
	{
		ROS_WARN_STREAM("File does not exist with path: \"" << cad_path << "\"");
		return ret;
	}
	if (filetype == "stl" || filetype == "STL")
	{
		if (pcl::io::loadPolygonFileSTL(cad_path, mesh))
		{
			ret = true;
		}
	}
	else if (filetype == "obj" || filetype == "OBJ")
	{
		if (pcl::io::loadPolygonFileOBJ(cad_path, mesh))
		{
			ret = true;
		}
	}
	else if (filetype == "ply" || filetype == "PLY")
	{
		if (pcl::io::loadPolygonFilePLY(cad_path, mesh))
		{
			ret = true;
		}
	}
	else
	{
		ROS_WARN_STREAM("Filetype not supported! - needs to be .obj, .stl or .ply - filetype was: " << filetype);
	}
	return ret;
}



std::ostream& operator <<(std::ostream &os, const ObjectModel& om)
{
	os 	<< om.model_name_ << " "
		<< om.num_views_ << " "
		<< om.single_expo_ << " "
		<< om.double_expo_low_ << " "
		<< om.double_expo_high_ << " "
		<< om.single_edges_ << " "
		<< om.double_edges_ << std::endl;
	for (int i = 0; i < om.num_views_; i++)
	{
		os << om.poses_[i] << std::endl;
	}
	return os;
}



std::istream& operator >>(std::istream &is, ObjectModel& om)
{
	is >> om.model_name_;
	is >> om.num_views_;
	is >> om.single_expo_;
	is >> om.double_expo_low_;
	is >> om.double_expo_high_;
	is >> om.single_edges_;
	is >> om.double_edges_;

	om.poses_.clear();
	for (int it = 0; it < om.num_views_; it++)
	{
		Eigen::Matrix4f pose;
		for (int x = 0; x < 4; x++)
		{
			for (int y = 0; y < 4; y++)
			{
				is >> pose(x,y);
			}
		}
		om.poses_.push_back(pose);
	}
	return is;
}



bool ObjectModel::saveModelData()
{
	std::string model_filename(model_folder_ + "/model_data.txt");
	std::ofstream file;
	file.open(model_filename.c_str());
	if (file.is_open())
	{
		file << *this;
		file.close();
		return true;
	}
	else
	{
		ROS_WARN_STREAM("Unable to open data file for model " << model_name_ );
		return false;
	}
}



bool ObjectModel::saveModel(const std::string& cad_path)
{
	if (boost::filesystem::create_directories(model_folder_))
	{
		// Save mesh model
		std::string mesh_file_str = model_folder_ + "/model_mesh.stl";
		pcl::io::savePolygonFileSTL(mesh_file_str, model_mesh_);

		// Save information about model
		saveModelData();

		// Save views
		for (size_t i = 0; i < raw_clouds_.size(); i++)
		{
			std::stringstream ss;
			ss << model_folder_ << "/view_" << i << ".pcd";
			pcl::io::savePCDFileBinary(ss.str(), *raw_clouds_[i]);
		}

		saveParameters();
	}
	else
	{
		ROS_WARN_STREAM("Unable to create folder named: " << model_folder_);
	}
	return true;
}



bool ObjectModel::modelFolderExists()
{
	return boost::filesystem::exists(model_folder_);
}



std::string ObjectModel::getBestExpoString()
{
	if(double_edges_ > 1.5 * single_edges_)
	{
		return getDoubleExpoString();
	}
	else
	{
		return getSingleExpoString();
	}
}



std::string ObjectModel::getSingleExpoString()
{
	std::stringstream ss;
	ss << getSingleExpo();
	return ss.str();
}



std::string ObjectModel::getDoubleExpoString()
{
	std::stringstream ss;
	ss << getDoubleExpoLow() << " " << getDoubleExpoHigh();
	return ss.str();
}



void ObjectModel::updateExposure(double single, int single_edges, double double_low, double double_high, int double_edges)
{
	setSingleExpo(single);
	setDoubleExpoLow(double_low);
	setDoubleExpoHigh(double_high);
	single_edges_ = single_edges;
	double_edges_ = double_edges;
	saveModelData();
}



} /* namespace vrm3dvision */
