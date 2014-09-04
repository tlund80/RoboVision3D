/*
 * surface_model_estimator.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: kent
 */

#include "surface_model_estimator.hpp"

namespace vrm3dvision {

using namespace HalconCpp;

SurfaceModelEstimator::SurfaceModelEstimator()
{
}



SurfaceModelEstimator::~SurfaceModelEstimator()
{
}

bool SurfaceModelEstimator::isHalconLicenseAvailable()
{
	try
	{
		HTuple tmp_tuple;
		tmp_tuple.Append("Er Du der halcon?");
	}
	catch (HException &e)
	{
		return false;
	}
	return true;
}

bool SurfaceModelEstimator::prepareModel(ObjectModel& model)
{
	try
	{
		std::string halcon_folder = model.getModelFolder() + "/halcon";

		if(boost::filesystem::exists(halcon_folder))
		{
			model.surface_model_ids_.clear();
			for(int i = 0; i < model.getNumViews(); i++)
			{
				HTuple model_handle;
				std::stringstream ss;
				ss << halcon_folder << "/sfm_" << i << ".sfm";
				ReadSurfaceModel(HTuple(ss.str().c_str()), &model_handle);
				model.surface_model_ids_.push_back((int) model_handle);
			}
		}
		else
		{
			if (boost::filesystem::create_directories(halcon_folder))
			{
				model.surface_model_ids_.clear();
				model.loadRawClouds();
				for(int i = 0; i < model.getNumViews(); i++)
				{
					HalconCpp::HTuple x, y, z;
					convertCloudToHalcon(*model.raw_clouds_[i], x, y, z);

					HalconCpp::HTuple model_handle;
					createSurfaceModelFromPoints(x, y, z, model.surface_model_params_, &model_handle);

					std::stringstream ss;
					ss << model.getModelFolder() << "/halcon/sfm_" << i << ".sfm";
					HalconCpp::WriteSurfaceModel(model_handle, HTuple(ss.str().c_str()));
					model.surface_model_ids_.push_back((int) model_handle);
				}
			}
			else
			{
				ROS_WARN_STREAM("Failed to create folder for Halcon surface models.");
				return false;
			}
		}

	}
	catch (HException &e)
	{
		ROS_WARN_STREAM("Halcon error:" << (int) e.ErrorNumber());
		return false;
	}

	return true;
}



bool SurfaceModelEstimator::estimatePose(const pcl::PointCloud<pcl::PointXYZRGB>& input_cloud, ObjectModel& model, Eigen::Matrix4f& t)
{
	try
	{
		ROS_INFO_STREAM("Pose estimation using Halcon with " << model.surface_model_ids_.size() << " surface models");

		SurfaceModelParams params = model.surface_model_params_;
		HTuple gen_param_name, gen_param_value, pose, score, result_id;
		parseDetectionParameters(params, gen_param_name, gen_param_value);

		int best_index = 0;
		double best_score = 0;
		HTuple best_pose;

		ROS_INFO_STREAM("RelSamplingDistance: " << params.rel_sampling_distance << " KeyPointFraction: " << params.key_point_fraction
						<< " MinScore: " << params.min_score << " MinScoreThreshold: " << params.min_score_threshold);

		double tm = ros::Time::now().toSec();

		HalconCpp::HTuple x, y, z;
		convertCloudToHalcon(input_cloud, x, y, z);

		HalconCpp::HTuple object_handle;
		createObjectModelWithNormals(x, y, z, &object_handle);

		ROS_INFO_STREAM("Created object model with normals in: " << (ros::Time::now().toSec()-tm)*1000 << " ms");

		std::vector<int> indexes = model.getViewOrder();

		for(size_t i = 0; i < indexes.size() && best_score < params.min_score_threshold; i++)
		{
			FindSurfaceModel(HTuple(model.surface_model_ids_[indexes[i]]), object_handle, HTuple(params.rel_sampling_distance),
					HTuple(params.key_point_fraction), HTuple(params.min_score), HTuple("true"),
					gen_param_name, gen_param_value, &pose, &score, &result_id);
			if( (int) score.TupleLength() > 0 )
			{
				if((double) score > best_score)
				{
					best_score = (double) score;
					best_pose = pose;
					best_index = indexes[i];
					ROS_INFO_STREAM("New best score: " << best_score << " from view " << best_index);
				}
			}
		}
		ROS_INFO_STREAM("Best score: " << best_score << " from view " << best_index << " Total time: " << (ros::Time::now().toSec()-tm)*1000 << " ms");

		if(best_score > params.min_score)
		{
			model.addSuccededView(best_index);

			HTuple HomMat;
			PoseToHomMat3d(best_pose,&HomMat);

			t(0,0) = (double)HomMat[0]; t(0,1) = (double)HomMat[1]; t(0,2) = (double)HomMat[2]; t(0,3) = (double)HomMat[3];
			t(1,0) = (double)HomMat[4]; t(1,1) = (double)HomMat[5]; t(1,2) = (double)HomMat[6]; t(1,3) = (double)HomMat[7];
			t(2,0) = (double)HomMat[8]; t(2,1) = (double)HomMat[9]; t(2,2) = (double)HomMat[10]; t(2,3) = (double)HomMat[11];
			t(3,0) = 0; t(3,1) = 0; t(3,2) = 0; t(3,3) = 1;

			t = t * model.poses_[best_index];



			return true;
		}
		else
		{
			return false;
		}
	}
	catch (HException &e)
	{
		ROS_WARN_STREAM("Halcon error: " << (int) e.ErrorNumber());
		return false;
	}
}



bool SurfaceModelEstimator::createSurfaceModelFromPoints(const HalconCpp::HTuple& x, const HalconCpp::HTuple& y, const HalconCpp::HTuple& z, const SurfaceModelParams& params, HalconCpp::HTuple* model_handle)
{
	HalconCpp::HTuple object_handle;
	createObjectModelWithNormals(x, y, z, &object_handle);

	HalconCpp::HTuple gen_param_name, gen_param_value;
	parseCreationParameters(params, gen_param_name, gen_param_value);

	HalconCpp::CreateSurfaceModel(object_handle, HalconCpp::HTuple(params.rel_sampling_distance), gen_param_name, gen_param_value, model_handle);
	return true;
}



void SurfaceModelEstimator::createObjectModelWithNormals(const HalconCpp::HTuple& x, const HalconCpp::HTuple& y, const HalconCpp::HTuple& z, HalconCpp::HTuple* handle)
{
	HalconCpp::HTuple temp_handle;
	HalconCpp::GenObjectModel3dFromPoints(x, y, z, &temp_handle);

	HalconCpp::HTuple gen_param_name, gen_param_value;
	gen_param_name.Append("mls_kNN");
	gen_param_name.Append("mls_order");
	gen_param_value.Append(HTuple(60));	//default 60
	gen_param_value.Append(HTuple(2));	//default 2
	HalconCpp::SurfaceNormalsObjectModel3d(temp_handle, HalconCpp::HTuple("mls"), gen_param_name, gen_param_value, handle); //SurfaceNormalsObjectModel3d
}



void SurfaceModelEstimator::parseCreationParameters(const SurfaceModelParams& params, HalconCpp::HTuple& gen_param_name, HalconCpp::HTuple& gen_param_value)
{
	gen_param_name.Append("model_invert_normals");
	gen_param_name.Append("pose_ref_rel_sampling_distance");
	gen_param_name.Append("feat_step_size_rel");
	gen_param_name.Append("feat_angle_resolution");

	if(params.model_invert_normals == 1)
	{
		gen_param_value.Append(HTuple("true"));
	}
	else
	{
		gen_param_value.Append(HTuple("false"));
	}

	gen_param_value.Append(HTuple(params.pose_ref_rel_sampling_distance));
	gen_param_value.Append(HTuple(params.feat_step_size_rel));
	gen_param_value.Append(HTuple(params.feat_angle_resolution));
}



void SurfaceModelEstimator::parseDetectionParameters(const SurfaceModelParams& params, HalconCpp::HTuple& gen_param_name, HalconCpp::HTuple& gen_param_value)
{
	gen_param_name.Append("num_matches");
	gen_param_name.Append("max_overlap_dist_rel");
	gen_param_name.Append("sparse_pose_refinement");
	gen_param_name.Append("score_type");
	gen_param_name.Append("pose_ref_use_scene_normals");
	gen_param_name.Append("dense_pose_refinement");
	gen_param_name.Append("pose_ref_num_steps");
	gen_param_name.Append("pose_ref_sub_sampling");
	gen_param_name.Append("pose_ref_dist_threshold_rel");
	gen_param_name.Append("pose_ref_scoring_dist_rel");

	gen_param_value.Append(HTuple(params.num_matches));
	gen_param_value.Append(HTuple(params.max_overlap_dist_rel));

	if(params.sparse_pose_refinement == 1)
	{
		gen_param_value.Append(HTuple("true"));
	}
	else
	{
		gen_param_value.Append(HTuple("false"));
	}

	gen_param_value.Append(HTuple(params.score_type.c_str()));

	if(params.pose_ref_use_scene_normals == 1)
	{
		gen_param_value.Append(HTuple("true"));
	}
	else
	{
		gen_param_value.Append(HTuple("false"));
	}

	if(params.dense_pose_refinement == 1)
	{
		gen_param_value.Append(HTuple("true"));
	}
	else
	{
		gen_param_value.Append(HTuple("false"));
	}

	gen_param_value.Append(HTuple(params.pose_ref_num_steps));
	gen_param_value.Append(HTuple(params.pose_ref_sub_sampling));
	gen_param_value.Append(HTuple(params.pose_ref_dist_threshold_rel));
	gen_param_value.Append(HTuple(params.pose_ref_scoring_dist_rel));
}


} /* namespace vrm3dvision */
