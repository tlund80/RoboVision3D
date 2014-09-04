/********************************************************************************************************************
 *
 * \file                pose_estimation.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2014-02-04
 * \version             0.1
 * \brief
 *
*********************************************************************************************************************/
#include "pose_estimator.hpp"

namespace vrm3dvision {

	PoseEstimator::PoseEstimator()
	{
		load_all_models_ = false;
		halcon_available_ = SurfaceModelEstimator::isHalconLicenseAvailable();

		use_halcon_ = false;

		exposures_.push_back(1.0);
		exposures_.push_back(2.0);
		exposures_.push_back(4.0);
		exposures_.push_back(8.0);
		exposures_.push_back(16.0);
		exposures_.push_back(32.0);
		exposures_.push_back(64.0);
	}

	PoseEstimator::~PoseEstimator()
	{
		md_it_ = model_database_.begin();
		for(md_it_ = model_database_.begin(); md_it_ != model_database_.end(); md_it_++)
		{
			md_it_->second.saveViewScore();
		}
	}

	void PoseEstimator::initialize()
	{
		// Create publisher
		//

		int total_models = 0;
		if (load_all_models_)
		{
			total_models = loadAllModels();
		}

		ROS_INFO_STREAM("Pose estimation node initialized - Has Halcon? " << halcon_available_ << " - Number of Models loaded: " << total_models);
	}


	bool PoseEstimator::loadModel(const std::string& model)
	{
		bool ret = false;
		if (model_database_.count(model))
		{
			current_model_ptr_ = &model_database_[model];
			ret = true;

		}
		else
		{
			ObjectModel tmp_model;
			if (tmp_model.initFromModel(model))
			{
				ap_.prepareModel(tmp_model);
				if (halcon_available_)
				{
					sme_.prepareModel(tmp_model);
				}

				ROS_INFO_STREAM("Loaded Model from folder: " << tmp_model.getModelFolder() << " with name: " << tmp_model.getModelName());
				model_database_.insert(std::pair<std::string&, ObjectModel>(tmp_model.getModelName(), tmp_model));
				current_model_ptr_ = &model_database_[tmp_model.getModelName()];
				ret = true;
			}
			else
			{
				ROS_ERROR_STREAM("Unable to load model: " << model);
				ret = false;
			}
		}

		return ret;
	}



	int PoseEstimator::loadAllModels()
	{
		int total_models = 0;
		if( boost::filesystem::exists( ObjectModel::getPathToModels() ) )
		{
			boost::filesystem::directory_iterator iterator(ObjectModel::getPathToModels());
			for(; iterator != boost::filesystem::directory_iterator(); ++iterator)
			{
				if (boost::filesystem::is_directory(*iterator))
				{
					if (loadModel(iterator->path().string()))
					{
						total_models++;
					}
				}
			}
		}
		return total_models;
	}



	bool PoseEstimator::prepareModel(const std::string& model_name, const SurfaceModelParams& smp, const AlignmentPrerejectiveParams& app, const std::string& method)
	{
		bool ret = false;
		if (loadModel(model_name))
		{
			current_model_ptr_->updateParameters(smp, app);

			use_halcon_ = false;
			if (!method.empty() && halcon_available_)
			{
				if (method.at(0) == 'h' || method.at(0) == 'H')
				{
					use_halcon_ = true;
				}
			}
			ret = true;
			if(use_halcon_)
				ROS_INFO_STREAM("PoseEstimator prepared for \"" << model_name <<  "\" using Halcon method.");
			else
				ROS_INFO_STREAM("PoseEstimator prepared for \"" << model_name <<  "\" using PCL method.");
		}
		else
		{
			ROS_ERROR_STREAM("Unknown model! - unable to do pose estimation");
			ret = false;
		}
		return ret;
	}



	bool PoseEstimator::estimatePose(const pcl::PointCloud<pcl::PointXYZRGB>& scene)
	{
		ROS_INFO_STREAM("Starting Pose Estimation..");

		Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
		bool success = false;

		if(use_halcon_)
		{
			success = sme_.estimatePose(scene, *current_model_ptr_, best_transform);
		}
		else
		{
			success = ap_.estimatePose(scene, *current_model_ptr_, best_transform);
		}

		if(success)
		{
			ROS_INFO_STREAM("Successfully estimated pose of object");
			Eigen::Matrix4d tmp = best_transform.cast<double>();
			Eigen::Affine3d affine(tmp);
			tf::Transform transform;
			tf::transformEigenToTF(affine, transform_);
		}
		else
		{
			ROS_INFO_STREAM("Failed to estimate pose of \"" << current_model_ptr_->getModelName() << " in the scene.");
		}

		return success;
	}

	bool PoseEstimator::createNewModel(const std::string& model_name, const std::string& cad_path, const SurfaceModelParams& smp, const AlignmentPrerejectiveParams& app)
	{
		bool ret = false;
		ObjectModel new_model(0, 350, 0.35, 60);
		if(new_model.createNewModelFromCad(model_name, cad_path))
		{
			new_model.updateParameters(smp, app);
			ap_.prepareModel(new_model);

			if(halcon_available_)
			{
				sme_.prepareModel(new_model);
			}

			model_database_.insert(std::pair<std::string&, ObjectModel>(new_model.getModelName(), new_model));
			current_model_ptr_ = &model_database_[new_model.getModelName()];
			ROS_INFO_STREAM("Successfully created new model \"" << model_name << "\".");
			ret = true;
		}
		return ret;
	}



	std::string PoseEstimator::getExpoExplorationString()
	{
		std::stringstream ss;
		for(size_t i = 0; i < exposures_.size(); i++)
		{
			ss << exposures_[i];
			if(i < exposures_.size()-1)
			{
				ss << " ";
			}
		}
		return ss.str();
	}



	void PoseEstimator::addExistingEdges(const std::vector<std::map<int,bool> >& existing_edges)
	{
		existing_edge_maps_.push_back(existing_edges);
	}



	void PoseEstimator::computeBestExposure(const std::vector<std::map<int,bool> >& existing_edges, bool save_result)
	{
		addExistingEdges(existing_edges);

		std::map<int, bool>::iterator iter_i, iter_j;
		int object_edges = 0;
		int extra_edges = 0;
		int best_single_expo = 0;
		int best_double_expo = 0;
		int best_single_index = 0;
		int best_double_index_low = 0;
		int best_double_index_high = 0;

		int edges[existing_edge_maps_.size()][existing_edge_maps_.size()];

		for(size_t i = 0; i < existing_edge_maps_.size(); i++)
		{
			object_edges = 0;
			for(size_t row = 0; row < existing_edge_maps_.at(i).size(); row++)
			{
				for (iter_i = existing_edge_maps_.at(i).at(row).begin(); iter_i != existing_edge_maps_.at(i).at(row).end(); iter_i++)
				{
					if(iter_i->second)
					{
						object_edges++;
					}
				}
			}
			if(object_edges > best_single_expo)
			{
				best_single_expo = object_edges;
				best_single_index = i;
			}

			for(size_t j = i; j < existing_edge_maps_.size(); j++)
			{
				extra_edges = 0;
				for(size_t row = 0; row < existing_edge_maps_.at(j).size(); row++)
				{
					for (iter_j = existing_edge_maps_.at(j).at(row).begin(); iter_j != existing_edge_maps_.at(j).at(row).end(); iter_j++)
					{
						if(iter_j->second)
						{
							iter_i = existing_edge_maps_.at(i).at(row).find(iter_j->first);
							if(iter_i == existing_edge_maps_.at(i).at(row).end())
							{
								extra_edges++;
							}
						}
					}
				}
				if((object_edges + extra_edges) > best_double_expo)
				{
					best_double_expo = (object_edges + extra_edges);
					best_double_index_low = i;
					best_double_index_high = j;
				}
				edges[j][i] = (object_edges + extra_edges);
			}
		}

		for(size_t i = 0; i < existing_edge_maps_.size(); i++)
		{
			std::cout << std::setw(2) << exposures_[i] << "ms: ";

			for(size_t j = 0; j <= i; j++)
			{
				if(j == i)
					std::cout << std::setw(7) << std::setprecision(5) << ((double) edges[i][j]/best_single_expo);
				else
					std::cout << std::setw(7) << std::setprecision(5) << ((double) edges[i][j]/best_double_expo) << " | ";
			}
			std::cout << std::endl;
		}

		ROS_INFO_STREAM("Best single exposure: " << exposures_[best_single_index] << "ms with " << best_single_expo << " edges");
		ROS_INFO_STREAM("Best double exposure: " << exposures_[best_double_index_low] << "+" << exposures_[best_double_index_high] << "ms with " << best_double_expo << " edges");

		if(best_single_index > 0 && best_single_index < exposures_.size()-1)
		{
			float new_single = best_single_index + 0.5*(edges[best_single_index-1][best_single_index-1] - edges[best_single_index+1][best_single_index+1]) /
								(edges[best_single_index-1][best_single_index-1]-2*edges[best_single_index][best_single_index]+edges[best_single_index+1][best_single_index+1]);
			ROS_INFO_STREAM("Interpolated single exposure: " << pow(2.0,new_single) << "ms");

		}

		if(best_double_index_low > 0)
		{
			float new_low = best_double_index_low + 0.5*(edges[best_double_index_high][best_double_index_low-1] - edges[best_double_index_high][best_double_index_low+1]) /
					(edges[best_double_index_high][best_double_index_low-1]-2*edges[best_double_index_high][best_double_index_low]+edges[best_double_index_high][best_double_index_low+1]);
			ROS_INFO_STREAM("Interpolated exposure: Low: " << pow(2.0,new_low) << "ms");

		}
		if(best_double_index_high < exposures_.size()-1)
		{
			float new_high = best_double_index_high + 0.5*(edges[best_double_index_high-1][best_double_index_low] - edges[best_double_index_high+1][best_double_index_low]) /
					(edges[best_double_index_high-1][best_double_index_low]-2*edges[best_double_index_high][best_double_index_low]+edges[best_double_index_high+1][best_double_index_low]);
			ROS_INFO_STREAM("Interpolated exposure: High: " << pow(2.0,new_high) << "ms");

		}

		if(save_result)
		{
			current_model_ptr_->updateExposure(exposures_[best_single_index], best_single_expo, exposures_[best_double_index_low], exposures_[best_double_index_high], best_double_expo);
		}

		existing_edge_maps_.clear();
	}

} /* namespace vrm3dvision */
