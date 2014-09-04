/********************************************************************************************************************
 *
 * \file                alignment_prerejective.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2014-02-04
 * \version             0.1
 * \brief
 *
*********************************************************************************************************************/
#include "alignment_prerejective.hpp"

namespace vrm3dvision {

	AlignPrerejective::AlignPrerejective()
	{
		normal_estimator_ = new pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal >();

		voxel_grid_ = new pcl::VoxelGrid<pcl::PointNormal>();
		voxel_grid_->setDownsampleAllData(true);

		feature_estimator_ = new pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33>();

		threads_ = 4;

		scp_pose_estimator_.resize(threads_);
		icp_.resize(threads_);
		for (int i = 0; i < threads_; i++)
		{
			scp_pose_estimator_[i] = new pcl::SampleConsensusPrerejectiveJK<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33>();
			scp_pose_estimator_[i]->setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
			icp_[i] = new pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal>();
		}


		min_score_ = 50;
		min_score_threshold_ = 60;
	}

	AlignPrerejective::~AlignPrerejective()
	{
		delete normal_estimator_;
		delete voxel_grid_;
		delete feature_estimator_;
		for (int i = 0; i < threads_; i++)
		{
			delete scp_pose_estimator_[i];
			delete icp_[i];
		}
	}

	bool AlignPrerejective::updateParameters(const AlignmentPrerejectiveParams& params)
	{
		double leaf_size = params.leaf_size;
		double normal_radius = params.normal_radius_ratio_leaf * leaf_size;
		double feature_radius = params.feature_radius_ratio_leaf * normal_radius;

		normal_estimator_->setRadiusSearch(normal_radius);
		//normal_estimator_.setKSearch(10);

		voxel_grid_->setLeafSize (leaf_size, leaf_size, leaf_size);

		feature_estimator_->setRadiusSearch(feature_radius);

		for (int i = 0; i < threads_; i++)
		{
			scp_pose_estimator_[i]->setCorrespondenceRandomness (params.correspondence_randomness); // Number of nearest features to use
			scp_pose_estimator_[i]->setSimilarityThreshold (params.similarity_threshold); // Polygonal edge length similarity threshold
			scp_pose_estimator_[i]->setMaximumIterations(params.max_iterations);
			scp_pose_estimator_[i]->setInlierFractionTermination(params.min_score_threshold);
			scp_pose_estimator_[i]->setMaxCorrespondenceDistance (params.max_correspondence_distance_ratio_leaf * leaf_size); // Set inlier threshold
			scp_pose_estimator_[i]->setInlierFraction (params.inlier_fraction); // Set required inlier fraction
			scp_pose_estimator_[i]->setInlierFractionTermination(params.min_score_threshold/100.0);

			icp_[i]->setMaxCorrespondenceDistance(params.icp_max_correspondence_distance_ratio_leaf * leaf_size);
			icp_[i]->setMaximumIterations(params.icp_max_iterations);
//			icp_[i]->setRANSACIterations(params.icp_max_iterations);
//			icp_[i]->setRANSACOutlierRejectionThreshold(params.icp_max_correspondence_distance_ratio_leaf * leaf_size);
		}

		min_score_ = params.min_score;
		min_score_threshold_ = params.min_score_threshold;

		return true;
	}

//	bool AlignPrerejective::estimatePose(const pcl::PointCloud<pcl::PointXYZRGB>& input_cloud, ObjectModel& model, Eigen::Matrix4f& t)
//	{
//		updateParameters(model.alignment_prerejective_params_);
//		pcl::PointCloud<pcl::PointNormal> scene;
//		pcl::copyPointCloud(input_cloud, scene);
//
//		bool ret = false;
//
//		double tm = ros::Time::now().toSec();
//		double tm_temp = ros::Time::now().toSec();
//
//		// Downsample
//		voxel_grid_->setInputCloud(scene.makeShared());
//		voxel_grid_->filter(scene);
//
//
//		ROS_INFO_STREAM("Downsampling took: " << (ros::Time::now().toSec()-tm_temp)*1000 << " ms - Size: " << scene.size());
//		tm_temp = ros::Time::now().toSec();
//
//		// Compute normals
//		normal_estimator_->setInputCloud(scene.makeShared());
//		normal_estimator_->compute(scene);
//
//		ROS_INFO_STREAM("Calculated Normals - it took: " << (ros::Time::now().toSec()-tm_temp)*1000 << " ms - Size: " << scene.size());
//		tm_temp = ros::Time::now().toSec();
//
//		// Calculate features
//		pcl::PointCloud<pcl::FPFHSignature33> feature_cloud;
//		feature_estimator_->setInputCloud(scene.makeShared());
//		feature_estimator_->setInputNormals(scene.makeShared());
//		feature_estimator_->compute(feature_cloud);
//
//		ROS_INFO_STREAM("Calculating features took: " << (ros::Time::now().toSec()-tm_temp)*1000 << " ms");
//		tm_temp = ros::Time::now().toSec();
//
//		Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
//		double best_score = -1;
//		int best_index = -1;
//
//		for (int i = 0; i < threads_; i++)
//		{
//			scp_pose_estimator_[i]->setInputTarget (scene.makeShared());
//			scp_pose_estimator_[i]->setTargetFeatures (feature_cloud.makeShared());
//			icp_[i]->setInputTarget(scene.makeShared());
//		}
//
//
//		Eigen::Matrix4f transformation;
//		Eigen::Matrix4f ident = Eigen::Matrix4f::Identity();
//		double time = ros::Time::now().toSec();
//		double score = estimatePoseView(transformation, ident , *model.ap_full_cloud_, *model.feature_cloud_full_ , scp_pose_estimator_[0], icp_[0]);
//		//ROS_INFO_STREAM("Pose estimation took: " << (ros::Time::now().toSec()-time)*1000 << " ms for view: " << indexes[i]);
//		//std::cout << "." << std::cout.flush();
//		if (score > min_score_)
//		{
//			best_transform = transformation;
//			best_score = score;
//		}
//
//		if (best_score > min_score_)
//		{
//			model.addSuccededView(best_index);
//
//			t = best_transform;
//
//			ret = true;
//			ROS_INFO("Final transform");
//			ROS_INFO ("    | %6.3f %6.3f %6.3f |", t (0,0), t (0,1), t (0,2));
//			ROS_INFO ("R = | %6.3f %6.3f %6.3f |", t (1,0), t (1,1), t (1,2));
//			ROS_INFO ("    | %6.3f %6.3f %6.3f |", t (2,0), t (2,1), t (2,2));
//			ROS_INFO ("t = < %0.5f, %0.5f, %0.5f >", t (0,3), t (1,3), t (2,3));
//			ROS_INFO ("Final Score: %f", best_score);
//			double total_time = (ros::Time::now().toSec()-tm)*1000;
//			ROS_INFO_STREAM("Total Pose estimation took: " << total_time << " ms - Succeded using view id: " << best_index << " with Index score: " << model.getFitness(best_index));
//
//		}
//		return ret;
//	}

	bool AlignPrerejective::estimatePose(const pcl::PointCloud<pcl::PointXYZRGB>& input_cloud, ObjectModel& model, Eigen::Matrix4f& t)
	{
		updateParameters(model.alignment_prerejective_params_);
		pcl::PointCloud<pcl::PointNormal> scene;
		pcl::copyPointCloud(input_cloud, scene);

		bool ret = false;

		double tm = ros::Time::now().toSec();
		double tm_temp = ros::Time::now().toSec();

		const double max_downscale_factor = 0.25;

		int max_view_size = 0;
		int avg_view_size = 0;
		int min_view_size = std::numeric_limits<int>::max();

		for (int i = 0; i < model.getNumViews(); i++)
		{
			avg_view_size += model.ap_clouds_[i]->size();
			if (model.ap_clouds_[i]->size() > max_view_size)
				max_view_size = model.ap_clouds_[i]->size();
			if (model.ap_clouds_[i]->size() < min_view_size)
				min_view_size = model.ap_clouds_[i]->size();
		}
		avg_view_size /= model.getNumViews();

		//ROS_WARN_STREAM("Min: " << min_view_size <<  " Avg: " << avg_view_size << " Max: " << max_view_size);

		// Downsample
		voxel_grid_->setInputCloud(scene.makeShared());
		voxel_grid_->filter(scene);


		//ROS_INFO_STREAM("Downsampling took: " << (ros::Time::now().toSec()-tm_temp)*1000 << " ms - Size: " << scene.size());
		tm_temp = ros::Time::now().toSec();

		// Compute normals
		normal_estimator_->setInputCloud(scene.makeShared());
		normal_estimator_->compute(scene);

		//ROS_INFO_STREAM("Calculated Normals - it took: " << (ros::Time::now().toSec()-tm_temp)*1000 << " ms - Size: " << scene.size());
		tm_temp = ros::Time::now().toSec();

		// Calculate features
		pcl::PointCloud<pcl::FPFHSignature33> feature_cloud;
		feature_estimator_->setInputCloud(scene.makeShared());
		feature_estimator_->setInputNormals(scene.makeShared());
		feature_estimator_->compute(feature_cloud);

		//ROS_INFO_STREAM("Calculating features took: " << (ros::Time::now().toSec()-tm_temp)*1000 << " ms");
		tm_temp = ros::Time::now().toSec();

		Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
		double best_score = -1;
		int best_index = -1;

		for (int i = 0; i < threads_; i++)
		{
			scp_pose_estimator_[i]->setInputTarget (scene.makeShared());
			scp_pose_estimator_[i]->setTargetFeatures (feature_cloud.makeShared());
			icp_[i]->setInputTarget(scene.makeShared());
		}

		std::vector<int> indexes = model.getViewOrder();

		omp_set_dynamic(0);     // Explicitly disable dynamic teams
		omp_set_num_threads(threads_); // Use 4 threads for all consecutive parallel regions
		volatile bool success = false;
		#pragma omp parallel for shared(success)
		for (size_t i = 0; i < indexes.size(); i++)
		{
			if (success)
			{
				continue;
			}

			Eigen::Matrix4f transformation;
			double time = ros::Time::now().toSec();
			double score = estimatePoseView(transformation, model.poses_[indexes[i]] , *model.ap_clouds_[indexes[i]], *model.feature_clouds_[indexes[i]], scp_pose_estimator_[omp_get_thread_num()], icp_[omp_get_thread_num()]);
			// Correct for different view sizes - make large views count more
			double new_score = score;
			if (score > 0.0)
			{
				new_score = score * (1.0 - (double(max_view_size - model.ap_clouds_[indexes[i]]->size()) / (double(max_view_size - min_view_size)) * max_downscale_factor));
			}
			//ROS_ERROR_STREAM("Old score: " << score << " New score: " << new_score << " Size: " << model.ap_clouds_[indexes[i]]->size() << " Max size: " << max_view_size);
			score = new_score;
			//ROS_INFO_STREAM("Pose estimation took: " << (ros::Time::now().toSec()-time)*1000 << " ms for view: " << indexes[i]);
			//std::cout << "." << std::cout.flush();
			#pragma omp critical
			{
				if (score > best_score)
				{
					ROS_INFO_STREAM("Best Pose Estimate updated from view: " << indexes[i] << " with score: " << score);
					best_transform = transformation;
					best_score = score;
					best_index = indexes[i];
					if (best_score > min_score_threshold_)
					{
						success = true;
					}
				}
			}
		}

		if (success || best_score > min_score_)
		{
			// Do ICP using full model
			icp_[0]->setInputSource(model.ap_full_cloud_);
			pcl::PointCloud<pcl::PointNormal> Final;
			icp_[0]->align(Final, best_transform);

			if (icp_[0]->hasConverged())
			{
//				t = best_transform;
				t = icp_[0]->getFinalTransformation();

				ROS_INFO("Final transform");
				ROS_INFO ("    | %6.3f %6.3f %6.3f |", t (0,0), t (0,1), t (0,2));
				ROS_INFO ("R = | %6.3f %6.3f %6.3f |", t (1,0), t (1,1), t (1,2));
				ROS_INFO ("    | %6.3f %6.3f %6.3f |", t (2,0), t (2,1), t (2,2));
				ROS_INFO ("t = < %0.5f, %0.5f, %0.5f >", t (0,3), t (1,3), t (2,3));
				ROS_INFO ("Final Score: %f", best_score);
				double total_time = (ros::Time::now().toSec()-tm)*1000;
				ROS_INFO_STREAM("Total Pose estimation took: " << total_time << " ms - Succeded using view id: " << best_index << " with Index score: " << model.getFitness(best_index));

				ret = true;

				model.addSuccededView(best_index);
			}
		}
		return ret;
	}

	double AlignPrerejective::estimatePoseView(Eigen::Matrix4f& transform, const Eigen::Matrix4f& pose, const pcl::PointCloud<pcl::PointNormal>& object, const pcl::PointCloud<pcl::FPFHSignature33>& object_features, pcl::SampleConsensusPrerejectiveJK<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33>* scp_pose_estimator, pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal>* icp)
	{
		double score = -1;
		// Perform alignment
		scp_pose_estimator->setInputSource (object.makeShared());
		scp_pose_estimator->setSourceFeatures (object_features.makeShared());

		// Do alignment
		pcl::PointCloud<pcl::PointNormal> object_aligned;
		scp_pose_estimator->align (object_aligned);

		if (scp_pose_estimator->hasConverged ())
		{
			pcl::PointCloud<pcl::PointNormal> Final;
			icp->setInputSource(object.makeShared());
			icp->align(Final, scp_pose_estimator->getFinalTransformation ());

			if (icp->hasConverged())
			{
				// Calculate score by rechecking inliers after ICP
				score = 100.0 * (double(scp_pose_estimator->recalculateInliers(icp->getFinalTransformation())) / double(object.size()));

				// Calculate final transform by multiplying the current pose to the result from the ICP
				transform = icp->getFinalTransformation() * pose;
			}


		}
		return score;
	}

	bool AlignPrerejective::prepareModel(ObjectModel& model)
	{
		updateParameters(model.alignment_prerejective_params_);
		std::string ap_folder = model.getModelFolder() + "/ap";

		if(boost::filesystem::exists(ap_folder)) // Model already prepared - load data
		{
			model.ap_clouds_.clear();
			model.feature_clouds_.clear();

			for(int i = 0; i < model.getNumViews(); i++)
			{
				std::stringstream ss;
				ss << ap_folder << "/view_" << i << ".pcd";
				pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
				pcl::io::loadPCDFile(ss.str(), *cloud);
				model.ap_clouds_.push_back(cloud);

				ss.str("");
				ss << ap_folder << "/feature_" << i << ".pcd";
				pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>());
				pcl::io::loadPCDFile(ss.str(), *features);
				model.feature_clouds_.push_back(features);
			}

			std::stringstream ss;
			ss << ap_folder << "/full_cloud" << ".pcd";
			pcl::PointCloud<pcl::PointNormal> cloud;
			pcl::io::loadPCDFile(ss.str(), cloud);
			model.ap_full_cloud_ = cloud.makeShared();

			ss.str("");
			ss << ap_folder << "/full_feature" << ".pcd";
			pcl::PointCloud<pcl::FPFHSignature33> features;
			pcl::io::loadPCDFile(ss.str(), features);
			model.feature_cloud_full_ = features.makeShared();

		}
		else // Model not prepared - calculate needed data
		{
			ROS_INFO_STREAM("Preparing model for use with Prerejective alignment pose estimation..");
			if (boost::filesystem::create_directories(ap_folder))
			{
				model.ap_clouds_.clear();
				model.feature_clouds_.clear();

				model.loadRawClouds();
				for(int i = 0; i < model.getNumViews(); i++)
				{
					// Compute normals
					pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
					pcl::copyPointCloud(*model.raw_clouds_[i], *cloud);

					int size = cloud->size();
					// Remove outliers
					pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
					sor.setInputCloud(cloud);
					sor.setMeanK(20);
					sor.setStddevMulThresh(1.5);
					//sor.setNegative (true); // To view outliers
					sor.filter(*cloud);

//					ROS_INFO_STREAM("Removed outliers in view: " << i << " - removed " << cloud->size() << "/" << size);

					// Downsample
					voxel_grid_->setInputCloud(cloud);
					voxel_grid_->filter(*cloud);

					normal_estimator_->setInputCloud(cloud);
					normal_estimator_->compute(*cloud);

					// Calculate features
					pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>());
					feature_estimator_->setInputCloud(cloud);
					feature_estimator_->setInputNormals(cloud);
					feature_estimator_->compute(*features);

					model.ap_clouds_.push_back(cloud);
					model.feature_clouds_.push_back(features);

					// Save files
					std::stringstream ss;
					ss << ap_folder << "/view_" << i << ".pcd";
					pcl::io::savePCDFileBinary(ss.str(), *cloud);
					ss.str("");
					ss << ap_folder << "/feature_" << i << ".pcd";
					pcl::io::savePCDFileBinary(ss.str(), *features);
				}

				constructFullModel(model, ap_folder);
			}
			else
			{
				ROS_WARN_STREAM("Failed to create folder for Alignment Prerejection data.");
				return false;
			}
		}

		return true;
	}

	void AlignPrerejective::constructFullModel(ObjectModel& model, const std::string& folder)
	{
		// Construct full model
		std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> aligned_clouds;

		for (size_t i = 0; i < model.ap_clouds_.size(); i++)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal> ());
			Eigen::Matrix4f pose_inverse;
			pose_inverse = model.poses_[i].inverse ();
			pcl::transformPointCloudWithNormals ( *model.ap_clouds_[i], *cloud, pose_inverse);
			aligned_clouds.push_back (cloud);
		}

		// Fuse clouds
		pcl::PointCloud<pcl::PointNormal>::Ptr full_cloud (new pcl::PointCloud<pcl::PointNormal> ());
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr full_feature_cloud(new pcl::PointCloud<pcl::FPFHSignature33>());

		for (size_t i = 0; i < aligned_clouds.size (); i++)
		{
			*full_cloud += *aligned_clouds[i];
			*full_feature_cloud += *model.feature_clouds_[i];
		}

		// Remove outliers
		int size = full_cloud->size();
		pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
		sor.setInputCloud(full_cloud);
		sor.setMeanK(20);
		sor.setStddevMulThresh(1.5);
		//sor.setNegative (true); // To view outliers
		sor.filter(*full_cloud);

//		ROS_INFO_STREAM("Removed outliers from full cloud - removed " << full_cloud->size() << "/" << size);


		// Downsample pointcloud
		pcl::PointCloud<pcl::PointNormal> full_cloud_downsampled;
		pcl::PointCloud<int> indices;
		pcl::UniformSampling<pcl::PointNormal> uniform_sampling;
		uniform_sampling.setInputCloud (full_cloud);
		uniform_sampling.setRadiusSearch((double)voxel_grid_->getLeafSize()(0));
		uniform_sampling.compute(indices);
		pcl::copyPointCloud(*full_cloud, indices.points, full_cloud_downsampled);
		model.ap_full_cloud_ = full_cloud_downsampled.makeShared();

		// Extract corresponding features
		pcl::PointCloud<pcl::FPFHSignature33> full_feature_cloud_downsampled;
		pcl::copyPointCloud(*full_feature_cloud, indices.points, full_feature_cloud_downsampled);
		model.feature_cloud_full_ = full_feature_cloud_downsampled.makeShared();

		// Save files
		std::stringstream ss;
		ss << folder << "/full_cloud" << ".pcd";
		pcl::io::savePCDFileBinary(ss.str(), *model.ap_full_cloud_);
		ss.str("");
		ss << folder << "/full_feature" << ".pcd";
		pcl::io::savePCDFileBinary(ss.str(), *model.feature_cloud_full_);
	}

} /* namespace vrm3dvision */
