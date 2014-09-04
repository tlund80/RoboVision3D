/********************************************************************************************************************
 *
 * \file                alignment_prerejective.hpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2014-02-04
 * \version             0.1
 * \brief
 *
*********************************************************************************************************************/
#ifndef _ALIGNMENT_PREREJECTIVE_HPP_
#define _ALIGNMENT_PREREJECTIVE_HPP_

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "sample_consensus_prerejective_jk.h"

#include "ros/ros.h"

#include <vrm3dvision/AlignmentPrerejectiveParams.h>

#include "object_model.hpp"

#include "omp.h"

namespace vrm3dvision {

	class AlignPrerejective
	{
		public:
			/// Constructor
			AlignPrerejective();
			~AlignPrerejective();

			bool estimatePose(const pcl::PointCloud<pcl::PointXYZRGB>& input_cloud, ObjectModel& model, Eigen::Matrix4f& t);

			bool prepareModel(ObjectModel& model);

		private:
			double estimatePoseView(Eigen::Matrix4f& transform, const Eigen::Matrix4f& pose, const pcl::PointCloud<pcl::PointNormal>& object, const pcl::PointCloud<pcl::FPFHSignature33>& object_features, pcl::SampleConsensusPrerejectiveJK<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33>* scp_pose_estimator, pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal>* icp);
			bool updateParameters(const AlignmentPrerejectiveParams& params);
			void constructFullModel(ObjectModel& model, const std::string& folder);
			pcl::VoxelGrid<pcl::PointNormal>* voxel_grid_;
			pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal >* normal_estimator_;
			pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33>* feature_estimator_;
			std::vector<pcl::SampleConsensusPrerejectiveJK<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33>*> scp_pose_estimator_;
			std::vector<pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal>*> icp_;

			double min_score_;
			double min_score_threshold_;

			int threads_;
	};

} /* namespace vrm3dvision */

#endif /* _ALIGNMENT_PREREJECTIVE_HPP_ */
