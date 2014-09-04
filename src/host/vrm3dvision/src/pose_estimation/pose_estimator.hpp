	/********************************************************************************************************************
 *
 * \file                pose_estimation.hpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2014-02-04
 * \version             0.1
 * \brief
 *
*********************************************************************************************************************/
#ifndef _POSE_ESTIMATION_HPP_
#define _POSE_ESTIMATION_HPP_

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_msgs/PolygonMesh.h>

#include <vrm3dvision/AlignmentPrerejectiveParams.h>
#include <vrm3dvision/SurfaceModelParams.h>

// Standard libraries
#include <string>
#include <vector>

// PCL
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/vtk_lib_io.h> // load polygon files
#include <pcl/PolygonMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

// BOOST
#define BOOST_NO_SCOPED_ENUMS
#include <boost/filesystem.hpp>

// Local includes
#include "object_model.hpp"
#include "surface_model_estimator.hpp"
#include "alignment_prerejective.hpp"

namespace vrm3dvision {

	class PoseEstimator
	{
		public:
			/// Constructor
			PoseEstimator();
			~PoseEstimator();

			void initialize();
			bool createNewModel(const std::string& model_name, const std::string& cad_path, const SurfaceModelParams& smp, const AlignmentPrerejectiveParams& app);
			bool prepareModel(const std::string& model_name, const SurfaceModelParams& smp, const AlignmentPrerejectiveParams& app, const std::string& method);
			bool estimatePose(const pcl::PointCloud<pcl::PointXYZRGB>& scene);
			bool loadModel(const std::string& model);
			inline tf::Transform getTransform() { return transform_; };
			inline pcl::PolygonMesh getModelMesh() { return current_model_ptr_->getModelMesh(); };
			void addExistingEdges(const std::vector<std::map<int,bool> >& existing_edges);
			void computeBestExposure(const std::vector<std::map<int,bool> >& existing_edges, bool save_result = false);
			double getSingleExpo() { return current_model_ptr_->getSingleExpo(); };
			double getDoubleExpoLow() { return current_model_ptr_->getDoubleExpoLow(); };
			double getDoubleExpoHigh() { return current_model_ptr_->getDoubleExpoHigh(); };
			std::string getBestExpoString() { return current_model_ptr_->getBestExpoString(); };
			std::string getSingleExpoString() { return current_model_ptr_->getSingleExpoString(); };
			std::string getDoubleExpoString() { return current_model_ptr_->getDoubleExpoString(); };
			std::string getExpoExplorationString();


		private:
			std::string camera_frame_;
			std::string object_frame_;
			std::string model_mesh_publisher_topic_;

			bool load_all_models_;
			bool halcon_available_;
			bool use_halcon_;

			int pose_estimate_queue_;

			std::map<std::string, ObjectModel> model_database_;
			std::map<std::string, ObjectModel>::iterator md_it_;

			ObjectModel* current_model_ptr_;
			SurfaceModelParams smp_;
			SurfaceModelEstimator sme_;
			AlignmentPrerejectiveParams app_;
			AlignPrerejective ap_;

			tf::Transform transform_;

			std::vector<double> exposures_;
			std::vector<std::vector<std::map<int,bool> > > existing_edge_maps_;

			int loadAllModels();

	};

} /* namespace vrm3dvision */

#endif /* _POSE_ESTIMATION_HPP_ */
