/*
 * Registration.h
 *
 *  Created on: Aug 10, 2013
 *      Author: thomas
 */

#ifndef REGISTRATIONNODE_H_
#define REGISTRATIONNODE_H_

//#include "Logger/DTILogger.h"
#include "SharedData.hpp"
#include <modelling_engine/RosInterface.h>
#include <modelling_engine/ReconstructPointCloud.h>


// PCL specific includes
#include <pcl/common/transforms.h>
//#include <pcl/common/common_headers.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/point_representation.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/exceptions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/fpfh.h>

//statistical_outlier_removal filtering
#include <pcl/filters/statistical_outlier_removal.h>

//Downsampling using the voxel grid approach
#include <pcl/filters/voxel_grid.h>

//pcl registration
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ia_ransac.h>
//#include <pcl/registration/transforms.h>
#include <pcl/registration/gicp.h>

//Estimate normals
#include <pcl/features/normal_3d.h>

#include <robot_msgs/ModellingResult.h>

namespace modelling_engine {

class RegistrationNode {
public:
	RegistrationNode(boost::shared_ptr<SharedData> data_ptr, boost::shared_ptr<RosInterface> ros_ptr, bool removePlane, bool preProcess);
	virtual ~RegistrationNode();

	void start(void);
	void stop(void);
	void setState(ScanningStates state){_state = state;};

private:

	int _state;
	int _count;
	bool stop_thread;
	bool _doRemoveTable;
	bool _doPreProcessCloud;
	boost::shared_ptr<SharedData> _sharedData;
	boost::shared_ptr<RosInterface> _rosInterface;
	boost::shared_ptr<ReconstructPointCloud> _reconstruct;

	void transformCloud(pcl::PointCloud<PointT>::Ptr &cloud,pcl::PointCloud<PointT>::Ptr &tar, Eigen::Matrix4f& transform);
	pcl::PointCloud<PointT>::Ptr statisticalOutlierRemoval(pcl::PointCloud<PointT>::Ptr cloud, int mean);
	pcl::PointCloud<PointT>::Ptr DownSampleCloud(pcl::PointCloud<PointT>::Ptr cloud, double leaf_size);
	pcl::PointCloud<PointT>::Ptr PassThroughFilter(pcl::PointCloud<PointT>::Ptr cloud, double min_depth, double max_depth);
	bool Registre_views(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_cloud, Eigen::Matrix4f& transform);
	bool Registre_views_ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_cloud, Eigen::Matrix4f& transform);
	void initial_alignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_cloud, Eigen::Matrix4f &final_transform, bool downsample = true);
	
	pcl::PointCloud<pcl::Normal>::Ptr EstimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius);
	void EstimateNormals(pcl::PointCloud<PointT>::Ptr &src_cloud,pcl::PointCloud<pcl::PointNormal>::Ptr &target_cloud, double neighbors);
	void checkForNaN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
	void removePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_cloud);

};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


} /* namespace perception */
#endif /* REGISTRATIONNODE_H_ */
