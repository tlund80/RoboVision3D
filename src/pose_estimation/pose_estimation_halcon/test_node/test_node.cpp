/*
 * test_node.cpp
 *
 *  Created on: Oct 1, 2013
 *      Author: thomas
 */
// ROS
#include <ros/package.h>
#include <ros/ros.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// Services
#include <pose_estimation_halcon/estimate.h>
#include <pose_estimation_halcon/prepareEstimation.h>

typedef sensor_msgs::PointCloud2 SensorCloudT;
typedef boost::shared_ptr<SensorCloudT> SensorCloudPtrT;

int main(int argc, char** argv) {
   // Initialize
   ros::init(argc, argv, "test_pose_estimation_halcon");
   ros::NodeHandle n("~");

   ROS_INFO("Loading model from %s", argv[1]); 
   SensorCloudPtrT model(new SensorCloudT);
   //sensor_msgs::PointCloud2 model;
   if(pcl::io::loadPCDFile(argv[1], *model) < 0) {
         ROS_ERROR("Failed to load test data!");
      return 1;
   }

   // Subscribe to prepare estimation service to create a surface model
   ROS_INFO("Subscribing to prepare pose estimation service...");
   ros::service::waitForService("/pose_estimation_halcon/prepare");
   ros::ServiceClient prepare = n.serviceClient<pose_estimation_halcon::prepareEstimation>("/pose_estimation_halcon/prepare");

   std::string surface_model_path ="/home/thomas/dti_co_worker/trunk/dti_co_worker_ws/src/dti_vision_3d/pose_estimation_halcon/data/surfaceModel.sfm"; //"/home/thomas/dti_co_worker/trunk/dti_co_worker/3D_Vision/pose_estimation_halcon/data/surfaceModel";
   pose_estimation_halcon::prepareEstimation pMsg;
   pMsg.request.model = *model;
   pMsg.request.surface_model_path = surface_model_path;
   pMsg.request.RelSamplingDistance = 0.05;
   pMsg.request.model_name = "salt";

  if(!prepare.call(pMsg)){
	  ROS_ERROR("Something went wrong when calling prepare pose estimation service");
	  return 1;
  }
	  std::string model_id = pMsg.response.model_id;
	  ROS_INFO("Model id: %s", model_id.c_str());


   // Subscribe to global estimation service
   ROS_INFO("Subscribing to halcon pose estimation service...");
   ros::service::waitForService("/pose_estimation_halcon/estimate");
   ros::ServiceClient estimate = n.serviceClient<pose_estimation_halcon::estimate>("/pose_estimation_halcon/estimate");

   pose_estimation_halcon::estimate eMsg;
   eMsg.request.RelSamplingDistance = 0.6;
   eMsg.request.table_top = true;
   eMsg.request.leaf_size = 0.005f;
   eMsg.request.surface_model_path = surface_model_path;
   eMsg.request.model_id = model_id;


   if(!estimate.call(eMsg)){
   	  ROS_ERROR("Something went wrong when calling pose estimation service");
   	  return 1;
    }


   return 0;
}
