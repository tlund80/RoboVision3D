/********************************************************************************************************************
 *
 * \file                vrm_image_proc.hpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-22
 * \version             0.1
 * \brief               image handler for VRmagic D3 camera module
 *
*********************************************************************************************************************/
#ifndef _VRM_IMAGE_PROC_HPP_
#define _VRM_IMAGE_PROC_HPP_

// ROS includes
#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <vrm3dvision/acquirePoseEstimate.h>
#include <vrm3dvision/acquireSequenceOnRobot.h>
#include <vrm3dvision/computeBestExposure.h>
#include <vrm3dvision/createNewModel.h>
#include <vrm3dvision/setExposure.h>
#include <vrm3dvision/triggerCamera.h>
#include <vrm3dvision/saveNextSequence.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

// ZeroMQ
#include <zmq.h>

// VRM communication protocol
#include <vrm_protocol/pubsub.hpp>
#include <vrm_protocol/image_group_msg.hpp>
#include <vrm_protocol/vrm_cmd_msg.hpp>
#include <vrm_protocol/vrm_log_msg.hpp>
#include <vrm_global.h>

// Local includes
#include "structured_light_reconstruction.hpp"
#include "../pose_estimation/pose_estimator.hpp"

// Standard libraries
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

// PCL
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// BOOST
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

struct camera_handler {
	image_transport::CameraPublisher publisher;
	camera_info_manager::CameraInfoManager* info_manager;
	ros::NodeHandle node_handle;
	image_transport::ImageTransport* image_transporter;
};

namespace vrm3dvision {

	/** @class vrm3dvision Node
	  * VRM D3 communication/handler class for ROS
	  */
	class VrmImageProc
	{
		public:
			/// Constructor
			VrmImageProc(const ros::NodeHandle& node_handle);
			void initialize();
			void mainLoop();

		private:
			// ROS variables
			ros::NodeHandle nh_;

			// ROS services
			ros::ServiceServer acquirePoseEstimateSrv_;
			ros::ServiceServer createNewModelSrv_;
			ros::ServiceServer saveNextSequenceSrv_;
			ros::ServiceClient request_point_cloud_srv_;
			ros::ServiceClient set_exposure_srv_;

			// ROS publishers
			ros::Publisher point_cloud_publisher_;
			ros::Publisher point_cloud_trimmed_publisher_;
			ros::Publisher model_mesh_publisher_;
			tf::TransformBroadcaster transformBroadcaster_;

			bool publish_trimmed_point_cloud_;
			double z_trim_distance_;

			// Post processing variables;
			bool remove_outliers_;
			bool remove_dominant_plane_;
			int mean_k_;
			int certainty_type_;
			double std_dev_thresh_;

			// ZeroMQ variables
			vrm_protocol::pubsub_client<vrm_protocol::image_group>* vrm_image_client_;

			std::string vrm_board_ip_;

			// Camera variables
			bool has_left_camera_;
			bool has_right_camera_;
			bool has_color_camera_;

			std::string stereo_camera_topic_;
			std::string color_camera_topic_;
			std::string point_cloud_pub_topic_;
			std::string point_cloud_trimmed_pub_topic_;
			std::string calibration_path_;
			std::string left_camera_name_;
			std::string right_camera_name_;
			std::string color_camera_name_;
			std::string left_camera_calibration_url_;
			std::string right_camera_calibration_url_;
			std::string color_camera_calibration_url_;
			std::string slr_pc_color_option_;

			camera_handler left_camera_;
			camera_handler right_camera_;
			camera_handler color_camera_;

			bool visualization_;

			StructuredLightReconstruction SLR_;
			PoseEstimator pe_;

			bool compute_best_exposure_;
			bool save_best_exposure_;
			bool estimate_object_pose_;

			double time_;

			// Functions
			void publishImages(const vrm_protocol::image_group& ig);
			void savePointCloudToPCD(const pcl::PointCloud<pcl::PointXYZRGB>& pc, const std::string& path, const std::string& filename);
			sensor_msgs::PointCloud trimPointCloud(const sensor_msgs::PointCloud& pc_in, double distance_z);
			void publishPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& pc_in);

			bool acquirePoseEstimateSrv(vrm3dvision::acquirePoseEstimate::Request &req, vrm3dvision::acquirePoseEstimate::Response &res);
			bool acquireSequenceOnRobot(vrm3dvision::acquireSequenceOnRobot::Request &req, vrm3dvision::acquireSequenceOnRobot::Response &res);
			bool computeBestExposureSrv(vrm3dvision::computeBestExposure::Request &req, vrm3dvision::computeBestExposure::Response &res);
			bool createNewModelSrv(vrm3dvision::createNewModel::Request &req, vrm3dvision::createNewModel::Response &res);
			bool saveNextSequenceSrv(vrm3dvision::saveNextSequence::Request &req, vrm3dvision::saveNextSequence::Response &res);
	};

} /* namespace vrm3dvision */

#endif /* _VRM_IMAGE_PROC_HPP_ */
