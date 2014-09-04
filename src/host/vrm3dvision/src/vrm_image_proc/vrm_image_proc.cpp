/********************************************************************************************************************
 *
 * \file                vrm_image_proc.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-22
 * \version             0.1
 * \brief               image handler for VRmagic D3 camera module
 *
*********************************************************************************************************************/
#include "vrm_image_proc.hpp"

namespace vrm3dvision {

	VrmImageProc::VrmImageProc(const ros::NodeHandle& nodeHandle)
	 : nh_(nodeHandle),
	   compute_best_exposure_(false),
	   estimate_object_pose_(false)
	{
		// Read parameter from launch file
		ros::NodeHandle local_nh("~");
		local_nh.param<std::string>("vrm_board_ip", vrm_board_ip_, "192.168.50.10");
		local_nh.param<std::string>("stereo_camera_topic", stereo_camera_topic_, "vrm_stereo_camera");
		local_nh.param<std::string>("color_cam_topic", color_camera_topic_, "color_cam");
		local_nh.param<std::string>("point_cloud_pub_topic", point_cloud_pub_topic_, "point_cloud");
		local_nh.param<std::string>("point_cloud_trimmed_pub_topic", point_cloud_trimmed_pub_topic_, "point_cloud_trimmed");
		local_nh.param<std::string>("calibration_path", calibration_path_, "");
		local_nh.param<std::string>("left_camera_name", left_camera_name_, "VrmD3LeftCamera");
		local_nh.param<std::string>("right_camera_name", right_camera_name_, "VrmD3RightCamera");
		local_nh.param<std::string>("color_camera_name", color_camera_name_, "VrmD3ColorCamera");
		local_nh.param<bool>("has_left_camera", has_left_camera_, true);
		local_nh.param<bool>("has_right_camera", has_right_camera_, true);
		local_nh.param<bool>("has_color_camera", has_color_camera_, false);
		local_nh.param<bool>("visualization", visualization_, false);
		local_nh.param<bool>("publish_trimmed_point_cloud", publish_trimmed_point_cloud_, true);
		local_nh.param<double>("z_trim_distance", z_trim_distance_, 0.37);
		local_nh.param<std::string>("pc_color_option", slr_pc_color_option_, "none");
		local_nh.param<bool>("remove_outliers", remove_outliers_, true);
		local_nh.param<bool>("remove_dominant_plane", remove_dominant_plane_, true);
		local_nh.param<int>("mean_k", mean_k_, 50);
		local_nh.param<int>("certainty_type", certainty_type_, 0);
		local_nh.param<double>("std_dev_thresh", std_dev_thresh_, 5.0);

		// Fix urls if default is not used
		if (local_nh.hasParam("calibration_path"))
		{
			if (calibration_path_.at(calibration_path_.size() - 1) != '/')
			{
				calibration_path_.push_back('/');
			}
			if (calibration_path_.at(0) != '/')
			{
				calibration_path_ = ros::package::getPath("vrm3dvision") + std::string("/") + calibration_path_;
			}
		}
		else
		{
			std::stringstream ss;
			ss << getenv("HOME") << "/.ros/camera_info/";
			calibration_path_ = ss.str();
		}
		left_camera_calibration_url_ = std::string("file://") + calibration_path_ + left_camera_name_ + std::string(".yaml");
		right_camera_calibration_url_ = std::string("file://") + calibration_path_ + right_camera_name_ + std::string(".yaml");
		color_camera_calibration_url_ = std::string("file://") + calibration_path_ + color_camera_name_ +  std::string(".yaml");
		vrm_image_client_ = new vrm_protocol::pubsub_client<vrm_protocol::image_group>();
	}

	void VrmImageProc::initialize()
	{
		std::string vrm_image_server_address = "tcp://" + vrm_board_ip_ + ":" + std::string(VRM_IMAGE_SERVER_PORT);
		ROS_INFO("Initializing image server with address: %s", vrm_image_server_address.c_str());
		vrm_image_client_->startup(vrm_image_server_address);

		if (has_left_camera_)
		{
			left_camera_.node_handle = ros::NodeHandle(nh_,nh_.getNamespace() + "/left");
			left_camera_.image_transporter = new image_transport::ImageTransport(left_camera_.node_handle);
			left_camera_.publisher = image_transport::CameraPublisher(left_camera_.image_transporter->advertiseCamera("image_raw",1));
			left_camera_.info_manager = new camera_info_manager::CameraInfoManager(left_camera_.node_handle,left_camera_name_.c_str(), left_camera_calibration_url_.c_str());
		}
		if (has_right_camera_)
		{
			right_camera_.node_handle = ros::NodeHandle(nh_,nh_.getNamespace() + "/right");
			right_camera_.image_transporter = new image_transport::ImageTransport(right_camera_.node_handle);
			right_camera_.publisher = image_transport::CameraPublisher(right_camera_.image_transporter->advertiseCamera("image_raw",1));
			right_camera_.info_manager = new camera_info_manager::CameraInfoManager(right_camera_.node_handle,right_camera_name_.c_str(), right_camera_calibration_url_.c_str());
		}
		if (has_color_camera_)
		{
			color_camera_.node_handle = ros::NodeHandle(nh_,nh_.getNamespace() + "/color_cam");
			color_camera_.image_transporter = new image_transport::ImageTransport(color_camera_.node_handle);
			color_camera_.publisher = image_transport::CameraPublisher(color_camera_.image_transporter->advertiseCamera("image_raw",1));
			color_camera_.info_manager = new camera_info_manager::CameraInfoManager(color_camera_.node_handle,color_camera_name_.c_str(), color_camera_calibration_url_.c_str());
		}
		// Init structured light reconstruction
		if (has_left_camera_ && has_right_camera_ && has_color_camera_)
		{
			std::vector<std::string> calib_names;
			calib_names.push_back(left_camera_name_);
			calib_names.push_back(right_camera_name_);
			calib_names.push_back(color_camera_name_);
			calib_names.push_back("vrm3dcalib");

			SLR_.initialize(calibration_path_, calib_names, stereo_camera_topic_, visualization_, slr_pc_color_option_, remove_outliers_, remove_dominant_plane_, std_dev_thresh_, mean_k_, certainty_type_);
			point_cloud_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(point_cloud_pub_topic_.c_str(), 1);
			if (publish_trimmed_point_cloud_)
				point_cloud_trimmed_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(point_cloud_trimmed_pub_topic_.c_str(), 1);
			// init save path to a default place in case random dot pattern images is saved before savenextsequence service is called
		}

		model_mesh_publisher_ = nh_.advertise<pcl_msgs::PolygonMesh>("model_mesh", 1);
		request_point_cloud_srv_ = nh_.serviceClient<vrm3dvision::triggerCamera>("triggerCamera");
		set_exposure_srv_ = nh_.serviceClient<vrm3dvision::setExposure>("setExposure");

		// Advertise services
		acquirePoseEstimateSrv_ = nh_.advertiseService("acquire_pose_estimate", &VrmImageProc::acquirePoseEstimateSrv, this);
		createNewModelSrv_ = nh_.advertiseService("create_new_model", &VrmImageProc::createNewModelSrv, this);
		saveNextSequenceSrv_ = nh_.advertiseService("saveNextSequence", &VrmImageProc::saveNextSequenceSrv, this);

		pe_.initialize();

	}


	bool VrmImageProc::acquirePoseEstimateSrv(vrm3dvision::acquirePoseEstimate::Request &req, vrm3dvision::acquirePoseEstimate::Response &res)
	{
		res.success = pe_.prepareModel(req.model, req.smp, req.app, req.method);

		if(res.success)
		{
			setExposure::Request req_expo;
			if(req.exposure.empty())
			{
				req_expo.exposure = pe_.getBestExpoString();
			}
			else
			{
				if (req.exposure.at(0) == 's' || req.exposure.at(0) == 'S')
				{
					req_expo.exposure = pe_.getSingleExpoString();
				}
				else if(req.exposure.at(0) == 'd' || req.exposure.at(0) == 'D')
				{
					req_expo.exposure = pe_.getDoubleExpoString();
				}
				else
				{
					req_expo.exposure = req.exposure;
				}
			}
			setExposure::Response res_expo;
			set_exposure_srv_.call(req_expo, res_expo);

			if (!req.manual_trigger)
			{
				// Ask for a point cloud
				triggerCamera::Request req_trig;
				req_trig.trigger = 1;
				triggerCamera::Response res_trig;
				request_point_cloud_srv_.call(req_trig,res_trig);
			}

			time_ = ros::Time::now().toSec();
			estimate_object_pose_ = true;
		}
		return true;
	}

	bool VrmImageProc::createNewModelSrv(vrm3dvision::createNewModel::Request &req, vrm3dvision::createNewModel::Response &res)
	{
		res.success = pe_.createNewModel(req.model_name, req.cad_path, req.smp, req.app);

//		setExposure::Request req_expo;
//		req_expo.exposure = pe_.getExpoExplorationString();
//		setExposure::Response res_expo;
//		set_exposure_srv_.call(req_expo, res_expo);
//
//		triggerCamera::Request req_trig;
//		req_trig.trigger = 1;
//		triggerCamera::Response res_trig;
//		request_point_cloud_srv_.call(req_trig,res_trig);
//		compute_best_exposure_ = true;
//		save_best_exposure_ = true;

		return true;
	}



	bool VrmImageProc::saveNextSequenceSrv(vrm3dvision::saveNextSequence::Request &req, vrm3dvision::saveNextSequence::Response &res)
	{
		res.success = SLR_.saveNextSequence(req.folder_name);
		return true;
	}



	void VrmImageProc::publishImages(const vrm_protocol::image_group& ig)
	{
		std_msgs::Header tmp_header;
		tmp_header.seq = ig.header.sequence_id();
		tmp_header.stamp = ros::Time::now(); // TODO make smarter time stamp

		if (ig.header.has_left_img() && has_left_camera_)
		{
			tmp_header.frame_id = "vrm_stereo_camera";
			//cv::remap(ig.left_image, ig.left_image, SLR_.getLeftRectMapX(), SLR_.getLeftRectMapY(), cv::INTER_LINEAR);

			cv_bridge::CvImage left_image = cv_bridge::CvImage(tmp_header,"mono8",ig.left_image);

			sensor_msgs::Image msg;
			left_image.toImageMsg(msg);

			// get current CameraInfo data
			sensor_msgs::CameraInfo cam_info = left_camera_.info_manager->getCameraInfo();
			cam_info.header.stamp = msg.header.stamp;
			cam_info.header.seq = msg.header.seq;
			cam_info.header.frame_id = msg.header.frame_id;
			cam_info.height = msg.height;
			cam_info.width = msg.width;
			left_camera_.publisher.publish(msg, cam_info);
		}
		if (ig.header.has_right_img() && has_right_camera_)
		{
			tmp_header.frame_id = "vrm_stereo_camera";
			cv_bridge::CvImage right_image = cv_bridge::CvImage(tmp_header,"mono8",ig.right_image);
			sensor_msgs::Image msg;
			right_image.toImageMsg(msg);

			// get current CameraInfo data
			sensor_msgs::CameraInfo cam_info = right_camera_.info_manager->getCameraInfo();
			cam_info.header.stamp = msg.header.stamp;
			cam_info.header.seq = msg.header.seq;
			cam_info.header.frame_id = msg.header.frame_id;
			cam_info.height = msg.height;
			cam_info.width = msg.width;
			right_camera_.publisher.publish(msg, cam_info);
		}
		if (ig.header.has_color_img() && has_color_camera_)
		{
			tmp_header.frame_id = "vrm_color_camera";
			cv::Mat cv_img;
			cv::cvtColor(ig.color_image,cv_img,CV_BayerGB2BGR);
			cv_bridge::CvImage color_image = cv_bridge::CvImage(tmp_header,"bgr8",cv_img);
			sensor_msgs::Image msg;
			color_image.toImageMsg(msg);

			// get current CameraInfo data
			sensor_msgs::CameraInfo cam_info = color_camera_.info_manager->getCameraInfo();
			cam_info.header.stamp = msg.header.stamp;
			cam_info.header.seq = msg.header.seq;
			cam_info.header.frame_id = msg.header.frame_id;
			cam_info.height = msg.height;
			cam_info.width = msg.width;
			color_camera_.publisher.publish(msg, cam_info);
		}
	}

	void VrmImageProc::mainLoop()
	{
		while (ros::ok())
		{
			ros::spinOnce();
			vrm_protocol::image_group ig;
			int exposure;
			if (vrm_image_client_->receive(ig,1000))
			{
				switch (ig.header.cam_mode()) {
					case vrm_protocol::MODE_STREAMING:
						publishImages(ig);
						break;
					case vrm_protocol::MODE_RANDOM_DOT_PATTERN:
						publishImages(ig);
						break;
					case vrm_protocol::MODE_STRIPE_PATTERN:
						exposure = SLR_.addImages(ig);
						if (exposure == ig.header.num_exposures())
						{
							point_cloud_publisher_.publish(SLR_.getPointCloud());
							point_cloud_trimmed_publisher_.publish(SLR_.getPointCloudTrimmed());
							ROS_INFO_STREAM("PC size:         " << SLR_.getPointCloud().size());
							ROS_INFO_STREAM("PC trimmed size: " << SLR_.getPointCloudTrimmed().size());

							if(estimate_object_pose_)
							{
								if(pe_.estimatePose(SLR_.getPointCloudTrimmed()))
								{
									transformBroadcaster_.sendTransform(tf::StampedTransform(pe_.getTransform(), ros::Time::now(), "/vrm_stereo_camera", "/object"));

									// Publish transform and model
									pcl_msgs::PolygonMesh pm_tmp;
									pcl_conversions::fromPCL(pe_.getModelMesh(), pm_tmp);
									pm_tmp.header.frame_id = "/object";
									model_mesh_publisher_.publish(pm_tmp);
								}
								estimate_object_pose_ = false;
								ROS_INFO_STREAM("Total time: " << (ros::Time::now().toSec() - time_)*1000.0);
							}
						}
						break;
					default:
						break;
				}
			}
			else
			{
				//ROS_INFO("No new images");
			}
		}
		vrm_image_client_->shutdown();
		delete(vrm_image_client_);
	}
} /* namespace vrm3dvision */
