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
		local_nh.param<bool>("allow_robot_control", allow_robot_control_, true);

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

		listener = new tf::TransformListener();
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
		computeBestExposureSrv_ = nh_.advertiseService("compute_best_exposure", &VrmImageProc::computeBestExposureSrv, this);
		createNewModelSrv_ = nh_.advertiseService("create_new_model", &VrmImageProc::createNewModelSrv, this);
		saveNextSequenceSrv_ = nh_.advertiseService("saveNextSequence", &VrmImageProc::saveNextSequenceSrv, this);

		pe_.initialize();

		if (allow_robot_control_)
		{

			move_group_ = new moveit::planning_interface::MoveGroup("manipulator", boost::shared_ptr<tf::Transformer>(), ros::Duration(10));

			// Setup move_group
			move_group_->setWorkspace(-2, -2, -2, 2, 2, 2);
			move_group_->setPlannerId("RRTConnectkConfigDefault");
			move_group_->setPlanningTime(5);
			move_group_->setGoalOrientationTolerance(0.002);
			move_group_->setGoalPositionTolerance(0.0005);

			moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

			ROS_INFO_STREAM("MoveGroup created with name: " << move_group_->getName());
			ROS_INFO("Reference frame: %s", move_group_->getEndEffectorLink().c_str());

			robot_action_client_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/arm_controller/follow_joint_trajectory", true);

			if (robot_action_client_->waitForServer(ros::Duration(3.0)))
			{
				ROS_INFO("Connected to robot..");
				acquireSequenceOnRobotSrv_ = nh_.advertiseService("acquire_sequence_on_robot", &VrmImageProc::acquireSequenceOnRobot, this);
				joint_trajectory_.trajectory.joint_names.push_back("shoulder_pan_joint");
				joint_trajectory_.trajectory.joint_names.push_back("shoulder_lift_joint");
				joint_trajectory_.trajectory.joint_names.push_back("elbow_joint");
				joint_trajectory_.trajectory.joint_names.push_back("wrist_1_joint");
				joint_trajectory_.trajectory.joint_names.push_back("wrist_2_joint");
				joint_trajectory_.trajectory.joint_names.push_back("wrist_3_joint");

				trajectory_point_.velocities.resize(6,0.0);
				trajectory_point_.time_from_start = ros::Duration(10);
				seq_state_ = sequence_state::READY;

			}
			else
			{
				seq_state_ = sequence_state::NOT_AVAILABLE;
				ROS_WARN("Unable to connect to robot driver.. Use of robot will not be possible..");
				allow_robot_control_ = false;
			}

		}

	}


	bool VrmImageProc::acquireSequenceOnRobot(vrm3dvision::acquireSequenceOnRobot::Request &req, vrm3dvision::acquireSequenceOnRobot::Response &res)
	{
		if 	(seq_state_ == sequence_state::READY)
		{
			seq_state_ = sequence_state::START;
		}
		else
		{
			ROS_ERROR_STREAM("Failed to start sequence..");
		}

		return true;
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

			estimate_object_pose_ = true;
		}
		return true;
	}



	bool VrmImageProc::computeBestExposureSrv(vrm3dvision::computeBestExposure::Request &req, vrm3dvision::computeBestExposure::Response &res)
	{
		//TODO specify model name: "" = dont save, "model name" = save if exists
		//TODO specify exposure: "" = use default string, "1 3 5 8" = use this expo
		if(req.model_name.empty())
		{
			save_best_exposure_ = false;
		}
		else
		{
			save_best_exposure_ = true;
		}
		setExposure::Request req_expo;
		req_expo.exposure = pe_.getExpoExplorationString();
		setExposure::Response res_expo;
		set_exposure_srv_.call(req_expo, res_expo);

		triggerCamera::Request req_trig;
		req_trig.trigger = 1;
		triggerCamera::Response res_trig;
		request_point_cloud_srv_.call(req_trig,res_trig);
		compute_best_exposure_ = true;

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

		// Sequence on robot variables
		double Q[5][6] = {	{4.686616460509157, -1.0642678877486176, -1.370877896551514, -2.3118966843081368, 1.5819701305466067, 0.0033684300627503127},
//							{4.686625888099416, -0.4349378437077142, -1.9104977784207, -2.0396790422702527, 1.581678470465072, 0.003449443423114177},
							{3.9217245139348553, -1.2621902574492638, -1.5053003517640846, -2.1502018786734016, 1.7678988485590268, -0.8197774529477213},
//							{3.7812492006304077, -1.7558937161019479, -1.227164682166345, -2.303540407855327, 1.815662312479777, -1.0527738068144048},
							{4.313036998441368, -2.0342691405052467, -0.8524093726919497, -2.622444479537444, 1.7849831760896766, 0.02237146795350909},
//							{4.655515709500898, -1.906779076835634, -0.8788364677347188, -2.5740738468460616, 1.582447665956432, -0.05820814707493849},
							{5.121633783390906, -2.040359715349746, -0.8523810899211731, -2.7251858029142704, 1.3482617339571388, -0.1670467684467125},
//							{5.68081841415763, -1.9791656616072597, -0.852588503298455, -2.916021353956386, 1.1933600663680624, -0.3637851854189025},
							{5.411107469312342, -1.4955624373973546, -1.116722034613898, -2.553475265204006, 1.4888155998674775, 0.7788318627962331} //,
//							{4.811008733068261, -1.2751054552947847, -0.9479137893610531, -2.6104366576927074, 1.5641814182128877, 0.12114574923353572}
						};

		geometry_msgs::Pose scan_pose;
		scan_pose.position.x = -0.2;
		scan_pose.position.y = 0.35;
		scan_pose.position.z = 0.70;

		scan_pose.orientation.x = -0.500206;
		scan_pose.orientation.y = 0.512015;
		scan_pose.orientation.z = 0.500089;
		scan_pose.orientation.w = 0.487386;

		geometry_msgs::Pose pre_pick_up_pose;
		pre_pick_up_pose.position.x = -0.2;
		pre_pick_up_pose.position.y = 0.35;
		pre_pick_up_pose.position.z = 0.4;

		pre_pick_up_pose.orientation.x = -0.500206;
		pre_pick_up_pose.orientation.y = 0.512015;
		pre_pick_up_pose.orientation.z = 0.500089;
		pre_pick_up_pose.orientation.w = 0.487386;

		geometry_msgs::Pose pick_up_pose;
		pick_up_pose.position.x = -0.2;
		pick_up_pose.position.y = 0.35;
		pick_up_pose.position.z = 0.4;

		pick_up_pose.orientation.x = -0.500206;
		pick_up_pose.orientation.y = 0.512015;
		pick_up_pose.orientation.z = 0.500089;
		pick_up_pose.orientation.w = 0.487386;

		geometry_msgs::Pose deliver_pose1_entry;
		deliver_pose1_entry.position.x = 0.15;
		deliver_pose1_entry.position.y = 0.60;
		deliver_pose1_entry.position.z = 0.35;

		deliver_pose1_entry.orientation.x = -0.25386;
		deliver_pose1_entry.orientation.y = 0.67316;
		deliver_pose1_entry.orientation.z = 0.24347;
		deliver_pose1_entry.orientation.w = 0.65048;

		geometry_msgs::Pose deliver_pose2_entry;
		deliver_pose2_entry.position.x = 0.15;
		deliver_pose2_entry.position.y = 0.495;
		deliver_pose2_entry.position.z = 0.35;

		deliver_pose2_entry.orientation.x = -0.25386;
		deliver_pose2_entry.orientation.y = 0.67316;
		deliver_pose2_entry.orientation.z = 0.24347;
		deliver_pose2_entry.orientation.w = 0.65048;

		geometry_msgs::Pose deliver_pose3_entry;
		deliver_pose3_entry.position.x = 0.15;
		deliver_pose3_entry.position.y = 0.39;
		deliver_pose3_entry.position.z = 0.35;

		deliver_pose3_entry.orientation.x = -0.25386;
		deliver_pose3_entry.orientation.y = 0.67316;
		deliver_pose3_entry.orientation.z = 0.24347;
		deliver_pose3_entry.orientation.w = 0.65048;

		geometry_msgs::Pose deliver_pose4_entry;
		deliver_pose4_entry.position.x = 0.15;
		deliver_pose4_entry.position.y = 0.285;
		deliver_pose4_entry.position.z = 0.35;

		deliver_pose4_entry.orientation.x = -0.25386;
		deliver_pose4_entry.orientation.y = 0.67316;
		deliver_pose4_entry.orientation.z = 0.24347;
		deliver_pose4_entry.orientation.w = 0.65048;

		std::vector<geometry_msgs::Pose> delivery_poses;
		delivery_poses.push_back(deliver_pose1_entry);
		delivery_poses.push_back(deliver_pose2_entry);
		delivery_poses.push_back(deliver_pose3_entry);
		delivery_poses.push_back(deliver_pose4_entry);

		std::vector<geometry_msgs::Pose> pre_delivery_poses;
		deliver_pose1_entry.position.z += 0.1;
		pre_delivery_poses.push_back(deliver_pose1_entry);
		deliver_pose2_entry.position.z += 0.1;
		pre_delivery_poses.push_back(deliver_pose2_entry);
		deliver_pose3_entry.position.z += 0.1;
		pre_delivery_poses.push_back(deliver_pose3_entry);
		deliver_pose4_entry.position.z += 0.1;
		pre_delivery_poses.push_back(deliver_pose4_entry);

		int delivery_position_idx = 0;
		bool image_sequence_ongoing = false;
		bool pose_estimation_started = false;
		bool move_to_next_position = false;

		moveit::planning_interface::MoveGroup::Plan my_plan;
		bool plan_success = false;
		bool move_success = false;
		bool object_found = false;

		ros::AsyncSpinner spinner(1);
		spinner.start();

		while (ros::ok())
		{
			ros::spinOnce();
			vrm_protocol::image_group ig;
			int exposure;
			bool success = false;

			// Sequence on robot state machine
			switch (seq_state_) {
				case sequence_state::START:
					delivery_position_idx = 0;
					ROS_INFO_STREAM("Sequence on robot is started.. - Going to scan position");
					seq_state_ = sequence_state::GO_TO_SCAN_POSITION;
					break;
				case sequence_state::GO_TO_SCAN_POSITION:

					move_group_->clearPoseTargets();
					move_group_->setPoseTarget(scan_pose);
					plan_success = move_group_->plan(my_plan);
					if (plan_success)
					{
						move_success = move_group_->execute(my_plan);
					}
					else
					{
						ROS_ERROR_STREAM("Failed to plan path to SCAN POSITION - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					if (move_success)
					{
						ROS_INFO_STREAM("Move to SCAN POSITION succeded - Trying to find object");
						seq_state_ = sequence_state::FIND_OBJECT;
					}
					else
					{
						ROS_ERROR_STREAM("Failed to move to SCAN POSITION - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}
					break;
				case sequence_state::FIND_OBJECT:
					// Find object..
					if (!pose_estimation_started)
					{
						vrm3dvision::acquirePoseEstimate::Request req;
						vrm3dvision::acquirePoseEstimate::Response res;

						req.model = "trafo3";
						req.exposure = "12";
						//req.method = "halcon";

						acquirePoseEstimateSrv(req, res);

						if (res.success)
						{
							ROS_INFO_STREAM("Pose estimation started successfully");
							pose_estimation_started = true;
							image_sequence_ongoing = true;
						}
						else
						{
							ROS_ERROR_STREAM("Service Call to pose estimator failed..! - Aborting..");
							seq_state_ = sequence_state::READY;
						}
					}
					else
					{
						if (!image_sequence_ongoing) // wait for pose estimation to be done..
						{
							if (object_found)
							{
								// Short sleep to make sure that the transform is available..
								ros::Duration(0.5).sleep();
							    tf::StampedTransform transform;
							    try{
							    	listener->lookupTransform("/table_link", "/object_grasp", ros::Time(0), transform);
							    }
							    catch (tf::TransformException ex){
							    	ROS_ERROR("PCL Viewer Node: %s",ex.what());
							    	return;
							    }

							    // Create pick-up pose..
							    pick_up_pose.position.x = transform.getOrigin().x();
								pick_up_pose.position.y = transform.getOrigin().y();
								pick_up_pose.position.z = transform.getOrigin().z();

								pick_up_pose.orientation.x = transform.getRotation().x();
								pick_up_pose.orientation.y = transform.getRotation().y();
								pick_up_pose.orientation.z = transform.getRotation().z();
								pick_up_pose.orientation.w = transform.getRotation().w();

							    tf::StampedTransform pre_transform;
							    try{
							    	listener->lookupTransform("/table_link", "/object_pre_grasp", ros::Time(0), pre_transform);
							    }
							    catch (tf::TransformException ex){
							    	ROS_ERROR("PCL Viewer Node: %s",ex.what());
							    	return;
							    }

							    pre_pick_up_pose.position.x = pre_transform.getOrigin().x();
							    pre_pick_up_pose.position.y = pre_transform.getOrigin().y();
							    pre_pick_up_pose.position.z = pre_transform.getOrigin().z();

							    pre_pick_up_pose.orientation.x = pre_transform.getRotation().x();
							    pre_pick_up_pose.orientation.y = pre_transform.getRotation().y();
							    pre_pick_up_pose.orientation.z = pre_transform.getRotation().z();
							    pre_pick_up_pose.orientation.w = pre_transform.getRotation().w();

							    ROS_INFO_STREAM("Object found at: x: " << transform.getOrigin().x() << " y: " << transform.getOrigin().y() << " z: " << transform.getOrigin().z());

								object_found = false;
								seq_state_ = sequence_state::PRE_PICK_UP;
							}
							else
							{
								ROS_ERROR_STREAM("Object not found.. - Aborting..");
								seq_state_ = sequence_state::READY;
							}
							pose_estimation_started = false;
						}
					}

					break;
				case sequence_state::PRE_PICK_UP:

					move_group_->clearPoseTargets();
					move_group_->setPoseTarget(pre_pick_up_pose);
					plan_success = move_group_->plan(my_plan);
					if (plan_success)
					{
						move_success = move_group_->execute(my_plan);
					}
					else
					{
						ROS_ERROR_STREAM("Failed to plan path to PRE_PICK_UP_POSITION - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					if (move_success)
					{
						ROS_INFO_STREAM("Move to PRE_PICK_UP_POSITION succeded - Going to PICK_UP_POSITION");
						seq_state_ = sequence_state::PICK_UP;
					}
					else
					{
						ROS_ERROR_STREAM("Failed to move to PRE_PICK_UP_POSITION - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					break;

				case sequence_state::PICK_UP:

					move_group_->clearPoseTargets();
					move_group_->setPoseTarget(pick_up_pose);
					plan_success = move_group_->plan(my_plan);
					if (plan_success)
					{
						move_success = move_group_->execute(my_plan);
					}
					else
					{
						ROS_ERROR_STREAM("Failed to plan path to PICK_UP_POSITION - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					if (move_success)
					{
						ROS_INFO_STREAM("Move to PICK_UP_POSITION succeded - Going to POST_PICK_UP_POSITION");
						seq_state_ = sequence_state::POST_PICK_UP;
					}
					else
					{
						ROS_ERROR_STREAM("Failed to move to PICK_UP_POSITION - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					break;

				case sequence_state::POST_PICK_UP:

					move_group_->clearPoseTargets();
					move_group_->setPoseTarget(pre_pick_up_pose);
					plan_success = move_group_->plan(my_plan);
					if (plan_success)
					{
						char tmp;
						std::cin >> tmp;
						move_success = move_group_->execute(my_plan);
					}
					else
					{
						ROS_ERROR_STREAM("Failed to plan path to POST_PICK_UP_POSITION - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					if (move_success)
					{
						ROS_INFO_STREAM("Move to POST_PICK_UP_POSITION succeded - Going to PRE_DELIVERY_POSITION");
						seq_state_ = sequence_state::PRE_DELIVER_OBJECT;
					}
					else
					{
						ROS_ERROR_STREAM("Failed to move to POST_PICK_UP_POSITION - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					break;

				case sequence_state::PRE_DELIVER_OBJECT:

					move_group_->clearPoseTargets();
					move_group_->setPoseTarget(pre_delivery_poses[delivery_position_idx]);

					plan_success = move_group_->plan(my_plan);
					if (plan_success)
					{
						move_success = move_group_->execute(my_plan);
					}
					else
					{
						ROS_ERROR_STREAM("Failed to plan path to PRE_DELIVERY_POSITION NO: " << delivery_position_idx << " - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					if (move_success)
					{
						ROS_INFO_STREAM("Move to PRE_DELIVERY_POSITION NO: " << delivery_position_idx << " succeded - Going to DELIVERY_POSITION ..");
						seq_state_ = sequence_state::DELIVER_OBJECT;

					}
					else
					{
						ROS_ERROR_STREAM("Failed to move to PRE_DELIVERY_POSITION NO: " << delivery_position_idx << " - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					break;
				case sequence_state::DELIVER_OBJECT:

					move_group_->clearPoseTargets();
					move_group_->setPoseTarget(delivery_poses[delivery_position_idx]);

					plan_success = move_group_->plan(my_plan);
					if (plan_success)
					{
						move_success = move_group_->execute(my_plan);
					}
					else
					{
						ROS_ERROR_STREAM("Failed to plan path to DELIVERY_POSITION NO: " << delivery_position_idx << " - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					if (move_success)
					{
						ROS_INFO_STREAM("Move to DELIVERY_POSITION NO: " << delivery_position_idx << " succeded - Going to POST_DELIVERY_POSITION..");
						seq_state_ = sequence_state::POST_DELIVER_OBJECT;
					}
					else
					{
						ROS_ERROR_STREAM("Failed to move to DELIVERY_POSITION NO: " << delivery_position_idx << " - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					break;
				case sequence_state::POST_DELIVER_OBJECT:

					move_group_->clearPoseTargets();
					move_group_->setPoseTarget(pre_delivery_poses[delivery_position_idx]);

					plan_success = move_group_->plan(my_plan);
					if (plan_success)
					{
						char tmp;
						std::cin >> tmp;
						move_success = move_group_->execute(my_plan);
					}
					else
					{
						ROS_ERROR_STREAM("Failed to plan path to POST_DELIVERY_POSITION NO: " << delivery_position_idx << " - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					if (move_success)
					{
						delivery_position_idx++;
						if (delivery_position_idx < 4)
						{
							ROS_INFO_STREAM("Move to POST_DELIVERY_POSITION NO: " << delivery_position_idx << " succeded - Going to SCAN POSITION..");
							seq_state_ = sequence_state::GO_TO_SCAN_POSITION;
						}
						else
						{
							ROS_INFO_STREAM("Move to DELIVERY_POSITION NO: " << delivery_position_idx << " succeded - No more delivery positions.. Stopping..");
							seq_state_ = sequence_state::READY;
						}
					}
					else
					{
						ROS_ERROR_STREAM("Failed to move to DELIVERY_POSITION NO: " << delivery_position_idx << " - Aborting..");
						seq_state_ = sequence_state::READY;
						move_success = false;
						break;
					}

					break;
				default:
					break;
			}

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
									object_found = true;
								}
								else
								{
									object_found = false;

								}
								estimate_object_pose_ = false;
							}

							if(compute_best_exposure_)
							{
								pe_.computeBestExposure(SLR_.getExistingEdges(), save_best_exposure_);
								compute_best_exposure_ = false;
							}
							image_sequence_ongoing = false;
						}
						else if(exposure > 0)
						{
							if(compute_best_exposure_)
							{
								point_cloud_publisher_.publish(SLR_.getPointCloud());
								point_cloud_trimmed_publisher_.publish(SLR_.getPointCloudTrimmed());
								pe_.addExistingEdges(SLR_.getExistingEdges());
								SLR_.clearExistingEdgesAndCloud();
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
