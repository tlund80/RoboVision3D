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

#define TEST_POSE_ESTIMATION false

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
			robot_action_client_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/arm_controller/follow_joint_trajectory", true);

			if (robot_action_client_->waitForServer(ros::Duration(10.0)))
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
			sequence_save_path_ = req.save_path;

			// Create folder
			if (!boost::filesystem::exists(sequence_save_path_))
			{
				if (sequence_save_path_.at(sequence_save_path_.size()-1) != '/')
				{
					sequence_save_path_.push_back('/');
				}
				if (boost::filesystem::create_directories(sequence_save_path_))
				{
					// Set exposure
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
					exposure_for_pose_ = req.exposure;
					pose_estimate_model_ = req.model;

					seq_state_ = sequence_state::START;
					if (save_results_ap.is_open())
					{
						save_results_ap.close();
					}
					std::stringstream ss;
					ss << sequence_save_path_ << "data_ap.txt";
					save_results_ap.open(ss.str().c_str());

					if (save_results_h.is_open())
					{
						save_results_h.close();
					}
					ss.str("");
					ss << sequence_save_path_ << "data_h.txt";
					save_results_h.open(ss.str().c_str());

					ROS_WARN_STREAM("SeqOnRobot triggered.. Going to start position..");
				}
				else
				{
					ROS_ERROR_STREAM("Failed to create folder: " << sequence_save_path_);
				}
			}
			else
			{
				ROS_ERROR_STREAM("Folder already exists.. Images will not be saved..! - try new path to unexisting folder");
			}


		}
		else if (seq_state_ == sequence_state::ONGOING)
		{
			ROS_WARN_STREAM("Sequence is already ongoing..");
		}
		else if (seq_state_ == sequence_state::NOT_AVAILABLE)
		{
			ROS_ERROR_STREAM("Robot control not possible..");
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
			cv::remap(ig.left_image, ig.left_image, SLR_.getLeftRectMapX(), SLR_.getLeftRectMapY(), cv::INTER_LINEAR);

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
		int position_idx = -1;
		int repetitions = 0;
		bool image_sequence_ongoing = false;
		bool move_to_next_position = false;

		while (ros::ok())
		{
			ros::spinOnce();
			vrm_protocol::image_group ig;
			int exposure;

			// Sequence on robot state machine
			switch (seq_state_) {
				case sequence_state::START:
					position_idx = -1;
					repetitions = 0;
					ROS_INFO_STREAM("Sequence on robot is started.. - Going to start position");

					seq_state_ = sequence_state::ONGOING;
					move_to_next_position = true;
					break;
				case sequence_state::ONGOING:
					if (move_to_next_position)
					{
						if (position_idx == 4)
						{
							save_results_ap.close();
							save_results_h.close();
							seq_state_ = sequence_state::READY;
						}
						else
						{
							// Move robot to next position
							position_idx++;

							joint_trajectory_.trajectory.points.clear();
							trajectory_point_.time_from_start = ros::Duration(3);
							trajectory_point_.positions.assign(Q[position_idx], Q[position_idx]+6);
							joint_trajectory_.trajectory.points.push_back(trajectory_point_);

							robot_action_client_->sendGoal(joint_trajectory_);

							if (robot_action_client_->waitForResult(ros::Duration(6)))
							{
								if (save_results_ap.is_open())
								{
									save_results_ap << "pos: " << position_idx << "\n";
								}
								else
								{
									ROS_ERROR_STREAM("Unable to write to file.. ");
								}
								if (save_results_h.is_open())
								{
									save_results_h << "pos: " << position_idx << "\n";
								}
								else
								{
									ROS_ERROR_STREAM("Unable to write to file.. ");
								}


								repetitions = 0;
								ros::Duration(0.5).sleep();
								ROS_INFO("Robot reached its position - starting point cloud acquisition..!");

								std::stringstream ss;
								ss << sequence_save_path_ << "pos_" << position_idx;
								if (!boost::filesystem::create_directories(ss.str().c_str()))
								{
									ROS_ERROR_STREAM("Unable to create folder: " << ss.str() << " - sequence is stopped!");
									seq_state_ = sequence_state::READY;
								}
								else
								{
									ROS_INFO_STREAM("Created folder: " << ss.str() << " - sequence is stopped!");
								}

							}
							else
							{
								ROS_ERROR("Robot could not go to desired position - sequence is aborted..");
								seq_state_ = sequence_state::READY;
							}
							move_to_next_position = false;
						}

					}
					if (!image_sequence_ongoing)
					{
						if (repetitions == 10)
						{
							move_to_next_position = true;
						}
						else
						{
							// Set save next sequence
							std::stringstream ss;
							ss << sequence_save_path_ << "pos_" << position_idx << "/rep_" << repetitions;
							vrm3dvision::saveNextSequence::Request req;
							req.folder_name = ss.str();
							vrm3dvision::saveNextSequence::Response res;
							saveNextSequenceSrv(req, res);
							if (res.success)
							{
//								// trigger camera
//								triggerCamera::Request req_trig;
//								req_trig.trigger = 1;
//								triggerCamera::Response res_trig;
//								request_point_cloud_srv_.call(req_trig,res_trig);

								vrm3dvision::acquirePoseEstimate::Request req;
								vrm3dvision::acquirePoseEstimate::Response res;
								req.model = pose_estimate_model_;
								req.exposure = exposure_for_pose_;
								acquirePoseEstimateSrv(req, res);
								if (!res.success)
								{
									ROS_ERROR_STREAM("Unable to call acquire pose estimate on model: " << pose_estimate_model_);
								}


								image_sequence_ongoing = true;
								repetitions++;
							}
							else
							{
								ROS_ERROR_STREAM("Could not save sequence to path: " << ss.str() << " - sequence aborted");
								seq_state_ = sequence_state::READY;
							}
						}
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
								if (TEST_POSE_ESTIMATION)
								{
									// ALIGNMENT PREREJECTIVE
									if (save_results_ap.is_open())
									{
										save_results_ap << "Start poseestimation\n";
									}
									else
									{
										ROS_ERROR_STREAM("Unable to write to file.. ");
									}
									int iterations = 1;
									std::vector<Eigen::Vector3d> rot_vec;
									std::vector<Eigen::Vector3d> trans_vec;
									int succeded = 0;
									std::vector<double> time;
									double total_time = 0;
									for (int i = 0; i < iterations; i++)
									{
										double start_time = ros::Time::now().toSec();
										if (pe_.estimatePose(SLR_.getPointCloudTrimmed()))
										{
											succeded++;
											tf::Transform transform = pe_.getTransform();

											tf::StampedTransform hand_eye_transform;
											try{
												listener->lookupTransform("/base_link", "/vrm_stereo_camera", ros::Time(0), hand_eye_transform);
												transform = hand_eye_transform * transform;
											}
											catch (tf::TransformException ex){
												ROS_ERROR("Unable to get transform: %s",ex.what());
											}

											Eigen::Vector3d rot;
											transform.getBasis().getEulerYPR(rot[0], rot[1], rot[2]);
											rot *= 180 / M_PI;
											rot_vec.push_back(rot);

											Eigen::Vector3d trans;
											trans[0] = transform.getOrigin().x();
											trans[1] = transform.getOrigin().y();
											trans[2] = transform.getOrigin().z();

											ROS_INFO_STREAM("Pose: x: " << trans[0] << " y: " << trans[1] << " z: " << trans[2] << " R: " << rot[0] << " P: " << rot[1] << " Y: " << rot[2]);

											//Eigen::Matrix3d rot_mat;
											//tf::matrixTFToEigen(transform.getBasis(), rot_mat);
											//trans = rot_mat * trans;

											trans_vec.push_back(trans);

											transformBroadcaster_.sendTransform(tf::StampedTransform(pe_.getTransform(), ros::Time::now(), "/vrm_stereo_camera", "/object"));

											// Publish transform and model
											pcl_msgs::PolygonMesh pm_tmp;
											pcl_conversions::fromPCL(pe_.getModelMesh(), pm_tmp);
											pm_tmp.header.frame_id = "/object";
											model_mesh_publisher_.publish(pm_tmp);

											if (save_results_ap.is_open())
											{
												save_results_ap << trans[0] << " " << trans[1] << " " << trans[2] << " " << rot[0] << " " << rot[1] << " " << rot[2] << "\n";
											}
											else
											{
												ROS_ERROR_STREAM("Unable to write to file.. ");
											}

										}
										else
										{
											if (save_results_ap.is_open())
											{
												save_results_ap << "FAILED\n";
											}
											else
											{
												ROS_ERROR_STREAM("Unable to write to file.. ");
											}
										}
										double tmp_time = ros::Time::now().toSec() - start_time;
										total_time += tmp_time;
										time.push_back(tmp_time);
									}

									// Calc average translation
									Eigen::Vector3d avg_trans = Eigen::Vector3d::Zero();
									for (int i = 0; i < trans_vec.size(); i++)
									{
										avg_trans = avg_trans + trans_vec[i];
									}
									avg_trans /= trans_vec.size();

									// Calc RMSE of translation
									Eigen::Vector3d rmse_trans = Eigen::Vector3d::Zero();
									for (int i = 0; i < trans_vec.size(); i++)
									{
										Eigen::Vector3d tmp = (trans_vec[i] - avg_trans);
										rmse_trans = rmse_trans + (tmp.cwiseProduct(tmp));
									}
									rmse_trans[0] = sqrt(rmse_trans[0]);
									rmse_trans[1] = sqrt(rmse_trans[1]);
									rmse_trans[2] = sqrt(rmse_trans[2]);
									double rmse_eucleadean = rmse_trans.norm();

									// Calc average rotation
									Eigen::Vector3d avg_rot = Eigen::Vector3d::Zero();
									for (int i = 0; i < rot_vec.size(); i++)
									{
										avg_rot = avg_rot + rot_vec[i];
									}
									avg_rot /= rot_vec.size();

									// Calc RMSE of rotation
									Eigen::Vector3d rmse_rot = Eigen::Vector3d::Zero();
									for (int i = 0; i < rot_vec.size(); i++)
									{
										Eigen::Vector3d tmp = (rot_vec[i] - avg_rot);
										rmse_rot = rmse_rot + (tmp.cwiseProduct(tmp));
									}
									rmse_rot[0] = sqrt(rmse_rot[0]);
									rmse_rot[1] = sqrt(rmse_rot[1]);
									rmse_rot[2] = sqrt(rmse_rot[2]);

									if (save_results_ap.is_open())
									{
										save_results_ap << "Avg time: " << (total_time/(double)iterations) << "\n";
									}
									else
									{
										ROS_ERROR_STREAM("Unable to write to file.. ");
									}

									ROS_INFO_STREAM("PoseEstimationTest: Succes: " << succeded << "/" << iterations << " Avg Time: " << total_time/(double)iterations);
									ROS_INFO_STREAM("Avg rotation: " << avg_rot[0] << " " << avg_rot[1] << " " << avg_rot[2]);
									ROS_INFO_STREAM("rotation RMSE: " << rmse_rot[0] << " " << rmse_rot[1] << " " << rmse_rot[2]);
									ROS_INFO_STREAM("Avg translation: " << avg_trans[0] << " " << avg_trans[1] << " " << avg_trans[2]);
									ROS_INFO_STREAM("Translation RMSE: " << rmse_trans[0] << " " << rmse_trans[1] << " " << rmse_trans[2]);
									ROS_INFO_STREAM("Eucleadean Translation RMSE: " << rmse_eucleadean);

									// HALCON
									if (save_results_h.is_open())
									{
										save_results_h << "Start poseestimation\n";
									}
									else
									{
										ROS_ERROR_STREAM("Unable to write to file.. ");
									}

									vrm3dvision::acquirePoseEstimate::Request req;
									vrm3dvision::acquirePoseEstimate::Response res;
									req.model = pose_estimate_model_;
									req.smp.min_score_threshold = 0.75;
									req.exposure = exposure_for_pose_;
									req.method = "h";

									res.success = pe_.prepareModel(req.model, req.smp, req.app, req.method);

									Eigen::Vector3d rot;
									Eigen::Vector3d trans;

									double start_time = ros::Time::now().toSec();
									if (pe_.estimatePose(SLR_.getPointCloudTrimmed()))
									{
										tf::Transform transform = pe_.getTransform();

										tf::StampedTransform hand_eye_transform;
										try{
											listener->lookupTransform("/base_link", "/vrm_stereo_camera", ros::Time(0), hand_eye_transform);
											transform = hand_eye_transform * transform;
										}
										catch (tf::TransformException ex){
											ROS_ERROR("Unable to get transform: %s",ex.what());
										}

										transform.getBasis().getEulerYPR(rot[0], rot[1], rot[2]);
										rot *= 180 / M_PI;

										trans[0] = transform.getOrigin().x();
										trans[1] = transform.getOrigin().y();
										trans[2] = transform.getOrigin().z();

										transformBroadcaster_.sendTransform(tf::StampedTransform(pe_.getTransform(), ros::Time::now(), "/vrm_stereo_camera", "/object"));

										// Publish transform and model
										pcl_msgs::PolygonMesh pm_tmp;
										pcl_conversions::fromPCL(pe_.getModelMesh(), pm_tmp);
										pm_tmp.header.frame_id = "/object";
										model_mesh_publisher_.publish(pm_tmp);

										if (save_results_h.is_open())
										{
											save_results_h << trans[0] << " " << trans[1] << " " << trans[2] << " " << rot[0] << " " << rot[1] << " " << rot[2] << "\n";
										}
										else
										{
											ROS_ERROR_STREAM("Unable to write to file.. ");
										}

									}
									else
									{
										if (save_results_h.is_open())
										{
											save_results_h << "FAILED\n";
										}
										else
										{
											ROS_ERROR_STREAM("Unable to write to file.. ");
										}
									}
									if (save_results_h.is_open())
									{
										save_results_h << "Avg time: " << (ros::Time::now().toSec() - start_time) << "\n";
									}
									else
									{
										ROS_ERROR_STREAM("Unable to write to file.. ");
									}

								}
								else
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
								}
								estimate_object_pose_ = false;
							}
							image_sequence_ongoing = false;

							if(compute_best_exposure_)
							{
								pe_.computeBestExposure(SLR_.getExistingEdges(), save_best_exposure_);
								compute_best_exposure_ = false;
							}
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
