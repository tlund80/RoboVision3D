/********************************************************************************************************************
 *
 * \file                vrm_calibration.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-22
 * \version             0.1
 * \brief               image handler for VRmagic D3 camera module
 *
*********************************************************************************************************************/
#include "vrm_calibration.hpp"

#define LEFT_IDX 0
#define CENTER_IDX 1
#define RIGHT_IDX 2

#define LEFT_CENTER 0
#define LEFT_RIGHT 1
#define CENTER_LEFT 2
#define CENTER_RIGHT 3
#define RIGHT_LEFT 4
#define RIGHT_CENTER 5

namespace vrm3dvision {

	VrmCalibration::VrmCalibration(const ros::NodeHandle& nodeHandle)
	 : nh_(nodeHandle),
	   alpha_slider_max_(100000),
	   save_next_image_set_(false),
	   rectified_(false),
	   recalculate_rectification_(false),
	   test_rectification_(false),
	   transformation_estimated_(false)
	{
		// Read parameter from launch file
		ros::NodeHandle local_nh("~");
		local_nh.param<int>("calib_board_size_width", calib_board_size_.width, 9); // 9
		local_nh.param<int>("calib_board_size_height", calib_board_size_.height, 6); // 6
		local_nh.param<double>("chessboard_square_size", calib_board_square_size_, 0.0075); // 0.0075
		local_nh.param<std::string>("left_camera_topic", left_cam_topic_, "/vrm3dvision/left");
		local_nh.param<std::string>("center_camera_topic", center_cam_topic_, "/vrm3dvision/color_cam");
		local_nh.param<std::string>("right_camera_topic", right_cam_topic_, "/vrm3dvision/right");
		local_nh.param<std::string>("right_camera_topic", image_topic_, "image_raw");
		local_nh.param<std::string>("transport_hint", transport_hint_, "raw");
		local_nh.param<bool>("load_image_points_from_file", load_image_points_from_file_, false);
		local_nh.param<std::string>("image_point_file", image_point_file_, "/home/jeppe/calib.txt");
		local_nh.param<std::string>("grid_type", grid_type_string_, "chessboard");
		local_nh.param<bool>("ransac", use_ransac_, false);
		local_nh.param<bool>("show_exposure", show_exposure_, true);
		local_nh.param<bool>("use_matlab_intrinsic", use_matlab_intrinsic_, false);
		local_nh.param<std::string>("matlab_intrinsic_path", matlab_intrinsic_path_, "/home/jeppe/calib_images");

		if (grid_type_string_ == "acircle")
		{
			grid_type_ = ACIRCLE_GRID;
		}
		else if (grid_type_string_ == "chessboard")
		{
			grid_type_ = CHESSBOARD_GRID;
		}
		else
		{
			ROS_ERROR_STREAM("Unknown grid type - needs to be \"acircle\" or \"chessboard\" - type given was: " << grid_type_string_ << " - Grid type set to chessboard...");
			grid_type_ = CHESSBOARD_GRID;
		}
		calibration_images_.resize(3);
		transformation_images_.resize(3);
		image_points_transformation_.resize(6);
		image_points_.resize(3);
		camera_matrixes_.resize(3);
		dist_coeffs_.resize(3);
		R.resize(6);
		P.resize(6);
		map_x_.resize(6);
		map_y_.resize(6);
		cam_services_.resize(3);
		alpha_slider_.resize(3);
		alpha_.resize(3);
		for (int i = 0; i < 3; i++)
		{
			alpha_slider_[i] = 100;
		}

		T_.resize(2);
		T2_.resize(2);
	}

	VrmCalibration::~VrmCalibration()
	{
		cvDestroyAllWindows();
	}

	void VrmCalibration::initialize()
	{
		// Create GUI
		cv::namedWindow(GUI_NAME, cv::WINDOW_NORMAL);
		cv::resizeWindow(GUI_NAME, 640, 870);

		cv::createTrackbar("Alpha-Left-Right", GUI_NAME, &alpha_slider_[0], alpha_slider_max_, &VrmCalibration::on_alpha_trackbar_left_right, this);
		on_alpha_trackbar_left_right(alpha_slider_[0],this);
		cv::createTrackbar("Alpha-Left-Center", GUI_NAME, &alpha_slider_[1], alpha_slider_max_, &VrmCalibration::on_alpha_trackbar_left_center, this);
		on_alpha_trackbar_left_center(alpha_slider_[1],this);
		cv::createTrackbar("Alpha-Center-Right", GUI_NAME, &alpha_slider_[2], alpha_slider_max_, &VrmCalibration::on_alpha_trackbar_center_right, this);
		on_alpha_trackbar_center_right(alpha_slider_[2],this);

		cv::waitKey(1);

		// Set up subscribers
		image_transport::ImageTransport it(nh_);

		std::string topic = left_cam_topic_ + "/" + image_topic_;
		left_cam_subscriber_.subscribe(it, topic.c_str(), 1, transport_hint_);
		topic = center_cam_topic_ + "/" + image_topic_;
		center_cam_subscriber_.subscribe(it, topic.c_str(), 1, transport_hint_);
		topic = right_cam_topic_ + "/" + image_topic_;
		right_cam_subscriber_.subscribe(it, topic.c_str(), 1, transport_hint_);

		exact_sync_.reset( new ExactSync(ExactPolicy(5), left_cam_subscriber_, center_cam_subscriber_, right_cam_subscriber_) );
		exact_sync_->registerCallback(boost::bind(&VrmCalibration::imageCb, this, _1, _2, _3));

		std::string service = left_cam_topic_ + "/set_camera_info";
		cam_services_[LEFT_IDX] = nh_.serviceClient<sensor_msgs::SetCameraInfo>(service);
		service = center_cam_topic_ + "/set_camera_info";
		cam_services_[CENTER_IDX] = nh_.serviceClient<sensor_msgs::SetCameraInfo>(service);
		service = right_cam_topic_ + "/set_camera_info";
		cam_services_[RIGHT_IDX] = nh_.serviceClient<sensor_msgs::SetCameraInfo>(service);

		// Calculate chessboard corners
		calcChessboardCorners(calib_board_size_, calib_board_square_size_, chessboard_corners_);

	}

	void VrmCalibration::mainLoop()
	{
		initialize();

		ROS_INFO_STREAM(" *** VRM Calibration Node started!");
		ROS_INFO_STREAM(" *** Calibration plate width: " << calib_board_size_.width << " height: " << calib_board_size_.height);
		ROS_INFO_STREAM(" *** Calibration plate square size: " << calib_board_square_size_);
		ROS_INFO_STREAM(" *** Calibration plate type: " << grid_type_string_);
		ROS_INFO_STREAM(" *** Left camera topic: " << left_cam_topic_);
		ROS_INFO_STREAM(" *** Center camera topic: " << center_cam_topic_);
		ROS_INFO_STREAM(" *** Right camera topic: " << right_cam_topic_);
		ROS_INFO_STREAM(" *** Transport hint: " << transport_hint_);

		if (load_image_points_from_file_ && !use_matlab_intrinsic_)
		{
			ROS_INFO_STREAM(" *** RUNNING FROM LOADED FILE USING OpenCV intrinsic *** ");
			if (loadImagePoints(image_point_file_))
			{
				calibrate();
    			computeT();
			}
		}
		else if (load_image_points_from_file_ && use_matlab_intrinsic_)
		{
			if (loadImagePoints(image_point_file_))
			{
				if (loadMatlabIntrinsic(matlab_intrinsic_path_))
				{
					// Perform stereo calibration
						doStereoCalibration();

						// Do three seperate calibrations
						calibrateThreeSeperate();

						computeT();
				}
				else
				{
					ROS_ERROR_STREAM("Unable to load matlab intrinsic");
					exit(-1);
				}
			}
		}

		while (ros::ok())
		{
		    // Handle user input
		    char key = cv::waitKey(1);
		    if (key == ' ') // Save images
		    {
		    	save_next_image_set_ = true;
		    }
		    else if (key == 'c') // Perform calibration
		    {
		    	if (!rectified_ && calibration_images_[LEFT_IDX].size() >= 3)
		    	{
		    		calibrate();
	    			computeT();
		    	}
		    }
		    else if (key == 'i') // Perform calibration
		    {
		    	saveImages("calib_images");
		    }
		    else if (key == 's') // Commit calibration
		    {
		    	if (rectified_ && transformation_estimated_)
		    	{
		    		commitCalibration();
		    	}
		    }
		    else if (key == 'l') // test rectification
		    {
		    	if (rectified_)
		    	{
		    		test_rectification_ = true;
		    	}
		    }
		    else if (key == 't') // Calculate transform
		    {
		    	if (rectified_)
		    	{
//		    		if (transformation_images_[0].size() > 0)
//		    		{
		    			computeT();
//		    		}
//		    		else
//		    		{
//		    			ROS_INFO_STREAM("After calibration images needs to be saved before estimating transformation");
//		    		}
		    	}
		    	else
		    	{
		    		ROS_INFO_STREAM("Calibration needs to be done before estimation transformation between camera sets");
		    	}
		    }

		    if (recalculate_rectification_ && rectified_)
		    {
		    	calibrateThreeSeperate();
				computeT();
		    }

			// Spin for the specified duration
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.05));
		}
	}

	void VrmCalibration::on_alpha_trackbar_left_right( int, void* object )
	{
		VrmCalibration* vc = (VrmCalibration*) object;
		vc->alpha_[0] = (double) vc->alpha_slider_[0]/vc->alpha_slider_max_;
		if (vc->rectified_)
		{
			vc->recalculate_rectification_ = true;
		}
	}

	void VrmCalibration::on_alpha_trackbar_left_center( int, void* object )
	{
		VrmCalibration* vc = (VrmCalibration*) object;
		vc->alpha_[1] = (double) vc->alpha_slider_[1]/vc->alpha_slider_max_;
		if (vc->rectified_)
		{
			vc->recalculate_rectification_ = true;
		}
	}

	void VrmCalibration::on_alpha_trackbar_center_right( int, void* object )
	{
		VrmCalibration* vc = (VrmCalibration*) object;
		vc->alpha_[2] = (double) vc->alpha_slider_[2]/vc->alpha_slider_max_;
		if (vc->rectified_)
		{
			vc->recalculate_rectification_ = true;
		}
	}


	void VrmCalibration::updateGUI(const std::vector<cv::Mat>& input_images, cv::Mat& gui_image)
	{
	    std::vector<cv::Mat> small_images(6);
	    std::vector<cv::Mat> small_images_color(6);
	    std::vector<cv::Mat> images_rect(6);

	    int threads = 3;
		omp_set_dynamic(0);     // Explicitly disable dynamic teams
		omp_set_num_threads(threads); // Use 3 threads for all consecutive parallel regions
		#pragma omp parallel for
	    for (int idx = LEFT_IDX; idx <= RIGHT_IDX; idx++)
	    {
		    if (rectified_)
		    {
	            cv::remap(input_images[idx], images_rect[idx*2], map_x_[idx*2], map_y_[idx*2], cv::INTER_LINEAR);
		    	cv::resize(images_rect[idx*2], small_images[idx*2], cv::Size(320, 240));
		    	cv::cvtColor(small_images[idx*2], small_images_color[idx*2], cv::COLOR_GRAY2BGR);

	            cv::remap(input_images[idx], images_rect[idx*2+1], map_x_[idx*2+1], map_y_[idx*2+1], cv::INTER_LINEAR);
		    	cv::resize(images_rect[idx*2+1], small_images[idx*2+1], cv::Size(320, 240));
		    	cv::cvtColor(small_images[idx*2+1], small_images_color[idx*2+1], cv::COLOR_GRAY2BGR);
		    }
		    else
		    {
		    	cv::resize(input_images[idx], small_images[idx*2], cv::Size(320, 240));
		    	cv::cvtColor(small_images[idx*2], small_images_color[idx*2], cv::COLOR_GRAY2BGR);

		    	cv::Scalar meanScalar = cv::mean(small_images[idx*2]);
		    	double mean = meanScalar.val[0];

		    	std::stringstream ss;
		    	ss << "Mean: " << mean;

		    	cv::putText(small_images_color[idx*2], ss.str(), cvPoint(5,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,255,0), 1, CV_AA);

//		    	std::vector<cv::Point2f> ptvec;
//		    	bool found;
//		    	if (grid_type_ == CHESSBOARD_GRID)
//		    	{
//		    		found = cv::findChessboardCorners( small_images[idx*2], calib_board_size_, ptvec, cv::CALIB_CB_ADAPTIVE_THRESH );
//					cv::drawChessboardCorners( small_images_color[idx*2], calib_board_size_, cv::Mat(ptvec), found );
//		    	}
//		    	else if (grid_type_ == ACIRCLE_GRID)
//		    	{
//					found = cv::findCirclesGrid( small_images[idx*2], calib_board_size_, ptvec, cv::CALIB_CB_ASYMMETRIC_GRID 	);
//					cv::drawChessboardCorners(small_images_color[idx*2], calib_board_size_, cv::Mat(ptvec), found);
//		    	}

		    }
	    }

	    for (int idx = LEFT_IDX; idx <= RIGHT_IDX; idx++)
	    {
			if (rectified_)
			{
				small_images_color[LEFT_RIGHT].copyTo(gui_image(cv::Rect(0,0,320,240)));
				small_images_color[RIGHT_LEFT].copyTo(gui_image(cv::Rect(320,0,320,240)));
				small_images_color[LEFT_CENTER].copyTo(gui_image(cv::Rect(0,240,320,240)));
				small_images_color[CENTER_LEFT].copyTo(gui_image(cv::Rect(320,240,320,240)));
				small_images_color[CENTER_RIGHT].copyTo(gui_image(cv::Rect(0,480,320,240)));
				small_images_color[RIGHT_CENTER].copyTo(gui_image(cv::Rect(320,480,320,240)));
			}
			else
			{
				small_images_color[0].copyTo(gui_image(cv::Rect(0,0,320,240)));
				small_images_color[2].copyTo(gui_image(cv::Rect(0,240,320,240)));
				small_images_color[4].copyTo(gui_image(cv::Rect(0,480,320,240)));
			}
	    }
	    if (rectified_)
	    {
	    	for(int k = 12; k < gui_image.rows; k += 25 )
	    	{
	    		cv::line(gui_image, cv::Point(0, k), cv::Point(gui_image.cols, k), cv::Scalar(0,255,0), 1);
	    	}
	    }
	}

	void VrmCalibration::saveImages(const std::string& path)
	{
		if (calibration_images_[0].size() > 0)
		{
			std::string tmp_path;
			if (path.at(0) != '/')
			{
				tmp_path = ros::package::getPath("vrm3dvision") + std::string("/") + path;
			}
			else
			{
				tmp_path = path;
			}
			if (!boost::filesystem::exists(tmp_path))
			{
				if (tmp_path.at(tmp_path.size()-1) != '/')
				{
					tmp_path.push_back('/');
				}
				if (boost::filesystem::create_directories(tmp_path))
				{
					for (size_t i = 0; i < calibration_images_[0].size(); i++)
					{
						std::stringstream ss;
						ss << tmp_path << "left_" << i << ".jpg";
						cv::imwrite(ss.str(), calibration_images_[LEFT_IDX][i]);
						ss.str("");
						ss << tmp_path << "center_" << i << ".jpg";
						cv::imwrite(ss.str(), calibration_images_[CENTER_IDX][i]);
						ss.str("");
						ss << tmp_path << "right_" << i << ".jpg";
						cv::imwrite(ss.str(), calibration_images_[RIGHT_IDX][i]);
					}
				}
				else
				{
					ROS_ERROR_STREAM("Failed to create folder: " << tmp_path);
				}
			}
			else
			{
				ROS_ERROR_STREAM("Folder already exists.. Images will not be saved..! - try new path to unexisting folder");
			}
		}

	}

	void VrmCalibration::saveImagePoints(const std::string& filename)
	{
		std::ofstream file;
		file.open(filename.c_str());
		if (file.is_open())
		{
			// Write number of images on first line
			file << image_points_[LEFT_IDX].size() << " " << image_size_.width << " " << image_size_.height << std::endl;
			file << std::fixed;
			file << std::setprecision(12);
			for (int cam_idx = LEFT_IDX; cam_idx <= RIGHT_IDX; cam_idx++)
			{
				for (size_t img_idx = 0; img_idx < image_points_[cam_idx].size(); img_idx++)
				{
					for (size_t pt_idx = 0; pt_idx < image_points_[cam_idx][img_idx].size(); pt_idx++)
					{
						file << image_points_[cam_idx][img_idx][pt_idx].x << " " << image_points_[cam_idx][img_idx][pt_idx].y << " ";
					}
					file << std::endl;
				}
			}
			file.close();
		}
		else
		{
			ROS_ERROR("Unable to open file for saving image points");
		}
	}

	bool VrmCalibration::loadImagePoints(const std::string filename)
	{
		bool ret = true;
		// Clear images points
		image_points_.clear();
		image_points_.resize(3);

		// Open file
		std::ifstream file (filename.c_str());
		std::string line;
		std::stringstream ss;

		if (file.is_open())
		{
			// Get number of images
			std::getline (file,line);
			ss.str(line);
			int images;
			ss >> images;
			ss >> image_size_.width;
			ss >> image_size_.height;
			if (images > 0)
			{
				ROS_INFO_STREAM("Number of images: " << images);
				for (int cam_idx = LEFT_IDX; cam_idx <= RIGHT_IDX; cam_idx++)
				{
					image_points_[cam_idx].resize(images);
					for (int img_idx = 0; img_idx < images; img_idx++)
					{
						std::getline(file, line);
						std::istringstream iss;
						iss.str(line);
						float x, y;
						while((iss >> x) && (iss >> y))
						{
							image_points_[cam_idx][img_idx].push_back(cv::Point2f(x,y));
						}
					}
				}
				file.close();
			}
			else
			{
				ROS_ERROR("File with image points in wrong format");
				ret = false;
			}

		}
		else
		{
			ROS_ERROR("Unable to open file with image points");
			ret = false;
		}
		return ret;
	}

	bool VrmCalibration::loadMatlabIntrinsic(const std::string& path)
	{
		std::vector<std::string> filenames(3);
		filenames[0] = path + "/Calib_Results_left.m";
		filenames[1] = path + "/Calib_Results_center.m";
		filenames[2] = path + "/Calib_Results_right.m";

		for (int cam = 0; cam < 3; cam++)
		{
			std::ifstream is(filenames[cam].c_str());

			double fcx, fcy;
			double ccx, ccy;
			double dist[5];

			if (is.is_open())
			{
				std::string s;
				while(getline(is, s))
				{
					if (s.size() > 2)
					{
						std::string ssub = s.substr(0,3);
						if (ssub == "fc ")
						{
							std::cout << "FC: " << s << std::endl;
							std::stringstream ss(s.substr(7,s.size()-7));
							std::string tmp;
							ss >> fcx;
							ss >> tmp;
							ss >> fcy;
						}
						else if (ssub == "cc ")
						{
							std::cout << "CC: " << s << std::endl;
							std::stringstream ss(s.substr(7,s.size()-7));
							std::string tmp;
							ss >> ccx;
							ss >> tmp;
							ss >> ccy;
						}
						else if (ssub == "kc ")
						{
							std::cout << "KC: " << s << std::endl;
							std::stringstream ss(s.substr(7,s.size()-7));
							std::string tmp;
							for (int i = 0; i < 5; i++)
							{
								ss >> dist[i];
								ss >> tmp;
							}
							std::cout << "D0: " << dist[0] << " D1: " << dist[1] << " D2: " << dist[2] << " D3: " << dist[3] << " D4: " << dist[4] << std::endl;
						}
					}
				}
				camera_matrixes_[cam] = cv::Mat::eye(3,3,CV_64F);
				camera_matrixes_[cam].at<double>(0,0) = fcx;
				camera_matrixes_[cam].at<double>(1,1) = fcy;
				camera_matrixes_[cam].at<double>(0,2) = ccx;
				camera_matrixes_[cam].at<double>(1,2) = ccy;
				dist_coeffs_[cam] = cv::Mat::zeros(1,5,CV_64F);
				for (int i = 0; i < 5; i++)
				{
					dist_coeffs_[cam].at<double>(0,i) = dist[i];
				}
			}
			else
			{
				std::cerr << "Unable to open calibration file..!" << std::endl;
				return false;
			}
		}

		return true;
	}

	void VrmCalibration::findCorners(int cam_idx)
	{
		// Set image_size
		image_size_.width = calibration_images_[LEFT_IDX][0].cols;
		image_size_.height = calibration_images_[LEFT_IDX][0].rows;

		image_points_[cam_idx].resize(calibration_images_[cam_idx].size());
		int good_images = 0;
		for (size_t img_idx = 0; img_idx < calibration_images_[cam_idx].size(); img_idx++)
		{
			bool found = false;
			std::vector<cv::Point2f> ptvec;

			if (grid_type_ == CHESSBOARD_GRID)
			{
				found = cv::findChessboardCorners( calibration_images_[cam_idx][img_idx], calib_board_size_, ptvec, cv::CALIB_CB_ADAPTIVE_THRESH );
				if( found )
				{
					// Find corners with subpixel accuracy
					cv::cornerSubPix( calibration_images_[cam_idx][img_idx], ptvec, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 60, 0.001 ));
				}
			}
			else if(grid_type_ == ACIRCLE_GRID)
			{
				found = cv::findCirclesGrid( calibration_images_[cam_idx][img_idx], calib_board_size_, ptvec, cv::CALIB_CB_ASYMMETRIC_GRID );
			}

			if(found)
			{
				good_images++;

				// Save the found points
				image_points_[cam_idx][img_idx].resize(ptvec.size());
				std::copy(ptvec.begin(), ptvec.end(), image_points_[cam_idx][img_idx].begin());
			}
		}

		ROS_INFO_STREAM("Found corners in " << good_images << " for camera " << cam_idx);

	}

	void VrmCalibration::calcTransformations()
	{

		// Calculate Transformation between LEFT-CENTER and LEFT-RIGHT(anchor)
		cv::Mat R01;

		//cv::gemm(R[LEFT_CENTER], R[LEFT_RIGHT], 1, 0, 0, R01, CV_GEMM_A_T);
		cv::gemm(R[LEFT_RIGHT], R[LEFT_CENTER], 1, 0, 0, R01, CV_GEMM_B_T);

		T_[0] = cv::Mat::eye(4,4,CV_64F);
		R01.copyTo(T_[0](cv::Rect(0,0,3,3)));

		// Calculate Transformation between RIGHT-CENTER and LEFT-RIGHT(anchor)
		cv::Mat R02;
		cv::gemm(R[RIGHT_LEFT], R[RIGHT_CENTER], 1, 0, 0, R02, CV_GEMM_B_T);
		T_[1] = cv::Mat::eye(4,4,CV_64F);
		R02.copyTo(T_[1](cv::Rect(0,0,3,3)));

		double x_off = sqrt(pow(T13.at<double>(0,0),2.0) + pow(T13.at<double>(1,0),2.0) + pow(T13.at<double>(2,0),2.0));

		T_[1].at<double>(0,3) = x_off;

//		// Calculate Transformation between CENTER-RIGHT and LEFT-RIGHT(anchor)
//
//		// 1. Calc T from Center rect to original center
//		cv::Mat R_center = R[CENTER_RIGHT].t();
//		cv::Mat T_center = cv::Mat::eye(4,4,CV_64F);
//		R_center.copyTo(T_center(cv::Rect(0,0,3,3)));
//
//		// 2. Calc T12 inverse
//		cv::Mat T12_inv = cv::Mat::eye(4,4,CV_64F);
//		cv::Mat R12_t = R12.t();
//		R12_t.copyTo(T12_inv(cv::Rect(0,0,3,3)));
//		cv::Mat t12_inv = -R12_t * T12;
//		for (int i = 0; i < 3; i++)
//			T12_inv.at<double>(i,3) = t12_inv.at<double>(i,0);
//
//
//		cv::Mat T_lr = cv::Mat::eye(4,4,CV_64F);
//		R[LEFT_RIGHT].copyTo(T_lr(cv::Rect(0,0,3,3)));
//
//		T_[1] = T_lr * T12_inv * T_center;

	}

	void VrmCalibration::computeT()
	{
		// Calculate new R and T between the 3 sets (Left camera in the Left-right rectified pair is the reference)
		ROS_INFO_STREAM("ComputeT started");

		// New method
		calcTransformations();

		recalculate_rectification_ = false;
		transformation_estimated_ = true;

		ROS_INFO_STREAM("T LC2LR:\n" << T_[0]);
		ROS_INFO_STREAM("T RC2LR:\n" << T_[1]);

		ROS_INFO_STREAM("ComputeT done");
	}



	void VrmCalibration::calibrateThreeSeperate()
	{
		ROS_INFO("calibrateThreeSeperate started");

		cv::Mat Q;
		cv::stereoRectify( camera_matrixes_[LEFT_IDX], dist_coeffs_[LEFT_IDX], camera_matrixes_[RIGHT_IDX], dist_coeffs_[RIGHT_IDX],
	                   image_size_, R13, T13, R[LEFT_RIGHT], R[RIGHT_LEFT], P[LEFT_RIGHT], P[RIGHT_LEFT], Q,
	                   cv::CALIB_ZERO_DISPARITY, alpha_[0], image_size_);

		cv::stereoRectify( camera_matrixes_[LEFT_IDX], dist_coeffs_[LEFT_IDX], camera_matrixes_[CENTER_IDX], dist_coeffs_[CENTER_IDX],
	                   image_size_, R12, T12, R[LEFT_CENTER], R[CENTER_LEFT], P[LEFT_CENTER], P[CENTER_LEFT], Q,
	                   cv::CALIB_ZERO_DISPARITY, alpha_[1], image_size_);

		cv::stereoRectify( camera_matrixes_[RIGHT_IDX], dist_coeffs_[RIGHT_IDX], camera_matrixes_[CENTER_IDX], dist_coeffs_[CENTER_IDX],
	                   image_size_, R32, T32, R[RIGHT_CENTER], R[CENTER_RIGHT], P[RIGHT_CENTER], P[CENTER_RIGHT], Q,
	                   cv::CALIB_ZERO_DISPARITY, alpha_[2], image_size_);

		// Init retification maps
		cv::initUndistortRectifyMap(camera_matrixes_[LEFT_IDX], dist_coeffs_[LEFT_IDX], R[LEFT_RIGHT], P[LEFT_RIGHT], image_size_, CV_16SC2, map_x_[LEFT_RIGHT], map_y_[LEFT_RIGHT]);
		cv::initUndistortRectifyMap(camera_matrixes_[LEFT_IDX], dist_coeffs_[LEFT_IDX], R[LEFT_CENTER], P[LEFT_CENTER], image_size_, CV_16SC2, map_x_[LEFT_CENTER], map_y_[LEFT_CENTER]);
		cv::initUndistortRectifyMap(camera_matrixes_[CENTER_IDX], dist_coeffs_[CENTER_IDX], R[CENTER_LEFT], P[CENTER_LEFT], image_size_, CV_16SC2, map_x_[CENTER_LEFT], map_y_[CENTER_LEFT]);
		cv::initUndistortRectifyMap(camera_matrixes_[CENTER_IDX], dist_coeffs_[CENTER_IDX], R[CENTER_RIGHT], P[CENTER_RIGHT], image_size_, CV_16SC2, map_x_[CENTER_RIGHT], map_y_[CENTER_RIGHT]);
		cv::initUndistortRectifyMap(camera_matrixes_[RIGHT_IDX], dist_coeffs_[RIGHT_IDX], R[RIGHT_LEFT], P[RIGHT_LEFT], image_size_, CV_16SC2, map_x_[RIGHT_LEFT], map_y_[RIGHT_LEFT]);
		cv::initUndistortRectifyMap(camera_matrixes_[RIGHT_IDX], dist_coeffs_[RIGHT_IDX], R[RIGHT_CENTER], P[RIGHT_CENTER], image_size_, CV_16SC2, map_x_[RIGHT_CENTER], map_y_[RIGHT_CENTER]);


		ROS_WARN_STREAM("F lr: " << P[LEFT_RIGHT].at<double>(0,0));
		ROS_WARN_STREAM("F rl: " << P[RIGHT_LEFT].at<double>(0,0));
		ROS_WARN_STREAM("F lc: " << P[LEFT_CENTER].at<double>(0,0));
		ROS_WARN_STREAM("F cl: " << P[CENTER_LEFT].at<double>(0,0));
		ROS_WARN_STREAM("F rc: " << P[RIGHT_CENTER].at<double>(0,0));
		ROS_WARN_STREAM("F cr: " << P[CENTER_RIGHT].at<double>(0,0));

		recalculate_rectification_ = false;

		rectified_ = true;

		ROS_INFO("calibrateThreeSeperate done");
	}

	void VrmCalibration::calibrate()
	{
		ROS_INFO_STREAM("Starting Calibration using " << calibration_images_[0].size() << " image sets...");
		// Find chessboard in the original images
		bool failed = false;
	    int threads = 3;
		omp_set_dynamic(0);     // Explicitly disable dynamic teams
		omp_set_num_threads(threads); // Use 3 threads for all consecutive parallel regions
		#pragma omp parallel for
		for (int cam_idx = LEFT_IDX; cam_idx <= RIGHT_IDX; cam_idx++)
		{
			if (!load_image_points_from_file_)
			{
				// Find corners
				findCorners(cam_idx);
			}

			// Perform single camera calibration
			std::vector<std::vector<cv::Point2f> > img_pts;
			std::vector<std::vector<cv::Point3f> > obj_pts;
			int totalPoints = 0;
			for (size_t img_idx = 0; img_idx < image_points_[cam_idx].size(); img_idx++)
			{
				if (!image_points_[cam_idx][img_idx].empty())
				{
					img_pts.push_back(image_points_[cam_idx][img_idx]);
					totalPoints += (int)image_points_[cam_idx][img_idx].size();
				}
			}

			if (img_pts.size() < 3)
			{
				ROS_ERROR_STREAM("Not enough views for calibrating camera " << cam_idx << " - Got " << img_pts.size() << " images - 3 is needed..");
				failed = true;
			}
			else
			{
				obj_pts.push_back(chessboard_corners_);
				obj_pts.resize(img_pts.size(),obj_pts[0]);

				std::vector<cv::Mat> rvecs, tvecs;
				double err = cv::calibrateCamera(obj_pts, img_pts, image_size_, camera_matrixes_[cam_idx], dist_coeffs_[cam_idx], rvecs, tvecs, cv::CALIB_FIX_K3, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 60, DBL_EPSILON));
				bool ok = cv::checkRange(camera_matrixes_[cam_idx]) && checkRange(dist_coeffs_[cam_idx]);
				if(!ok)
				{
					ROS_INFO("Error: camera %d was not calibrated\n", cam_idx);
					failed = true;
				}
				else
				{
					ROS_INFO_STREAM("Cam " << cam_idx << " calibrated an error of: " << (err/totalPoints) << " using " << img_pts.size() << " images");
					ROS_INFO_STREAM("Camera Matrix: " << camera_matrixes_[cam_idx]);
					ROS_INFO_STREAM("Dist coeffs: " << dist_coeffs_[cam_idx]);
				}
			}
		}
		if (failed)
		{
			ROS_ERROR_STREAM("Unable to calibrate cameras..");
			return;
		}

		// Perform stereo calibration
		doStereoCalibration();

		// Do three seperate calibrations
		calibrateThreeSeperate();

		saveImagePoints(image_point_file_.c_str());

		ROS_INFO_STREAM("Calibration done");
	}

	void VrmCalibration::doStereoCalibration()
	{
		// Perform stereo calibration between cam 1-3, 1-2 and 2-3
		for (int i = LEFT_IDX; i <= RIGHT_IDX; i++)
		{
			int anchor_idx, slave_idx;
			if (i == LEFT_IDX)
			{
				anchor_idx = LEFT_IDX;
				slave_idx = RIGHT_IDX;
			}
			else if (i == CENTER_IDX)
			{
				anchor_idx =  LEFT_IDX;
				slave_idx = CENTER_IDX;
			}
			else if (i == RIGHT_IDX)
			{
				anchor_idx = RIGHT_IDX;
				slave_idx = CENTER_IDX;
			}

			int total_points = 0;
		    std::vector<std::vector<cv::Point3f> > obj_pts;
			std::vector<std::vector<cv::Point2f> > left_img_pts, right_img_pts;

			for (size_t img_idx = 0; img_idx < std::min(image_points_[anchor_idx].size(), image_points_[slave_idx].size()); img_idx++)
			{
				if (!image_points_[anchor_idx][img_idx].empty() && !image_points_[slave_idx][img_idx].empty())
				{
					left_img_pts.push_back(image_points_[anchor_idx][img_idx]);
					right_img_pts.push_back(image_points_[slave_idx][img_idx]);
					total_points += image_points_[anchor_idx][img_idx].size();
				}
			}

			if (left_img_pts.size() < 3)
			{
				ROS_ERROR_STREAM("Not enough shared views for calibrating camera " << anchor_idx << " - Got " << left_img_pts.size() << " images - 3 is needed..");
			}

			obj_pts.push_back(chessboard_corners_);
			obj_pts.resize(left_img_pts.size(),obj_pts[0]);

			cv::Mat R, T, E, F;

	        double err = cv::stereoCalibrate(	obj_pts, left_img_pts, right_img_pts, camera_matrixes_[anchor_idx], dist_coeffs_[anchor_idx],
												camera_matrixes_[slave_idx], dist_coeffs_[slave_idx],
												image_size_, R, T, E, F,
												cv::TermCriteria(cv::TermCriteria::COUNT, 100, DBL_EPSILON),
												cv::CALIB_FIX_INTRINSIC);

			ROS_INFO_STREAM("Camset " << anchor_idx << " and " << slave_idx << " calibrated - error: " << (err/pow(total_points,2.0)));
			ROS_INFO_STREAM("R: " << R);
			ROS_INFO_STREAM("T: " << T);

			if ( anchor_idx == LEFT_IDX && slave_idx == CENTER_IDX)
			{
				R12 = R;
				T12 = T;
			}
			else if (anchor_idx == LEFT_IDX && slave_idx == RIGHT_IDX)
			{
				R13 = R;
				T13 = T;
			}
			else if (anchor_idx == RIGHT_IDX && slave_idx == CENTER_IDX)
			{
				R32 = R;
				T32 = T;
			}
		}
	}

	void VrmCalibration::commitCalibration()
	{

		bool succes = true;
		if (rectified_ && transformation_estimated_)
		{
			for( int cam_idx = LEFT_IDX; cam_idx <= RIGHT_IDX; cam_idx++ )
			{
				int R_idx, P_idx;
				if (cam_idx == LEFT_IDX)
					R_idx = P_idx = LEFT_RIGHT;
				else if (cam_idx == RIGHT_IDX)
					R_idx = P_idx = RIGHT_LEFT;

				sensor_msgs::SetCameraInfoRequest cam_info;

				cam_info.camera_info.distortion_model = "plumb_bob";
				cam_info.camera_info.height = image_size_.height;
				cam_info.camera_info.width = image_size_.width;

				// K
				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						cam_info.camera_info.K.at(i*3 + j) = camera_matrixes_[cam_idx].at<double>(i,j);

				// D
				for (int i = 0; i < 5; i++)
					cam_info.camera_info.D.push_back(dist_coeffs_[cam_idx].at<double>(0,i));

				if (cam_idx != CENTER_IDX)
				{
					// R
					for (int i = 0; i < 3; i++)
						for (int j = 0; j < 3; j++)
							cam_info.camera_info.R.at(i*3 + j) = R[R_idx].at<double>(i,j);

					// P
					for (int i = 0; i < 3; i++)
						for (int j = 0; j < 4; j++)
							cam_info.camera_info.P.at(i*4 + j) = P[P_idx].at<double>(i,j);
				}
				else // Center cam
				{
					// R
					for (int i = 0; i < 3; i++)
						for (int j = 0; j < 3; j++)
							if (i==j)
								cam_info.camera_info.R.at(i*3 + j) = 1;
							else
								cam_info.camera_info.R.at(i*3 + j) = 0;

					// P
					for (int i = 0; i < 3; i++)
						for (int j = 0; j < 4; j++)
							if(j==3)
								cam_info.camera_info.P.at(i*4 + j) = 0;
							else
								cam_info.camera_info.P.at(i*4 + j) = camera_matrixes_[cam_idx].at<double>(i,j);

				}

				sensor_msgs::SetCameraInfoResponse resp;
				cam_services_[cam_idx].call(cam_info,resp);
				if (!resp.success)
				{
					succes = false;
					ROS_ERROR_STREAM("Unable to commit camera calibration for camera " << cam_idx);
				}
			}
			if (succes)
			{
				ROS_INFO("Calibration succesfully committed");
			}
		}
		else
		{
			ROS_ERROR("Calibration can not be committed before the calibration is done.!");
		}
		std::stringstream filepath;
		filepath << getenv("HOME") << "/.ros/camera_info/vrm3dcalib.yaml";
		saveToFile(filepath.str());
	}

	void VrmCalibration::saveToFile(const std::string& filename)
	{
		cv::FileStorage fs;
	    fs.open(filename.c_str(), cv::FileStorage::WRITE);
	    if (fs.isOpened())
	    {
	    	fs << "imageWidth" << image_size_.width;
		    fs << "imageHeight" << image_size_.height;

		    fs << "K_left" << camera_matrixes_[LEFT_IDX];
		    fs << "K_center" << camera_matrixes_[CENTER_IDX];
		    fs << "K_right" << camera_matrixes_[RIGHT_IDX];

		    fs << "D_left" << dist_coeffs_[LEFT_IDX];
		    fs << "D_center" << dist_coeffs_[CENTER_IDX];
		    fs << "D_right" << dist_coeffs_[RIGHT_IDX];

//		    if (use_ransac_)
//		    {
//			    fs << "T_lr_lc" << T2_[0];
//			    fs << "T_lr_cr" << T2_[1];
//		    }
//		    else
//		    {
			    fs << "T_lr_lc" << T_[0];
			    fs << "T_lr_cr" << T_[1];
//		    }

		    fs << "R_lr" << R[LEFT_RIGHT];
		    fs << "R_lc" << R[LEFT_CENTER];
		    fs << "R_cl" << R[CENTER_LEFT];
		    fs << "R_cr" << R[CENTER_RIGHT];
		    fs << "R_rl" << R[RIGHT_LEFT];
		    fs << "R_rc" << R[RIGHT_CENTER];

		    fs << "P_lr" << P[LEFT_RIGHT];
		    fs << "P_lc" << P[LEFT_CENTER];
		    fs << "P_cl" << P[CENTER_LEFT];
		    fs << "P_cr" << P[CENTER_RIGHT];
		    fs << "P_rl" << P[RIGHT_LEFT];
		    fs << "P_rc" << P[RIGHT_CENTER];
	    }
	    else
	    {
	    	ROS_ERROR_STREAM("Unable to save camera calibration in OpenCV format");
	    }
	}

	void VrmCalibration::imageCb(const sensor_msgs::ImageConstPtr& left_input, const sensor_msgs::ImageConstPtr& center_input, const sensor_msgs::ImageConstPtr& right_input)
	{
		// Copy images to OpenCV
		std::vector<cv::Mat> input_images(3);

	    try {
	      input_images[LEFT_IDX] = cv_bridge::toCvCopy(left_input, "mono8")->image;
	      input_images[CENTER_IDX] = cv_bridge::toCvCopy(center_input, "mono8")->image;
	      input_images[RIGHT_IDX] = cv_bridge::toCvCopy(right_input, "mono8")->image;
	    }
	    catch (cv_bridge::Exception& e) {
	      ROS_ERROR("Unable to copy images to OpenCV");
	    }

	    // Update GUI image
	    cv::Mat gui_image(720, 640, CV_8UC3);
	    updateGUI(input_images, gui_image);
	    cv::imshow(GUI_NAME,gui_image);

	    if (save_next_image_set_) // Save images
	    {
	    	if (!rectified_)
	    	{
		    	for (int idx = LEFT_IDX; idx <= RIGHT_IDX; idx++)
		    	{
		    		calibration_images_[idx].push_back(input_images[idx]);
		    	}
		    	ROS_INFO_STREAM("Saving image set - total images: " << calibration_images_[LEFT_IDX].size());
	    	}
	    	else if (rectified_)
	    	{
		    	for (int idx = LEFT_IDX; idx <= RIGHT_IDX; idx++)
		    	{
		    		transformation_images_[idx].push_back(input_images[idx]);
		    	}
		    	ROS_INFO_STREAM("Saving image set for transformation estimation - total images: " << transformation_images_[LEFT_IDX].size());
	    	}
	    	save_next_image_set_ = false;
	    }
	    if (test_rectification_)
	    {
	    	testRectification(input_images);
	    	test_rectification_ = false;
	    }
	}

	void VrmCalibration::testRectification(const std::vector<cv::Mat>& input_images)
	{
		std::vector<std::vector<cv::Point2f> > ptvec(3);
		std::vector<cv::Mat> images_rect(3);
		int finds = 0;
		for (int idx = LEFT_IDX; idx <= RIGHT_IDX; idx++)
		{
            cv::remap(input_images[idx], images_rect[idx], map_x_[idx], map_y_[idx], cv::INTER_LINEAR);

			bool found = cv::findChessboardCorners( images_rect[idx], calib_board_size_, ptvec[idx], cv::CALIB_CB_ADAPTIVE_THRESH );

			if( found )
			{
				cv::cornerSubPix( images_rect[idx], ptvec[idx], cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
				finds++;
			}
		}

		// Calculate error
		if (finds == 3)
		{
			double x_lc(0), y_lc(0), x_lr(0), y_lr(0), x_rc(0), y_rc(0);
			for (size_t i = 0; i < ptvec[0].size(); i++)
			{
				x_lc += fabs(ptvec[LEFT_IDX][i].x - ptvec[CENTER_IDX][i].x);
				y_lc += fabs(ptvec[LEFT_IDX][i].y - ptvec[CENTER_IDX][i].y);

				x_lr += fabs(ptvec[LEFT_IDX][i].x - ptvec[RIGHT_IDX][i].x);
				y_lr += fabs(ptvec[LEFT_IDX][i].y - ptvec[RIGHT_IDX][i].y);

				x_rc += fabs(ptvec[RIGHT_IDX][i].x - ptvec[CENTER_IDX][i].x);
				y_rc += fabs(ptvec[RIGHT_IDX][i].y - ptvec[CENTER_IDX][i].y);
			}
			x_lc /= ptvec[0].size();
			y_lc /= ptvec[0].size();
			x_lr /= ptvec[0].size();
			y_lr /= ptvec[0].size();
			x_rc /= ptvec[0].size();
			y_rc /= ptvec[0].size();

			ROS_INFO_STREAM("Rectification test");
			ROS_INFO_STREAM("Left to center: X : " << x_lc << " Y: " << y_lc);
			ROS_INFO_STREAM("Left to right: X : " << x_lr << " Y: " << y_lr);
			ROS_INFO_STREAM("Right to center: X : " << x_rc << " Y: " << y_rc);

		}
		else
		{
			ROS_INFO("Did not find chessboard in all three images");
		}
	}


	void VrmCalibration::calcChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners)
	{
	    corners.resize(0);
	    if (grid_type_ == CHESSBOARD_GRID)
	    {
	    	for( int i = 0; i < boardSize.height; i++ )
	    		for( int j = 0; j < boardSize.width; j++ )
	    			corners.push_back(cv::Point3f(float(j*squareSize), float(i*squareSize), 0));
	    }
	    else if (grid_type_ == ACIRCLE_GRID)
	    {
			for( int i = 0; i < boardSize.height; i++ )
				for( int j = 0; j < boardSize.width; j++ )
					corners.push_back(cv::Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
	    }
	}


	void computeCentroids(const std::vector<cv::Point3d>& points, cv::Point3d& mean)
	{
		double sumX = 0, sumY = 0, sumZ = 0;

			for(size_t i = 0; i < points.size(); i++)
			{
				sumX += points[i].x;
				sumY += points[i].y;
				sumZ += points[i].z;
			}
			mean.x = sumX / points.size();
			mean.y = sumY / points.size();
			mean.z = sumZ / points.size();
	}

	void demeanPoints(const std::vector<cv::Point3d>& src, std::vector<cv::Point3d>& dst, const cv::Point3d& mean)
	{
		cv::Point3f temp;
		for(size_t i = 0; i < src.size(); i++)
		{
			temp.x = src[i].x - mean.x;
			temp.y = src[i].y - mean.y;
			temp.z = src[i].z - mean.z;
			dst.push_back(temp);
		}
	}


	void computeCorrelationMatrix(const std::vector<cv::Point3d>& oldPointsDemean, const std::vector<cv::Point3d>& newPointsDemean, cv::Mat& corrMat)
	{
		cv::Mat temp(3,3,CV_64F);

		for(int i = 0; i < 3; i++)
			{
				for(int j = 0; j < 3; j++)
				{
					temp.at<double>(i,j) = 0;

					for(size_t k = 0; k < oldPointsDemean.size(); k++)
					{
						if(i == 0)
						{
							if(j == 0)
								temp.at<double>(i,j) += oldPointsDemean[k].x * newPointsDemean[k].x;
							else if(j == 1)
								temp.at<double>(i,j) += oldPointsDemean[k].x * newPointsDemean[k].y;
							else if(j == 2)
								temp.at<double>(i,j) += oldPointsDemean[k].x * newPointsDemean[k].z;
						}

						if(i == 1)
						{
							if(j == 0)
								temp.at<double>(i,j) += oldPointsDemean[k].y * newPointsDemean[k].x;
							else if(j == 1)
								temp.at<double>(i,j) += oldPointsDemean[k].y * newPointsDemean[k].y;
							else if(j == 2)
								temp.at<double>(i,j) += oldPointsDemean[k].y * newPointsDemean[k].z;
						}

						if(i == 2)
						{
							if(j == 0)
								temp.at<double>(i,j) += oldPointsDemean[k].z * newPointsDemean[k].x;
							else if(j == 1)
								temp.at<double>(i,j) += oldPointsDemean[k].z * newPointsDemean[k].y;
							else if(j == 2)
								temp.at<double>(i,j) += oldPointsDemean[k].z * newPointsDemean[k].z;
						}
					}
				}
			}
		corrMat = temp;
	}

	void computeRotation(const cv::Mat& corrMat, cv::Mat& R1, cv::Mat& R2)
	{
		cv::SVD decomp = cv::SVD(corrMat);
		R1 = decomp.u * decomp.vt;			// Orthogonal Procrustes algorithm
		R2 = decomp.vt.t() * decomp.u.t();	// Matlab code
	}

	void computeTranslation(const cv::Point3d& oldCentroid, const cv::Point3d& newCentroid, const cv::Mat& R, cv::Point3d& T1, cv::Point3d& T2)
	{
		cv::Point3d oldCentroidRot;
		oldCentroidRot.x = oldCentroid.x * R.at<double>(0,0) + oldCentroid.y * R.at<double>(0,1) + oldCentroid.z * R.at<double>(0,2);
		oldCentroidRot.y = oldCentroid.x * R.at<double>(1,0) + oldCentroid.y * R.at<double>(1,1) + oldCentroid.z * R.at<double>(1,2);
		oldCentroidRot.z = oldCentroid.x * R.at<double>(2,0) + oldCentroid.y * R.at<double>(2,1) + oldCentroid.z * R.at<double>(2,2);

		T1 = newCentroid - oldCentroidRot;
		T2 = newCentroid - oldCentroid;
	}

	int computeInliers(const std::vector<cv::Point3d>& oldPointsTransformed, const std::vector<cv::Point3d>& newPoints, double threshold, std::vector<int>& inlierIndexes, double& avgError)
	{
		int inliers = 0;
		double sum = 0;
		for(size_t i = 0; i < newPoints.size(); i++)
		{
			cv::Point3d diff = newPoints[i] - oldPointsTransformed[i];
			double euclidean = sqrt(pow(diff.x,2.0) + pow(diff.y,2.0) + pow(diff.z,2.0));
			if(euclidean < threshold)
			{
				inlierIndexes.push_back(i);
				inliers++;
				sum += euclidean;
			}
		}
		avgError = sum/inlierIndexes.size();

		return inliers;
	}

	void getRandomPoints(const std::vector<cv::Point3d>& oldPoints, const std::vector<cv::Point3d>& newPoints, std::vector<cv::Point3d>& randomOldPoints, std::vector<cv::Point3d>& randomNewPoints)
	{
		std::vector<int> usedIndex;
		while(randomNewPoints.size() < 3)
		{
			bool newIndex = true;
			int randomIndex = rand()%oldPoints.size();
			for(size_t i = 0; i < usedIndex.size(); i++)
				if(randomIndex == usedIndex[i])
					newIndex = false;

			if(newIndex)
			{
				randomOldPoints.push_back(oldPoints[randomIndex]);
				randomNewPoints.push_back(newPoints[randomIndex]);
				usedIndex.push_back(randomIndex);
			}
		}
	}

	void VrmCalibration::computeTransformation(std::vector<cv::Point3d>& oldPoints, std::vector<cv::Point3d>& newPoints, cv::Mat& transform)
	{
		// Initialize parameters for RANSAC
		std::vector<int> bestInlierIndexes;
		int maxInliers = 0;
		cv::Mat bestT;
		double bestAvgError;
		int maxRuns = 1000;
		double threshold = 0.0001;

		for(int run = 0; run < maxRuns; run++)
		{
			// Choose random points
			std::vector<cv::Point3d> randomOldPoints, randomNewPoints;
			getRandomPoints(oldPoints, newPoints, randomOldPoints, randomNewPoints);

			// Compute transformation
			cv::Mat newT;
			computeTransform(randomNewPoints, randomOldPoints, newT);

			// Apply transform
			std::vector<cv::Point3d> newPointsTransformed;
			transformPoints(newPoints, newT, newPointsTransformed);

			// Count inliers
			std::vector<int> inlierIndexes;
			double avgError;
			int inliers = computeInliers(newPointsTransformed, oldPoints, threshold, inlierIndexes, avgError);

			// Update max inliers
			if(inliers > maxInliers)
			{
				bestInlierIndexes = inlierIndexes;
				maxInliers = inliers;
				bestAvgError = avgError;
				bestT = newT;
			}
		}

		// Reestimate tranform based on inliers
		std::vector<cv::Point3d> oldPointInliers, newPointInliers;
		for(size_t i = 0; i < bestInlierIndexes.size(); i++)
		{
			oldPointInliers.push_back(oldPoints[bestInlierIndexes[i]]);
			newPointInliers.push_back(newPoints[bestInlierIndexes[i]]);
		}

		computeTransform(newPointInliers, oldPointInliers, bestT);

		std::vector<cv::Point3d> newPointsInliersTransformed;
		transformPoints(newPointInliers, bestT, newPointsInliersTransformed);

		std::vector<int> inlierIndexes;
		int inliers = computeInliers(newPointsInliersTransformed, oldPointInliers, threshold, inlierIndexes, bestAvgError);
		std::cout << std::endl << "!!!!!!!!!!!!! REESTIMATED RESULT !!!!!!!!!!!!!" << std::endl;
		std::cout << "Max inliers: " << inliers << "/" << oldPoints.size() << std::endl;
		std::cout << "New transform: " << bestT << std::endl;
		std::cout << "Average euclidean distance: " << bestAvgError << std::endl;

		bestT.copyTo(transform);
	}

	void VrmCalibration::transformPoints(const std::vector<cv::Point3d>& inPoints, const cv::Mat& T, std::vector<cv::Point3d>& outPoints)
	{
		for(size_t i = 0; i < inPoints.size(); i++)
		{
			cv::Point3d temp;
			temp.x = (T.at<double>(0,0)*inPoints[i].x + T.at<double>(0,1)*inPoints[i].y + T.at<double>(0,2)*inPoints[i].z) + T.at<double>(0,3);
			temp.y = (T.at<double>(1,0)*inPoints[i].x + T.at<double>(1,1)*inPoints[i].y + T.at<double>(1,2)*inPoints[i].z) + T.at<double>(1,3);
			temp.z = (T.at<double>(2,0)*inPoints[i].x + T.at<double>(2,1)*inPoints[i].y + T.at<double>(2,2)*inPoints[i].z) + T.at<double>(2,3);
			outPoints.push_back(temp);
		}
	}

	void VrmCalibration::computeTransform(const std::vector<cv::Point3d>& oldPoints, const std::vector<cv::Point3d>& newPoints, cv::Mat& T)
	{
		cv::Point3d oldCentroid, newCentroid;
		computeCentroids(oldPoints, oldCentroid);
		computeCentroids(newPoints, newCentroid);

		std::vector<cv::Point3d> oldPointsDemean, newPointsDemean;
		demeanPoints(oldPoints, oldPointsDemean, oldCentroid);
		demeanPoints(newPoints, newPointsDemean, newCentroid);

		cv::Mat corrMat(3,3,CV_64F);
		computeCorrelationMatrix(oldPointsDemean, newPointsDemean, corrMat);

		cv::Mat R1, R2;
		computeRotation(corrMat, R1, R2);

		cv::Point3d T1, T2;
		computeTranslation(oldCentroid, newCentroid, R2, T1, T2);

		// Construct T
		T = cv::Mat::eye(4,4,CV_64F);
		for (int x = 0; x < 3; x++)
		{
			for (int y = 0; y < 4; y++)
			{
				if (y < 3)
					T.at<double>(x,y) = R2.at<double>(x,y);
				else if (x == 0)
					T.at<double>(x,y) = T1.x;
				else if (x == 1)
					T.at<double>(x,y) = T1.y;
				else
					T.at<double>(x,y) = T1.z;
			}
		}
	}

//	void VrmCalibration::computeT()
//	{
//		// Calculate new R and T between the 3 sets (Left camera in the Left-right rectified pair is the reference)
//		ROS_INFO_STREAM("ComputeT started");
//
//		// Rectify images and find chessboard corners
//		for (int cam_idx = LEFT_IDX; cam_idx <= RIGHT_IDX; cam_idx++)
//		{
//			for (int ref_idx = cam_idx*2; ref_idx <= cam_idx*2+1; ref_idx++)
//			{
//				image_points_transformation_[ref_idx].resize(transformation_images_[cam_idx].size());
//				for (size_t img_idx = 0; img_idx < transformation_images_[cam_idx].size(); img_idx++)
//				{
//					cv::Mat img_rect;
//					cv::remap(transformation_images_[cam_idx][img_idx], img_rect, map_x_[ref_idx], map_y_[ref_idx], CV_INTER_LINEAR);
//					bool found = false;
//					std::vector<cv::Point2f> ptvec;
//
//					if (grid_type_ == CHESSBOARD_GRID)
//					{
//						found = cv::findChessboardCorners( img_rect, calib_board_size_, ptvec, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
//						if( found )
//						{
//							// Find corners with subpixel accuracy
//							cv::cornerSubPix( img_rect, ptvec, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 60, 0.001 ));
//							cv::drawChessboardCorners(img_rect, calib_board_size_, ptvec, found);
//
//							std::stringstream ss;
//							ss << getenv("HOME") << "/test_" << ref_idx << ".jpg";
//
//							cv::imwrite(ss.str(), img_rect);
//						}
//					}
//					else if(grid_type_ == ACIRCLE_GRID)
//					{
//						found = cv::findCirclesGrid( img_rect, calib_board_size_, ptvec, cv::CALIB_CB_ASYMMETRIC_GRID );
//					}
//
//					if(found)
//					{
//						// Save the found points
//						image_points_transformation_[ref_idx][img_idx].resize(ptvec.size());
//						std::copy(ptvec.begin(), ptvec.end(), image_points_transformation_[ref_idx][img_idx].begin());
//					}
//				}
//			}
//		}
//
//		std::vector<std::vector<cv::Point3d> > points_3d(3);
//		for (int cam_idx = LEFT_IDX; cam_idx <= RIGHT_IDX; cam_idx++)
//		{
//			int anchor_idx, slave_idx;
//			if (cam_idx == LEFT_IDX)
//			{
//				anchor_idx = LEFT_RIGHT;
//				slave_idx = RIGHT_LEFT;
//			}
//			else if (cam_idx == CENTER_IDX)
//			{
//				anchor_idx = LEFT_CENTER;
//				slave_idx = CENTER_LEFT;
//			}
//			else
//			{
//				anchor_idx = RIGHT_CENTER;
//				slave_idx = CENTER_RIGHT;
//			}
//
//			std::vector<cv::Point2f> anchor_img_pts, slave_img_pts;
//			for (size_t img_idx = 0; img_idx < std::min(image_points_transformation_[anchor_idx].size(), image_points_transformation_[slave_idx].size()); img_idx++)
//			{
//				if (!image_points_transformation_[0][img_idx].empty() && !image_points_transformation_[1][img_idx].empty() && !image_points_transformation_[2][img_idx].empty() &&
//						!image_points_transformation_[3][img_idx].empty() && !image_points_transformation_[4][img_idx].empty() && !image_points_transformation_[5][img_idx].empty())
//				{
//					anchor_img_pts.insert(anchor_img_pts.end(), image_points_transformation_[anchor_idx][img_idx].begin(), image_points_transformation_[anchor_idx][img_idx].end());
//					slave_img_pts.insert(slave_img_pts.end(), image_points_transformation_[slave_idx][img_idx].begin(), image_points_transformation_[slave_idx][img_idx].end());
//				}
//			}
//
//			ROS_INFO_STREAM("anchor_img_pts size: " << anchor_img_pts.size() << " slave_img_pts size: " << slave_img_pts.size());
//
//			cv::Mat_<float> points_homo;
//			cv::triangulatePoints(P[anchor_idx], P[slave_idx], anchor_img_pts, slave_img_pts, points_homo);
//
//			for (int i = 0; i < points_homo.cols; i++)
//			{
//				cv::Point3d pt;
//				pt.x = points_homo.at<float>(0,i) / points_homo.at<float>(3,i);
//				pt.y = points_homo.at<float>(1,i) / points_homo.at<float>(3,i);
//				pt.z = points_homo.at<float>(2,i) / points_homo.at<float>(3,i);
//				points_3d[cam_idx].push_back(pt);
//			}
//
//		}
//
//		// New method
//		calcTransformations();
//
//		// Compute transformation between the three rectified camera sets
//		for (int i = 0; i < 2; i++)
//		{
//			int idx_anchor, idx_slave;
//			if (i == 0) // Left-right and Left-center
//			{
//				idx_anchor = 0;
//				idx_slave = 1;
//				ROS_INFO_STREAM("Transformation between LEFT-RIGHT and LEFT-CENTER..");
//
//			}
//			else if (i == 1) // Left-right to center-right
//			{
//				idx_anchor = 0;
//				idx_slave = 2;
//				ROS_INFO_STREAM("Transformation between LEFT-RIGHT and RIGHT-CENTER..");
//			}
//
//			//computeTransform(points_3d[idx_slave], points_3d[idx_anchor], T2_[i]);
//			computeTransformation(points_3d[idx_anchor], points_3d[idx_slave], T2_[i]);
//
//			// Calculate error using VIS2 method
//			std::vector<cv::Point3d> transformed_points;
//			transformPoints(points_3d[idx_slave], T2_[i], transformed_points);
//
//			double x_err_abs(0), y_err_abs(0), z_err_abs(0);
//			for (size_t j = 0; j < transformed_points.size(); j++)
//			{
////				ROS_INFO_STREAM("Anchor:      " << points_3d[idx_anchor][i]);
////				ROS_INFO_STREAM("SlaveTrans:  " << transformed_points[i]);
////				ROS_INFO_STREAM("Slave:       " << points_3d[idx_slave][i]);
//
//				x_err_abs += fabs(points_3d[idx_anchor][j].x - transformed_points[j].x);
//				y_err_abs += fabs(points_3d[idx_anchor][j].y - transformed_points[j].y);
//				z_err_abs += fabs(points_3d[idx_anchor][j].z - transformed_points[j].z);
//
//				x_err_abs /= transformed_points.size();
//				y_err_abs /= transformed_points.size();
//				z_err_abs /= transformed_points.size();
//
//				// Calculate error using new method
//				std::vector<cv::Point3d> transformed_points2;
//				transformPoints(points_3d[idx_slave], T_[i], transformed_points2);
//
//				double x_err_abs2(0), y_err_abs2(0), z_err_abs2(0);
//				for (size_t k = 0; k < transformed_points2.size(); k++)
//				{
//					x_err_abs2 += fabs(points_3d[idx_anchor][k].x - transformed_points2[k].x);
//					y_err_abs2 += fabs(points_3d[idx_anchor][k].y - transformed_points2[k].y);
//					z_err_abs2 += fabs(points_3d[idx_anchor][k].z - transformed_points2[k].z);
//				}
//				x_err_abs2 /= transformed_points2.size();
//				y_err_abs2 /= transformed_points2.size();
//				z_err_abs2 /= transformed_points2.size();
//
//
//				ROS_INFO_STREAM("Vis2 method:");
//				ROS_INFO_STREAM("T: " << T2_[i]);
//				ROS_INFO_STREAM("New method:");
//				ROS_INFO_STREAM("T: " << T_[i]);
//				//ROS_INFO_STREAM("Translation: " << );
//				ROS_INFO_STREAM("Avg. absolute (VIS2) error: x: " << x_err_abs << " y: " << y_err_abs << " z: " << z_err_abs);
//				ROS_INFO_STREAM("Avg. absolute (NEW) error: x: " << x_err_abs2 << " y: " << y_err_abs2 << " z: " << z_err_abs2);
//			}
//
//			recalculate_rectification_ = false;
//			transformation_estimated_ = true;
//
//			for (int i = 0; i < 3; i++)
//			{
//				transformation_images_[i].clear();
//				image_points_transformation_[i*2].clear();
//				image_points_transformation_[i*2+1].clear();
//			}
//
//			ROS_INFO("calibrateThreeSeperate done");
//		}
//	}


} /* namespace vrm3dvision */
