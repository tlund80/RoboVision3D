/********************************************************************************************************************
 *
 * \file                vrm_calibration_node.hpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2014-02-20
 * \version             0.1
 * \brief
 *
*********************************************************************************************************************/

#ifndef _VRM_CALIBRATION_HPP_
#define _VRM_CALIBRATION_HPP_

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <ros/package.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

// Standard libraries
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

// OPENCV Includes
#include <opencv2/opencv.hpp>

#include <opencv2/highgui/highgui.hpp>

// BOOST
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

// OpenMP includes
#include <omp.h>

#define GUI_NAME "Calibration GUI"
#define ACIRCLE_GRID 10
#define CHESSBOARD_GRID 11

namespace vrm3dvision {

	/** @class VrmCalibration
	  *
	  */
	class VrmCalibration
	{
		public:
			/// Constructor
			VrmCalibration(const ros::NodeHandle& node_handle);
			~VrmCalibration();

			void mainLoop();

		private:
			// ROS services

			// ROS variables
			ros::NodeHandle nh_;
			image_transport::SubscriberFilter left_cam_subscriber_;
			image_transport::SubscriberFilter center_cam_subscriber_;
			image_transport::SubscriberFilter right_cam_subscriber_;

			std::vector<ros::ServiceClient> cam_services_;

			typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
			typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

			boost::shared_ptr<ExactSync> exact_sync_;

			// Calibration variables
			cv::Size calib_board_size_;
			double calib_board_square_size_;
			std::string left_cam_topic_;
			std::string center_cam_topic_;
			std::string right_cam_topic_;
			std::string image_topic_;
			std::string transport_hint_;
			cv::Size image_size_;
			bool load_image_points_from_file_;
			std::string image_point_file_;
			std::string grid_type_string_;
			bool use_ransac_;
			bool show_exposure_;

			int grid_type_;

			std::vector<cv::Point3f> chessboard_corners_;

			std::vector<std::vector<cv::Mat> > calibration_images_;
			std::vector<std::vector<std::vector<cv::Point2f> > > image_points_;

			std::vector<std::vector<cv::Mat> > transformation_images_;
			std::vector<std::vector<std::vector<cv::Point2f> > > image_points_transformation_;

			std::vector<cv::Mat> camera_matrixes_;
			std::vector<cv::Mat> dist_coeffs_;

			std::vector<cv::Mat> T_, T2_;

			cv::Mat R12, T12, R13, T13, R32, T32;
			std::vector<cv::Mat> P, R;
			std::vector<cv::Mat> map_x_, map_y_;

			std::vector<double> alpha_;
			int alpha_slider_max_;
			std::vector<int> alpha_slider_;

			bool save_next_image_set_;

			bool rectified_;

			bool recalculate_rectification_;
			bool test_rectification_;
			bool transformation_estimated_;

			bool use_matlab_intrinsic_;
			std::string matlab_intrinsic_path_;

			// Functions
			void initialize();
			void calcChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners);

			void imageCb(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& center, const sensor_msgs::ImageConstPtr& right);

			void updateGUI(const std::vector<cv::Mat>& input_images, cv::Mat& gui_image);

			void calibrate();

			void saveToFile(const std::string& filename);

			void commitCalibration();

			static void on_alpha_trackbar_left_right( int, void* object );
			static void on_alpha_trackbar_left_center( int, void* object );
			static void on_alpha_trackbar_center_right( int, void* object );

			void findCorners(int cam_idx);

			void saveImagePoints(const std::string& filename);

			bool loadImagePoints(const std::string filename);

			bool loadMatlabIntrinsic(const std::string& path);

			void saveImages(const std::string& path);

			void testRectification(const std::vector<cv::Mat>& input_images);

			void doStereoCalibration();

			void calibrateThreeSeperate();

			void calcTransformations();

			void computeT();

			void transformPoints(const std::vector<cv::Point3d>& inPoints, const cv::Mat& T, std::vector<cv::Point3d>& outPoints);

			void computeTransform(const std::vector<cv::Point3d>& oldPoints, const std::vector<cv::Point3d>& newPoints, cv::Mat& T);
			void computeTransformation(std::vector<cv::Point3d>& oldPoints, std::vector<cv::Point3d>& newPoints, cv::Mat& transform);


	};

} /* namespace vrm3dvision */

#endif /* _VRM_CALIBRATION_HPP_ */
