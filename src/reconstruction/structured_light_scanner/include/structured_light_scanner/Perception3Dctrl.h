/*
 * Perception3Dctrl.h
 *
 *  Created on: Apr 18, 2013
 *      Author: thomas
 */

#ifndef PERCEPTION3DCTRL_H_
#define PERCEPTION3DCTRL_H_

//ROS includes
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//OpenCv includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

//Qt includes
#include <QApplication>
#include <QtGui>
#include <QImage>

#include <cmath>
#include <csignal>
#include "Projector_controller/ProjectorUI.h"
#include "Imagebuffer.h"

#include <boost/thread/thread.hpp>

#include <sys/time.h>
#include "Projector_controller/CamTrigger.h"
#include "OcclusionMask.h"
#include "Reconstruction3D.h"
#include "LoadCalibParameters.h"
#include "ScanActionServer.h"
#include "IActionServer.h"

#include <actionlib/server/simple_action_server.h>
#include <structured_light_scanner/ScanAction.h>
#include <structured_light_scanner/StartAcquisition.h>

#include <boost/thread/mutex.hpp>



// OPENCV WINDOWS LOCATIONS
//***************************************
#define WPOS1X 20
#define WPOS1Y 10
#define WPOS2X 420
#define WPOS2Y 10
#define WPOS3X 820
#define WPOS3Y 10

#define VISUALIZATION_MODE 0

namespace structured_light_scanner {

class Perception_3D_ctrl : public IActionServer {
public:
	Perception_3D_ctrl(ros::NodeHandle _nh);
	virtual ~Perception_3D_ctrl();

	void Initialize(void);
	void setTopicSubscriber();
	void setTopicPublisher();
	void loadParams();
	void startServices();
	void startActionServer();
	void startPerception_3D(int argc, char **argv);

	bool startAcquisition(structured_light_scanner::StartAcquisition::Request &req, structured_light_scanner::StartAcquisition::Response &res);

	void startScan(bool _scan);

private:
	ros::NodeHandle nh;
	image_transport::ImageTransport *it;

	std::string action_name_;
	structured_light_scanner::Scan_Action_Server *myServer;


	boost::mutex::scoped_lock _mutexStartScan;

	boost::thread run_thread;

	structured_light_scanner::ScanState _state;

	bool takeScan, scanFinish;

	//Node parameters
	std::string left_image_topic;
	std::string right_image_topic;
	std::string calib_path_left;
	std::string calib_path_right;
	std::string left_camera_name;
	std::string right_camera_name;
	std::string store_path;
	int num_of_patterns;

	image_transport::Subscriber _LeftImageSub;
	image_transport::Subscriber _RightImageSub;

	//ros::Publisher _pointCloudPub;
	pcl_ros::Publisher<pcl::PointXYZRGB> _pointCloudPub;
	ros::ServiceServer _serviceStartAcqusisition;
	cv::Mat left_pos, left_neg, right_pos, right_neg;
	cv::Mat blackLeft, whiteLeft,blackRight, whiteRight, rgb, RGB;
	cv::Mat OMaskLeft, OMaskRight, OMaskLeftLogical,  OMaskRightLogical;
	//cv::Mat EncImLeft, EncImRight;

	structured_light_scanner::ProjectorUI *ui;
	int _argc;
	char **_argv;

	int width, height;

	OcclusionMask OMask;
	Reconstruction_3D recon;
	structured_light_scanner::LoadCalibParameters calib;

	structured_light_scanner::circ_buffer left_buf;
	structured_light_scanner::circ_buffer right_buf;


	void LeftimageCallback(const sensor_msgs::ImageConstPtr& msg);
	void RightimageCallback(const sensor_msgs::ImageConstPtr& msg);
	void aqusitionThread();
};

} /* namespace perception */
#endif /* PERCEPTION3DCTRL_H_ */
