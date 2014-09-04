/*
 * RosInterface.h
 *
 *  Created on: Jul 26, 2013
 *      Author: thomas
 */

#ifndef ROSINTERFACE_H_
#define ROSINTERFACE_H_

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "opencv2/opencv.hpp"

//Action lib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <structured_light_scanner/ScanAction.h>

// Boost lib
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/xpressive/regex_primitives.hpp>

#include "pcl_ros/point_cloud.h"
#include <pcl_msgs/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/publisher.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include "robot_msgs/Pose.h"
#include <modelling_engine/tiv_types.hpp>
#include <modelling_engine/rot.hpp>
#include <modelling_engine/SharedData.hpp>

#include "Logger/DTILogger.h"

#include <modelling_engine/ClearModel.h>
#include "robot_msgs/PerceptionMsg.h"
#include "robot_msgs/ResultPose.h"
#include <robot_msgs/ModellingResult.h>

namespace modelling_engine {
  
typedef pcl::PointXYZRGB PointT;  

class RosInterface {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	RosInterface(boost::shared_ptr<SharedData> data_ptr, ros::NodeHandle n);
	virtual ~RosInterface();

	void ControlCallback(const robot_msgs::PerceptionMsg::ConstPtr& msg);
	void cloudCB(const pcl::PointCloud<PointT>::Ptr& msg);
	//void robotFlangeCallback(const robot_msgs::Pose::ConstPtr msg);

	void pubRegisteredModel(pcl::PointCloud<PointT> pc);
	void pubLastPointCloud(pcl::PointCloud<PointT> pc);
	void pubLastMesh(pcl::PolygonMesh mesh);
	void pubModelMesh(pcl::PolygonMesh mesh);
	void pubFinishMsg(robot_msgs::ModellingResult msg);

	bool clearModelService(modelling_engine::ClearModel::Request& req, modelling_engine::ClearModel::Response& res);
	void scanDoneCB(const actionlib::SimpleClientGoalState& state,const structured_light_scanner::ScanResultConstPtr& result);
	void scanActiveCb(void);
	void scanfeedbackCb(const structured_light_scanner::ScanFeedbackConstPtr& feedback);

	void clearModelTopic(robot_msgs::ModellingResult msg);

	int getScanningState(void)
	{
		lock(_mutexState);
		return _reg_state;
	}
	void setScanningState( int s)
	{
		lock(_mutexState);
		_reg_state = s;
	};

	bool takeScan(void);

private:

	Eigen::Matrix4f GlobalTransform;
	typedef boost::mutex::scoped_lock  lock;
	boost::mutex _mutexState;


	boost::shared_ptr<SharedData> _sharedData;
	int _reg_state;
	
	//Point cloud from sensor
	pcl::PointCloud<PointT>::Ptr pointCloud;
	//Point cloud in global coordinatesystem
	pcl::PointCloud<PointT>::Ptr transformed_cloud;

	//SharedData *_sharedData;
	ros::NodeHandle _n;

	//Action client
	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<structured_light_scanner::ScanAction> *ac;

	//Intermediate robot poses
	std::vector<tiv::pose> robot_poses;

	// tf
	 tf::TransformListener listener;

	//Topic Subscribers
	ros::Subscriber _robotFlangSub;
	ros::Subscriber _pointcloudSub;
	ros::Subscriber _controlSub;
	ros::Subscriber _clearModelSub;

	//Topic publisher
	ros::Publisher _pubState;
	ros::Publisher _posePub;
	ros::Publisher _pubFinish;
	pcl_ros::Publisher<PointT> _pubLastPointCloud;
	pcl_ros::Publisher<PointT> _pubRegisteredModel;
	pcl_ros::Publisher<pcl::PolygonMesh> _pubLastMesh;
	pcl_ros::Publisher<pcl::PolygonMesh> _pubModelMesh;

	//Services
	ros::ServiceServer _clearModelService;

	//Parameters
	std::string _robotFlangeTopic;
	std::string _handeye_calib_path;
	
	std::string _sensor_frame;
	std::string _robotbase_frame;
	std::string _pointCloudTopic;

	int count;
	
	bool debug;
};

} /* namespace perception */
#endif /* ROSINTERFACE_H_ */
