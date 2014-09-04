/*
 * RosInterface.h
 *
 *  Created on: Jan 14, 2013
 *      Author: Thomas
 */

#ifndef ROSINTERFACE_H_
#define ROSINTERFACE_H_

#include "ros/ros.h"
#include "robot_msgs/MoveCmd.h"
#include "robot_msgs/CommandFinish.h"
#include "robot_msgs/ResetCmd.h"
#include "robot_msgs/CommandFinish.h"
#include "robot_msgs/DigitalIO.h"
#include "robot_msgs/JointCurrent.h"
#include "robot_msgs/Ur5Mode.h"
#include "std_msgs/String.h"
#include <actionlib/server/simple_action_server.h>
#include <simulated_kinect/ScanAction.h>

#include <rw/math/Q.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>

//#include <QObject>
#include <QThread>

namespace simulated_kinect {

typedef pcl::PointXYZRGBA PointT;
enum ScanState {FAILED=-1, STOPPED=0, RECEIVED=1, STARTED=2, FINISHED=3};

class RosInterface : public QThread
{
  Q_OBJECT
public:
	RosInterface();
	virtual ~RosInterface();

private:
	void execute_move(const robot_msgs::MoveCmd::ConstPtr& msg, double r){
		unsigned int n = msg->joint.q.size();
		rw::math::Q q(n);
		for(unsigned int i=0; i<n; i++)
			q[i] = msg->joint.q[i];
		switch(msg->moveType)
		{
		case 0:		// moveJ
			//_moveJ(msg->cmdId, q, msg->acc, msg->vel, r);
			break;
		case 1:		// moveL
			//_moveL(msg->cmdId, q, msg->acc, msg->vel, r);
			break;
		}

	}
	
	void moveCmdCallback(const robot_msgs::MoveCmd::ConstPtr& msg);
	void moveCmd_buffer_Callback(const robot_msgs::MoveCmd::ConstPtr& msg);
	void moveCmd_bufferExe_Callback(const robot_msgs::MoveCmd::ConstPtr& msg);
	void moveJ(uint32_t cmd_id, const rw::math::Q&, double acceleration, double velocity, double r);
	void moveL(uint32_t cmd_id, const rw::math::Q&, double acceleration, double velocity, double r);
	void resetCmdCallback(const robot_msgs::ResetCmd::ConstPtr& msg);
	
Q_SIGNALS:
	void moveCmdCalled(robot_msgs::MoveCmd::ConstPtr msg);
	void moveCmd_buffer_Called(robot_msgs::MoveCmd::ConstPtr msg);
	void getScan(bool saveScan);
	
private:
	ros::NodeHandle _n;
	std::vector<ros::Subscriber> _subscribers;
	ros::Publisher _jointPublisher;
	ros::Publisher _pubRobotMode;
	ros::Publisher _pubCmdFinish;
	ros::Publisher _joint_state_publisher;
	std::map<std::string, pcl_ros::Publisher<PointT> > _pointCloudPub;
	ros::Subscriber _subMoveCmd;
	
	std::vector<robot_msgs::MoveCmd::ConstPtr> _moveCmdBuffer;
	
	sensor_msgs::PointCloud2 ros_pointcloud;
	bool _isRunning;
	
	//Action Server member variables
	ScanState _state;
	int _progress;
	double _scanTime;
	bool _scanSuccess;
	std::string action_name_;
	 
	// create messages that are used to published feedback/result
	ScanFeedback feedback_;
	ScanResult result_;
	
public:
	void publishPointCloud(pcl::PointCloud<PointT>::Ptr pc, std::string frame_id);
	void publishJoint(const rw::math::Q &q);
	void addPointCloudPublisher(std::string frame_id);
	//void pubRobotMode(const RobotModeData &robotModeData);
	void pubCmdFinish(uint32_t cmdId);
	void executeCB(const simulated_kinect::ScanGoalConstPtr& goal);
	
	void setActionServer_PointCloud(pcl::PointCloud<PointT>::Ptr pc){ pcl::toROSMsg(*pc, ros_pointcloud);}
	void setActionServer_ScanTime(double scantime){_scanTime = scantime;}
	void setActionServerProgress(int progress){_progress = progress;}
	void setScanState(ScanState state){_state = state;}

protected:	
	void run();
	bool isRunning(){return _isRunning; };
	void stop();
	
	actionlib::SimpleActionServer<simulated_kinect::ScanAction>* as_;
	
};


class RobotModeData
{
public:
	RobotModeData() {}

	long long timestamp;
	bool physical;

	// true = real robot, false = simulated robot
	bool real;

	bool robotPowerOn;
	bool emergencyStopped;
	bool securityStopped;
	bool programRunning;
	bool programPaused;
	int robotMode;
	double speedFraction;
};


} /* namespace ur5_executor */
#endif /* ROSINTERFACE_H_ */
