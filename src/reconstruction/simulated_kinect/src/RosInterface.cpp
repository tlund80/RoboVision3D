/*
 * RosInterface.cpp
 *
 *  Created on: Jan 14, 2013
 *      Author: martin
 */

#include <simulated_kinect/RosInterface.h>
#include "sensor_msgs/JointState.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include <DTI/Config/Configurations.h>
#include <QThread>
#include <qvarlengtharray.h>

using namespace rw::math;

namespace simulated_kinect {

RosInterface::RosInterface()
{
	ROS_INFO("Starting RosInterface......");
  
	//Publisher
	_pubRobotMode    = _n.advertise<robot_msgs::Ur5Mode>("/ROS_Sensor/robotMode", 1000);
	_joint_state_publisher = _n.advertise<sensor_msgs::JointState>("/ROS_Sensor/joint_states", 1000);
	//_pointCloudPub = pcl_ros::Publisher<PointT> (_n, "/ROS_Sensor/cloud_out", 1);
	_pubCmdFinish = _n.advertise<robot_msgs::CommandFinish>("/ROS_Sensor/commandFinish", 1000);

	// Subscriber
	_subscribers.push_back(_n.subscribe("/ROS_Sensor/moveCmd",          1000, &RosInterface::moveCmdCallback, this));
	_subscribers.push_back(_n.subscribe("/ROS_Sensor/moveCmd_buffer",   10, &RosInterface::moveCmd_buffer_Callback, this));
	_subscribers.push_back(_n.subscribe("/ROS_Sensor/moveCmd_bufferExe",10, &RosInterface::moveCmd_bufferExe_Callback, this));
	_subscribers.push_back(_n.subscribe("/ROS_Sensor/resetCmd",         10, &RosInterface::resetCmdCallback, this));
	
	//ros::this_node::getName()
	action_name_ = "/ROS_Sensor/takeScan";
	as_ = new actionlib::SimpleActionServer<simulated_kinect::ScanAction>(_n,action_name_, boost::bind(&RosInterface::executeCB, this, _1), false);
	as_->start();
	
	_isRunning = false;
	_scanSuccess = false;
	_state = STOPPED;
	_progress = 0;
	_scanTime = 10000;
}

RosInterface::~RosInterface()
{
  _isRunning = false;
  as_->shutdown();
}

void RosInterface::run()
{
  ROS_INFO("Starting RosInterface thread....");
  _isRunning = true;
  while(_isRunning)
  {
   ros::MultiThreadedSpinner spinner(2);
   spinner.spin();
   
  }
  
  ros::waitForShutdown();
  
}

void RosInterface::stop()
{
    ROS_INFO("Stopping RosInterface thread....");
    _isRunning = false;
}

void RosInterface::addPointCloudPublisher(std::string frame_id)
{
  std::string topic = "/ROS_Sensor/" + frame_id + "/cloud_out";
  pcl_ros::Publisher<PointT> _Pub = pcl_ros::Publisher<PointT> (_n,topic, 1); ;
  _pointCloudPub.insert( std::pair<std::string, pcl_ros::Publisher<PointT> >(frame_id, _Pub));
}

void RosInterface::moveCmdCallback(const robot_msgs::MoveCmd::ConstPtr& msg)
{
    ROS_INFO("moveCmdCallback");
    emit moveCmdCalled(msg);
}

void RosInterface::moveCmd_buffer_Callback(const robot_msgs::MoveCmd::ConstPtr& msg)
{
    emit moveCmd_buffer_Called(msg);
}

void RosInterface::moveCmd_bufferExe_Callback(const robot_msgs::MoveCmd::ConstPtr& msg)
{
	std::vector<robot_msgs::MoveCmd::ConstPtr>::iterator it;
	for(it=_moveCmdBuffer.begin(); it!=_moveCmdBuffer.end(); it++)
		execute_move(*it, 0.05);
	execute_move(msg, 0.0);

	_moveCmdBuffer.clear();
}

void RosInterface::resetCmdCallback(const robot_msgs::ResetCmd::ConstPtr& msg)
{
	//_resetCmd(msg->cmdId);
}

void RosInterface::publishJoint(const rw::math::Q &q)
{
	robot_msgs::Joint jointMsg;
	for(unsigned int j=0; j<q.size(); j++)
		jointMsg.q.push_back(q[j]);
	_jointPublisher.publish(jointMsg);


	//std::string joint_names[6] = {"elbow_joint", "shoulder_pan_joint", "wrist_3_joint", "wrist_1_joint", "shoulder_lift_joint", "wrist_2_joint"};
	std::string joint_names[6] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
	sensor_msgs::JointState jointStateMsg;
	jointStateMsg.header.stamp = ros::Time::now();
	for(unsigned int j=0; j<q.size(); j++)
	{
		jointStateMsg.name.push_back(joint_names[j]);
		jointStateMsg.position.push_back(q[j]);
	}
	_joint_state_publisher.publish(jointStateMsg);
}

void RosInterface::publishPointCloud(pcl::PointCloud<PointT>::Ptr pc, std::string frame_id)
{
  pc->header.frame_id = frame_id;
  _pointCloudPub.find(frame_id)->second.publish(pc);
 // _pointCloudPub .publish(pc);
}

void RosInterface::moveJ(uint32_t cmd_id, const rw::math::Q&, double acceleration, double velocity, double r)
{
 
  
}

void RosInterface::moveL(uint32_t cmd_id, const rw::math::Q&, double acceleration, double velocity, double r)
{
  
}

void RosInterface::pubCmdFinish(uint32_t cmdId)
{
	robot_msgs::CommandFinish msg;
	msg.cmdId = cmdId;
	_pubCmdFinish.publish(msg);
}

void RosInterface::executeCB(const simulated_kinect::ScanGoalConstPtr& goal)
{
	ROS_INFO("Simulated Kinect: Action server called!");

	// helper variables
	ros::Time nowTime = ros::Time::now();
	ros::Rate r(5);
	_scanSuccess = true;
	static int callCount = 0;
	_progress = 0;

	// print general call count
	ROS_DEBUG_STREAM("Received scan request: " << ++callCount<< "\n"
				<< "Scan goal: " << goal->scan);

	_state = RECEIVED;
	emit getScan(false);
	

		// update the consisten variable in the feedback!
		if(_scanSuccess)
		{
			feedback_.state = (int)_state;
			feedback_.scan_request_received = nowTime;
			feedback_.progress = _progress;
		}
		else{
			_state = FAILED;
		}

		/* ***********************************************/
		/* ***** Publish feedback only when active *******/
		/* ***********************************************/
		while (_scanSuccess && // did start the thread
					_state < FINISHED && _state > STOPPED) // state is valid (active=1:2)
		    {
		        // check that preempt has not been requested by the client
		        if (as_->isPreemptRequested() || !ros::ok())
		        {
		          ROS_WARN_STREAM(action_name_ << " --> Preempted");
		          // set the action state to preempted
		          as_->setPreempted();
		          _scanSuccess = false;
		          break;
		        }
		        //Update feedback
		        feedback_.progress = _progress;
		        feedback_.state = (int)_state;

		       // DTI_TRACE(action_name_ << " --> state while active = " << feedback_.state
		       // 					   << " --> progress = " << _progress);

		        // publish the feedback.
		        as_->publishFeedback(feedback_);
		        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
		        r.sleep();
		    }

			if(_state == FINISHED)
			{
				ROS_INFO_STREAM("Scanning finish!");
				_scanSuccess = true;
				ROS_INFO_STREAM(action_name_ << ": Finished, successfully and generated a point cloud!\n");
				result_.success = _scanSuccess;
				result_.cloud_id = 100;
				result_.scan_request_received = nowTime;
				result_.scan_time = _scanTime;
				result_.pc = ros_pointcloud;
				as_->setSucceeded(result_);
				//as_.setSucceeded(SkillEngine::PrimitiveResult(result_));
			}
			else if(_state == FAILED)
			{
				ROS_INFO_STREAM(action_name_ << ": Finished, but FAILED");
				result_.success = _scanSuccess;
				result_.cloud_id = 100;
				result_.scan_request_received = nowTime;
				result_.scan_time = 100000;
				as_->setAborted(result_, "execution failed!");
			}
			else{
				ROS_WARN("Unknown behavior in ActionServer!!");
			}
}

} /* namespace RosInterface */
