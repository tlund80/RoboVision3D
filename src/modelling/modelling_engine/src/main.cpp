/*
 * main.cpp
 *
 *  Created on: Jul 26, 2013
 *      Author: thomas
 */
#include "ros/ros.h"
#include <modelling_engine/RosInterface.h>
#include <modelling_engine/SharedData.hpp>
#include <modelling_engine/RegistrationNode.h>
//#include <modelling_engine/ReconstructPointCloud.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>


#include <signal.h>
#include <exception>

using namespace modelling_engine;

boost::shared_ptr<boost::thread> registrationThread;
boost::shared_ptr<RegistrationNode> _registration;

void stop(int sig)
{
	_registration->stop();
}

void quit(int sig)
{
	std::cout << "\rQuitting!!" << std::endl;
	stop(SIGTERM);
	// wait for node to shutdown..
	std::cout << "Waiting for ros::ok() to finish" << std::endl;
	//while(ros::ok());

	std::cout << "Joining registrationThread... id = " <<  registrationThread->get_id();
	std::flush(std::cout);
	registrationThread->join();
	std::cout << "   Done." << std::endl;

	// TODO: delete/stop!! boost threads!
	exit(sig);
}

int main(int argc, char **argv)
{
	ROS_INFO("********************************************************");
	ROS_INFO("	This node implements 3D modeling Engine 	    ");
	ROS_INFO("	Input: Point cloud from structured light sensor    ");
	ROS_INFO("	Output: Registered point cloud and next view point ");
	ROS_INFO("*********************************************************");
	
	ros::init(argc, argv, "Modeling_Engine");
	ros::NodeHandle nh("~");
	
	bool _remove_table;
	bool _preprocess_cloud;
	
	if(!nh.getParam("remove_table",_remove_table))
	    ROS_ERROR("Could not get 'remove_table' from parameter server");
	if(!nh.getParam("preprocess_cloud",_preprocess_cloud))
	    ROS_ERROR("Could not get 'preprocess_cloud' from parameter server");

	//Create shared data space
	boost::shared_ptr<SharedData> _sharedData;
	_sharedData.reset(new SharedData());
	
	//Create ROS interface
	boost::shared_ptr<RosInterface> _rosInterface;
	_rosInterface.reset(new RosInterface(_sharedData,nh));

	_registration.reset(new RegistrationNode(_sharedData, _rosInterface, _remove_table, _preprocess_cloud));

//	signal(SIGTERM,stop);

	try
	{
		registrationThread.reset(new boost::thread(boost::bind(&RegistrationNode::start, _registration)));

	} catch (std::exception &e)
	{
		std::cout << "Exception: " << std::endl;
		std::cout << e.what() << std::endl;
	}



//	signal(SIGINT,quit);
//	signal(SIGTERM,quit);
//	signal(SIGKILL,quit);

	ROS_INFO("3D Modeling Engine is running");

	ros::spin();

	//quit(SIGTERM);

	return 0;
}
