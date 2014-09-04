/********************************************************************************************************************
 *
 * \file                vrm_log_node.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-22
 * \version             0.1
 * \brief               ROS handler for VRmagic D3 camera module
 *
*********************************************************************************************************************/

#include <ros/ros.h>

// VRM communication protocol
#include <vrm_protocol/pubsub.hpp>
#include <vrm_protocol/vrm_log_msg.hpp>
#include <vrm_global.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vrm_log");
	ros::NodeHandle nh("/");

	// Read parameter from launch file
	ros::NodeHandle local_nh("~");
	std::string vrm_board_ip;
	local_nh.param<std::string>("vrm_board_ip", vrm_board_ip, "192.168.50.10");

	// Initialize log handler
	std::string vrm_log_server_address = "tcp://" + vrm_board_ip + ":" + VRM_LOG_SERVER_PORT;
	ROS_INFO("Initializing log client with address: %s", vrm_log_server_address.c_str());

	vrm_protocol::pubsub_client<vrm_protocol::vrm_log> vrm_log_client;
	vrm_log_client.startup(vrm_log_server_address);

    while(ros::ok())
    {
    	vrm_protocol::vrm_log log_msg;
    	if(vrm_log_client.receive(log_msg, 200))
    	{
    		switch (log_msg.level) {
				case LOG_ERROR_LEVEL:
					ROS_ERROR_STREAM("[D3 LOG]" << log_msg.message);
					break;
				case LOG_WARNING_LEVEL:
					ROS_WARN_STREAM("[D3 LOG]" << log_msg.message);
					break;
				case LOG_INFO_LEVEL:
					ROS_INFO_STREAM("[D3 LOG]" << log_msg.message);
					break;
				default:
					break;
			}
    	}
    }

    // Close ZMQ socket
    vrm_log_client.shutdown();

	return 0;
}
