/********************************************************************************************************************
 *
 * \file                vrm3dvision_node.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-22
 * \version             0.1
 * \brief               ROS handler for VRmagic D3 camera module
 *
*********************************************************************************************************************/

#include "vrm_command.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vrm_command");
	ros::NodeHandle nh("/");

	vrm3dvision::VrmCommand vc(nh);
	vc.mainLoop(); // never returns

	return 0;
}
