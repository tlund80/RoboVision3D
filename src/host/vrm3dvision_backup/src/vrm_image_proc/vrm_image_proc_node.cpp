/********************************************************************************************************************
 *
 * \file                vrm_image_proc_node.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-22
 * \version             0.1
 * \brief               image handler for VRmagic D3 camera module
 *
*********************************************************************************************************************/

#include "vrm_image_proc.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vrm_image_proc");
	ros::NodeHandle nh("/");

	vrm3dvision::VrmImageProc vip(nh);

	vip.initialize();

	vip.mainLoop(); // never returns

	return 0;
}
