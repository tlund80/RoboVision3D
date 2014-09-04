/********************************************************************************************************************
 *
 * \file                vrm_calibration_node.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2014-02-20
 * \version             0.1
 * \brief
 *
*********************************************************************************************************************/

#include "vrm_calibration.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vrm_calibration_node");
	ros::NodeHandle nh("/");

	vrm3dvision::VrmCalibration vc(nh);

	vc.mainLoop(); // never returns

	return 0;
}
