/********************************************************************************************************************
 *
 * \file                pose_estimation_node.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2014-02-04
 * \version             0.1
 * \brief
 *
*********************************************************************************************************************/

#include "pose_estimator.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose_estimation_node");
	ros::NodeHandle nh("/");

	vrm3dvision::PoseEstimator pe(nh);

	pe.initialize();

	ros::spin(); // never returns

	return 0;
}
