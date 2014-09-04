/*
 * SharedData.cpp
 *
 *  Created on: Aug 1, 2013
 *      Author: thomas
 */

#include <Modeling_Engine/SharedData.hpp>

namespace perception_3D {

SharedData::SharedData() {
	// TODO Auto-generated constructor stub

	_model.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	_handeye_transform =  Eigen::Matrix4f::Identity();
	_transform =  Eigen::Matrix4f::Identity();
	_dif_transform =  Eigen::Matrix4f::Identity();
	
	//Current Camera viewpoint
	_viewPoint =  Eigen::Matrix4f::Identity();
	//Last Camera viewpoint
	 _lastviewPoint =  Eigen::Matrix4f::Identity();
	//Robot flange pose
	 _robotFlange =  Eigen::Matrix4f::Identity();

	
}

SharedData::~SharedData() {
	// TODO Auto-generated destructor stub
	std::cout << "SharedData::~SharedData() called!!" << std::endl;
}

}
