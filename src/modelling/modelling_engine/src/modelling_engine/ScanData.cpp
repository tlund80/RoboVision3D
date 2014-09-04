/*
 * ScanData.cpp
 *
 *  Created on: Aug 13, 2013
 *      Author: thomas
 */

#include <Modeling_Engine/ScanData.hpp>

namespace perception_3D {

ScanData::ScanData() {
	// TODO Auto-generated constructor stub
	_viewPoint = Eigen::Matrix4f::Identity();
	_transform = Eigen::Matrix4f::Identity();
}

ScanData::~ScanData() {
	// TODO Auto-generated destructor stub
}

} /* namespace perception */
