/*
 * ScanData.cpp
 *
 *  Created on: Aug 13, 2013
 *      Author: thomas
 */

#include <modelling_engine/ScanData.hpp>

namespace modelling_engine {

ScanData::ScanData() {
	// TODO Auto-generated constructor stub
	_viewPoint = Eigen::Matrix4f::Identity();
	_transform = Eigen::Matrix4f::Identity();
}

ScanData::~ScanData() {
	// TODO Auto-generated destructor stub
}

} /* namespace perception */
