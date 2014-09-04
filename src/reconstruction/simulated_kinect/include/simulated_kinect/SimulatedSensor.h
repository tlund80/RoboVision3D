/*
 * RosInterface.h
 *
 *  Created on: Jan 14, 2013
 *      Author: Thomas
 */

#ifndef SIMULATEDSENSOR_H_
#define SIMULATEDSENSOR_H_


#include "std_msgs/String.h"

#include <rw/math/Q.hpp>
//#include <Logger/DTILogger.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>

#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/proximity/CollisionDetector.hpp>

#include <rw/geometry/PointCloud.hpp>

#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/SimulatedKinnect.hpp>

#include <boost/foreach.hpp>

namespace simulated_kinect {

typedef pcl::PointXYZ PointT;
  
class SimulatedSensor 
{
public:
	SimulatedSensor(const char* wcFilename, std::string deviceName, std::string tcpFrameName, std::string sensorFrameName);
	virtual ~SimulatedSensor();

private:
      
      void init();
      
      
      const char* _workCell_Filename;
      std::string _deviceName;
      std::string _tcpFrameName;
      std::string _sensorFrameName;
      rw::models::WorkCell::Ptr _workcell;
      rw::kinematics::State _state;
      rw::kinematics::State _initState;
      rw::models::Device::Ptr _robot;
      rw::kinematics::Frame *_tcpFrame;
      rw::kinematics::Frame *_sensorFrame;
      
      rwlibs::simulation::FrameGrabber::Ptr _fgrabber2D;
       rwlibs::simulation::FrameGrabber25D::Ptr _fgrabber25D;
      rwlibs::simulation::SimulatedKinnect::Ptr _kinectgrabber;
	
public:
      
      void saveRgbImage(rw::kinematics::Frame *camFrame, std::string name);
	
};

} /* namespace simulated_kinect */
#endif /* SIMULATEDSENSOR_H_ */
