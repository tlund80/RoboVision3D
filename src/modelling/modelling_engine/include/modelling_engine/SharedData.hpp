/*
 * SharedData.h
 *
 *  Created on: Jul 26, 2013
 *      Author: thomas
 */

#ifndef SHAREDDATA_H_
#define SHAREDDATA_H_

// STL includes
#include <vector>
#include <exception>
#include <iostream>

// Boost includes
#include <boost/thread/mutex.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl_ros/point_cloud.h>

// Eigen includes
#include <Eigen/Core>

// Own includes
#include "robot_msgs/Pose.h"
#include <modelling_engine/tiv_types.hpp>
#include <modelling_engine/rot.hpp>
#include "ScanData.hpp"

namespace modelling_engine {
  
enum ScanningStates{INIT_REGISTRATION = 0, REGISTRE, RECONSTRUCT, REFINE, FINALIZE, NOP};

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class SharedData : boost::noncopyable{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SharedData();
	virtual ~SharedData();

private:

	typedef boost::mutex::scoped_lock  lock;

	boost::mutex _mutexModel;
	boost::mutex _mutexTransform;
	boost::mutex _mutexDifTransform;
	boost::mutex _mutexHandEyeTransform;
	boost::mutex _mutexlastViewPoint;
	boost::mutex _mutexViewPoint;
	boost::mutex _mutexRobotFlangePose;
	boost::mutex _mutexPointCloud2;
	boost::mutex _mutexAddToList;
	
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pointcloud2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _model;

	//boost::ptr_vector<perception_3D::ScanData> _pointCloudList;
	std::vector<boost::shared_ptr<ScanData> > _pointCloudList;

	//Current Camera viewpoint
	Eigen::Matrix4f _viewPoint;
	//Last Camera viewpoint
	Eigen::Matrix4f _lastviewPoint;
	//Robot flange pose
	Eigen::Matrix4f _robotFlange;
	Eigen::Matrix4f _transform;
	Eigen::Matrix4f _dif_transform;
	Eigen::Matrix4f _handeye_transform;

public:
	void setHandEyeTransform(const Eigen::Matrix4f handeye) 
	{
		lock(_mutexHandEyeTransform);
		// Update hand_eye transform
		_handeye_transform(0,0) = handeye(0,0); _handeye_transform(0,1) = handeye(0,1); _handeye_transform(0,2) = handeye(0,2); _handeye_transform(0,3) = handeye(0,3);
		_handeye_transform(1,0) = handeye(1,0); _handeye_transform(1,1) = handeye(2,1); _handeye_transform(1,2) = handeye(1,2); _handeye_transform(1,3) = handeye(1,3);
		_handeye_transform(2,0) = handeye(2,0); _handeye_transform(2,1) = handeye(2,1); _handeye_transform(2,2) = handeye(2,2); _handeye_transform(2,3) = handeye(2,3);
		_handeye_transform(3,0) = handeye(3,0); _handeye_transform(3,1) = handeye(3,1); _handeye_transform(3,2) = handeye(3,2); _handeye_transform(3,3) = handeye(3,3);
		
		//_handeye_transform = handeye;
  
	}

	void getHandEyeTransform(Eigen::Matrix4f& trans){
		lock(_mutexHandEyeTransform);
		
		trans(0,0) = _handeye_transform(0,0); trans(0,1) = _handeye_transform(0,1); trans(0,2) = _handeye_transform(0,2); trans(0,3) = _handeye_transform(0,3);
		trans(1,0) = _handeye_transform(1,0); trans(1,1) = _handeye_transform(2,1); trans(1,2) = _handeye_transform(1,2); trans(1,3) = _handeye_transform(1,3);
		trans(2,0) = _handeye_transform(2,0); trans(2,1) = _handeye_transform(2,1); trans(2,2) = _handeye_transform(2,2); trans(2,3) = _handeye_transform(2,3);
		trans(3,0) = _handeye_transform(3,0); trans(3,1) = _handeye_transform(3,1); trans(3,2) = _handeye_transform(3,2); trans(3,3) = _handeye_transform(3,3);
		//trans =  _handeye_transform ;
	
	  
	}

	void setViewPoint(const Eigen::Matrix4f vp){
		lock(_mutexViewPoint);
		// Update newest viewpoint
		//_viewPoint(0,0) = vp(0,0); _viewPoint(0,1) = vp(0,1); _viewPoint(0,2) = vp(0,2); _viewPoint(0,3) = vp(0,3);
		//_viewPoint(1,0) = vp(1,0); _viewPoint(1,1) = vp(2,1); _viewPoint(1,2) = vp(1,2); _viewPoint(1,3) = vp(1,3);
		//_viewPoint(2,0) = vp(2,0); _viewPoint(2,1) = vp(2,1); _viewPoint(2,2) = vp(2,2); _viewPoint(2,3) = vp(2,3);
		//_viewPoint(3,0) = vp(3,0); _viewPoint(3,1) = vp(3,1); _viewPoint(3,2) = vp(3,2); _viewPoint(3,3) = vp(3,3);
		
		_viewPoint = vp;
	
	}

	void getViewPoint(Eigen::Matrix4f& vp){
		lock(_mutexViewPoint);
		//vp(0,0) = _viewPoint(0,0); vp(0,1) = _viewPoint(0,1); vp(0,2) = _viewPoint(0,2); vp(0,3) = _viewPoint(0,3);
		//vp(1,0) = _viewPoint(1,0); vp(1,1) = _viewPoint(2,1); vp(1,2) = _viewPoint(1,2); vp(1,3) = _viewPoint(1,3);
		//vp(2,0) = _viewPoint(2,0); vp(2,1) = _viewPoint(2,1); vp(2,2) = _viewPoint(2,2); vp(2,3) = _viewPoint(2,3);
		//vp(3,0) = _viewPoint(3,0); vp(3,1) = _viewPoint(3,1); vp(3,2) = _viewPoint(3,2); vp(3,3) = _viewPoint(3,3);
	
		vp = _viewPoint;
	}

	void setLastViewPoint(const Eigen::Matrix4f vp){
		lock(_mutexlastViewPoint);
		
		//std::cout << "Setting last view Point: \n" << vp << std::endl; 
		//Save old viewPoint
		//_lastviewPoint(0,0) = vp(0,0); _lastviewPoint(0,1) = vp(0,1); _lastviewPoint(0,2) = vp(0,2); _lastviewPoint(0,3) = vp(0,3);
		//_lastviewPoint(1,0) = vp(1,0); _lastviewPoint(1,1) = vp(2,1); _lastviewPoint(1,2) = vp(1,2); _lastviewPoint(1,3) = vp(1,3);
		//_lastviewPoint(2,0) = vp(2,0); _lastviewPoint(2,1) = vp(2,1); _lastviewPoint(2,2) = vp(2,2); _lastviewPoint(2,3) = vp(2,3);
		//_lastviewPoint(3,0) = vp(3,0); _lastviewPoint(3,1) = vp(3,1); _lastviewPoint(3,2) = vp(3,2); _lastviewPoint(3,3) = vp(3,3);
		
		_lastviewPoint = vp;
	}

	void getLastViewPoint(Eigen::Matrix4f& vp){
		lock(_mutexlastViewPoint);
		//vp(0,0) = _lastviewPoint(0,0); vp(0,1) = _lastviewPoint(0,1); vp(0,2) = _lastviewPoint(0,2); vp(0,3) = _lastviewPoint(0,3);
		//vp(1,0) = _lastviewPoint(1,0); vp(1,1) = _lastviewPoint(2,1); vp(1,2) = _lastviewPoint(1,2); vp(1,3) = _lastviewPoint(1,3);
		//vp(2,0) = _lastviewPoint(2,0); vp(2,1) = _lastviewPoint(2,1); vp(2,2) = _lastviewPoint(2,2); vp(2,3) = _lastviewPoint(2,3);
		//vp(3,0) = _lastviewPoint(3,0); vp(3,1) = _lastviewPoint(3,1); vp(3,2) = _lastviewPoint(3,2); vp(3,3) = _lastviewPoint(3,3);
		vp = _lastviewPoint;
	}

	void setRobotFlangePose(const Eigen::Matrix4f r_pose){
		lock(_mutexRobotFlangePose);
		// Update newest Robot Flange Pose
		_robotFlange(0,0) = r_pose(0,0); _robotFlange(0,1) = r_pose(0,1); _robotFlange(0,2) = r_pose(0,2); _robotFlange(0,3) = r_pose(0,3);
		_robotFlange(1,0) = r_pose(1,0); _robotFlange(1,1) = r_pose(2,1); _robotFlange(1,2) = r_pose(1,2); _robotFlange(1,3) = r_pose(1,3);
		_robotFlange(2,0) = r_pose(2,0); _robotFlange(2,1) = r_pose(2,1); _robotFlange(2,2) = r_pose(2,2); _robotFlange(2,3) = r_pose(2,3);
		_robotFlange(3,0) = r_pose(3,0); _robotFlange(3,1) = r_pose(3,1); _robotFlange(3,2) = r_pose(3,2); _robotFlange(3,3) = r_pose(3,3);
		
		//_robotFlange = vp;
		
		//Compute camera viewpoint and update view point
		Eigen::Matrix4f ha; getHandEyeTransform(ha);
		Eigen::Matrix4f viewpoint =  _robotFlange * ha;

		//std::cout << "View point" << std::endl;
		 // std::cout << viewpoint << std::endl;
		
		setViewPoint(viewpoint);
	
	}
	Eigen::Matrix4f getRobotFlangePose(void){
		lock(_mutexRobotFlangePose);
			return _robotFlange ;
			
		}

	void setTransform(const Eigen::Matrix4f T){
		lock(_mutexTransform);
		//Save old viewPoint
		//_transform(0,0) = T(0,0); _transform(0,1) = T(0,1); _transform(0,2) = T(0,2); _transform(0,3) = T(0,3);
		//_transform(1,0) = T(1,0); _transform(1,1) = T(2,1); _transform(1,2) = T(1,2); _transform(1,3) = T(1,3);
		//_transform(2,0) = T(2,0); _transform(2,1) = T(2,1); _transform(2,2) = T(2,2); _transform(2,3) = T(2,3);
		//_transform(3,0) = T(3,0); _transform(3,1) = T(3,1); _transform(3,2) = T(3,2); _transform(3,3) = T(3,3);
		
		_transform = T;
	}
	
	void getTransfrom(Eigen::Matrix4f& T){
		lock(_mutexTransform);
		
		//T(0,0) = _transform(0,0); T(0,1) = _transform(0,1); T(0,2) = _transform(0,2); T(0,3) = _transform(0,3);
		//T(1,0) = _transform(1,0); T(1,1) = _transform(2,1); T(1,2) = _transform(1,2); T(1,3) = _transform(1,3);
		//T(2,0) = _transform(2,0); T(2,1) = _transform(2,1); T(2,2) = _transform(2,2); T(2,3) = _transform(2,3);
		//T(3,0) = _transform(3,0); T(3,1) = _transform(3,1); T(3,2) = _transform(3,2); T(3,3) = _transform(3,3);
		//return _transform;
		T = _transform;
	}

	void setDifTransform(const Eigen::Matrix4f T){
		lock(_mutexDifTransform);
		//Save old viewPoint
		//_dif_transform(0,0) = T(0,0); _dif_transform(0,1) = T(0,1); _dif_transform(0,2) = T(0,2); _dif_transform(0,3) = T(0,3);
		//_dif_transform(1,0) = T(1,0); _dif_transform(1,1) = T(2,1); _dif_transform(1,2) = T(1,2); _dif_transform(1,3) = T(1,3);
		//_dif_transform(2,0) = T(2,0); _dif_transform(2,1) = T(2,1); _dif_transform(2,2) = T(2,2); _dif_transform(2,3) = T(2,3);
		//_dif_transform(3,0) = T(3,0); _dif_transform(3,1) = T(3,1); _dif_transform(3,2) = T(3,2); _dif_transform(3,3) = T(3,3);
		_dif_transform = T;
	
	}
	void getDifTransfrom(Eigen::Matrix4f& T){
		lock(_mutexDifTransform);
		
		T(0,0) = _dif_transform(0,0); T(0,1) = _dif_transform(0,1); T(0,2) = _dif_transform(0,2); T(0,3) = _dif_transform(0,3);
		T(1,0) = _dif_transform(1,0); T(1,1) = _dif_transform(2,1); T(1,2) = _dif_transform(1,2); T(1,3) = _dif_transform(1,3);
		T(2,0) = _dif_transform(2,0); T(2,1) = _dif_transform(2,1); T(2,2) = _dif_transform(2,2); T(2,3) = _dif_transform(2,3);
		T(3,0) = _dif_transform(3,0); T(3,1) = _dif_transform(3,1); T(3,2) = _dif_transform(3,2); T(3,3) = _dif_transform(3,3);
		
	}

	void setModelPointcloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc)
	{
		lock(_mutexModel);
		_model = pc;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getModelPointcloud(void){
		lock(_mutexModel);
		return _model;
	
	  
	}
	
	void clearModelPointcloud()
	{
		lock(_mutexModel);
		_model->points.clear();
	}

	void setNewPointcloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc)
	{
		lock(_mutexPointCloud2);
		_pointcloud2 = pc;
		
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getNewPointcloud(void){
		lock(_mutexPointCloud2);
		return _pointcloud2;
		
	}

	boost::shared_ptr<ScanData> getNewestScanData(void){
		lock(_mutexAddToList);
		return _pointCloudList.back();
		
	}

	void addNewPointcloud2List(boost::shared_ptr<ScanData> data)
	{
		try{
		  lock(_mutexAddToList);
		  _pointCloudList.push_back(data);
		}catch(std::exception &e)
		{
		  std::cerr << "Exception in adding scan data to vector SharedData.hpp line 223" << std::endl;
		}
		
	}

	void clearPointcloudList()
	{
		lock(_mutexAddToList);
		_pointCloudList.clear();
	
	}

	int getPointcloudListLength()
	{
		lock(_mutexAddToList);
		return (int)_pointCloudList.size();
	
	}
};

} /* namespace perception */
#endif /* SHAREDDATA_H_ */
