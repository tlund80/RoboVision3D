/*
 * ScanData.h
 *
 *  Created on: Aug 13, 2013
 *      Author: thomas
 */

#ifndef SCANDATA_H_
#define SCANDATA_H_

#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl_ros/point_cloud.h>
#include <ros/ros.h> 

#include <boost/thread/mutex.hpp>

#include <modelling_engine/tiv_types.hpp>
#include <modelling_engine/rot.hpp>

namespace modelling_engine {

class ScanData {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ScanData();
	virtual ~ScanData();

private:

  	typedef boost::mutex::scoped_lock  lock;

	boost::mutex _mutexViewPoint;
	boost::mutex _mutexCentroid;
	boost::mutex _mutexTransform;
	boost::mutex _mutexPointCloud;
	boost::mutex _mutexReconstructionTime;
	boost::mutex _mutexScanTime;
	boost::mutex _mutexScanId;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pointCloud;
	Eigen::Matrix4f _viewPoint;
	tiv::pt3d _centroid;
	Eigen::Matrix4f _transform;
	double _reconstructionTime;
	ros::Time _scanTime;
	int _scanId;



public:

	/*
	 * Centroid of the point cloud
	 */
	void setCentroid(const tiv::pt3d centroid)
	{
		lock(_mutexCentroid);
		_centroid = centroid;
	}

	/*
	 * Acqusition view point
	 */
	void setViewPoint(const Eigen::Matrix4f& vp)
	{
		lock(_mutexViewPoint);
	//	_viewPoint(0,0) = vp(0,0); _viewPoint(0,1) = vp(0,1); _viewPoint(0,2) = vp(0,2); _viewPoint(0,3) = vp(0,3);
	//	_viewPoint(1,0) = vp(1,0); _viewPoint(1,1) = vp(2,1); _viewPoint(1,2) = vp(1,2); _viewPoint(1,3) = vp(1,3);
	//	_viewPoint(2,0) = vp(2,0); _viewPoint(2,1) = vp(2,1); _viewPoint(2,2) = vp(2,2); _viewPoint(2,3) = vp(2,3);
	//	_viewPoint(3,0) = vp(3,0); _viewPoint(3,1) = vp(3,1); _viewPoint(3,2) = vp(3,2); _viewPoint(3,3) = vp(3,3);
		
		_viewPoint = vp;
	}

	/*
	 * Transformation between robot base and viewpoint. Align the point cloud in a common coordinatesystem
	 */
	void setTransform(const Eigen::Matrix4f& transform)
	{
		lock(_mutexTransform);
	//	_transform(0,0) = transform(0,0); _transform(0,1) = transform(0,1); _transform(0,2) = transform(0,2); _transform(0,3) = transform(0,3);
	//	_transform(1,0) = transform(1,0); _transform(1,1) = transform(2,1); _transform(1,2) = transform(1,2); _transform(1,3) = transform(1,3);
	//	_transform(2,0) = transform(2,0); _transform(2,1) = transform(2,1); _transform(2,2) = transform(2,2); _transform(2,3) = transform(2,3);
	//	_transform(3,0) = transform(3,0); _transform(3,1) = transform(3,1); _transform(3,2) = transform(3,2); _transform(3,3) = transform(3,3);
		_transform = transform;
	}

	void setPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
	{
		lock(_mutexPointCloud);
		_pointCloud = pointCloud;
	}

	void setReconstructionTime(const double& reconstructionTime)
	{
		lock(_mutexReconstructionTime);
		_reconstructionTime = reconstructionTime;
	}
	void setScanTime(ros::Time scanTime)
	{
		lock(_mutexScanTime);
		_scanTime = scanTime;
	}
	void setScanId(int scanId)
	{
		lock(_mutexScanId);
		_scanId = scanId;
	}

	Eigen::Matrix4f getViewPoint(void)
	{
		lock(_mutexViewPoint);
		return _viewPoint;
	}

	/*
	 * Transformation to align the scan to the reference frame (first acquired Point cloud).
	 * Align the point cloud in a common coordinatesystem
	 */
	 void getTransform(Eigen::Matrix4f& T)
	{
		lock(_mutexTransform);
		//T(0,0) = _transform(0,0); T(0,1) = _transform(0,1); T(0,2) = _transform(0,2); T(0,3) = _transform(0,3);
		//T(1,0) = _transform(1,0); T(1,1) = _transform(1,1); T(1,2) = _transform(1,2); T(1,3) = _transform(1,3);
		//T(2,0) = _transform(2,0); T(2,1) = _transform(2,1); T(2,2) = _transform(2,2); T(2,3) = _transform(2,3);
		//T(3,0) = _transform(3,0); T(3,1) = _transform(3,1); T(3,2) = _transform(3,2); T(3,3) = _transform(3,3);
		T = _transform;
		//return _transform;
	}


	tiv::pt3d getCentroid(void)
	{
		lock(_mutexCentroid);
		return _centroid;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud(void)
	{
		lock(_mutexPointCloud);
		return _pointCloud;
	}
	
	double getReconstructionTime(void)
	{
		lock(_mutexReconstructionTime);
		return _reconstructionTime;
	}

	ros::Time getScanTime(void)
	{
		lock(_mutexScanTime);
		return _scanTime;
	}
	int getScanId(void)
	{
		lock(_mutexScanId);
		return _scanId;
	}

};

} /* namespace perception */
#endif /* SCANDATA_H_ */
