/********************************************************************************************************************
 *
 * Copyright (c) 2013
 *
 * Danish Technological Institute (DTI)
 *
 *-------------------------------------------------------------------------------------------------------------------
 *
 * @file       ScanActionServer.h
 * @Author     Thomas Sølund (thso@dti.dk)
 * @brief
 * @details
 * @addtogroup
 *
 * $Revision: 3026 $
 * $Date: 2013-07-10 09:46:01 +0200 (Wed, 10 Jul 2013) $
 * $Author: thso $
 * $Id:  ScanActionServer.h 3026 2013-07-10 07:46:01Z thso $
 *
 *
 *-------------------------------------------------------------------------------------------------------------------
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Danish Technolocial Institute (DTI) nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * This program is copyrighted software: you can redistribute it and/or modify
 * it under the terms and protection of the Consortium Agreements and/or NDAs
 * assosiated to collaborative projects with DTI. Redistribution to and use by
 * parties outside of such an agreement is prohibited.
 *
 * This program is distributed in the hope that modifications of this software
 * and results achieved thereof will be reported back to DTI and the collaborative
 * projects. When modifying and evaluating this software, ensure that the author
 * have been notified.
 *
 *
 ********************************************************************************************************************/
#ifndef PRIMITIVEACTIONSERVER_H_
#define PRIMITIVEACTIONSERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <structured_light_scanner/ScanAction.h>
#include "Config/Configurations.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/thread/mutex.hpp>

#include "IActionServer.h"

namespace structured_light_scanner {

enum ScanState {FAILED=-1, STOPPED=0, RECEIVED=1, STARTED=2, FINISHED=3};

class Scan_Action_Server  {
public:
	Scan_Action_Server(std::string name, ros::NodeHandle n);
	virtual ~Scan_Action_Server();

	void executeCB(const structured_light_scanner::ScanGoalConstPtr &goal);

	void setStartScan(void (IActionServer::*activate)(bool), IActionServer* _Comm);


	void setScanState(ScanState state)
	{
		lock(_mutexScanState);
		_state = state;
	}

	ScanState getScanState(void)
	{
		lock(_mutexScanState);
		return _state;
	}

	void updateProgress(int progress)
	{
		lock(_mutexProgress);
		_progress = progress;
	}

	int getProgress(void)
	{
		lock(_mutexProgress);
		return _progress;
	}

	void setScanTime(double scantime)
	{
		lock(_mutexScanTime);
		_scanTime = scantime;
	}

	void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc)
	{
		lock(_mutexPointCloud);
		pcl::toROSMsg(*pc, ros_pointcloud);
		//_point_cloud = pc;
	}

private:

	void startScan(bool sc);
	void (IActionServer::*_activateScan)(bool);

	IActionServer* _CommActivateScan;


	ScanState _state;
	int _progress;
	double _scanTime;
	bool _success;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr _point_cloud;

	sensor_msgs::PointCloud2 ros_pointcloud;

	typedef boost::mutex::scoped_lock  lock;
	boost::mutex _mutexProgress;
	boost::mutex _mutexScanState;
	boost::mutex _mutexScanTime;
	boost::mutex _mutexPointCloud;

protected:
	  ros::NodeHandle nh_;
	  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
	  actionlib::SimpleActionServer<structured_light_scanner::ScanAction> as_;
	  const std::string action_name_;
	  // create messages that are used to published feedback/result
	  structured_light_scanner::ScanFeedback feedback_;
	  structured_light_scanner::ScanResult result_;


};

} /* namespace dti_co_worker */
#endif /* PRIMITIVEACTIONSERVER_H_ */
