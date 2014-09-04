/********************************************************************************************************************
 *
 * Copyright (c) 2013
 *
 * Danish Technological Institute (DTI)
 *
 *-------------------------------------------------------------------------------------------------------------------
 *
 * @file       ScanActionServer.cpp
 * @Author     Thomas Sølund (thso@dti.dk)
 * @brief
 * @details
 * @addtogroup
 *
 * $Revision: 4075 $
 * $Date: 2013-08-04 12:17:47 +0100 (Tue, 12 Nov 2013) $
 * $Author: thso $
 * $Id: ScanActionServer.cpp 4075 2013-08-04 11:17:47Z thso $
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
#include <structured_light_scanner/ScanActionServer.h>

#include "Logger/DTILogger.h"
#include <iostream>

namespace structured_light_scanner {

Scan_Action_Server::Scan_Action_Server(std::string name, ros::NodeHandle n) :
		  action_name_(name),
		  nh_(n),
		  as_(nh_, name, boost::bind(&Scan_Action_Server::executeCB, this, _1), false)
{
	// TODO Auto-generated constructor stub
	_activateScan = NULL;
	_CommActivateScan = NULL;

	ROS_INFO( "HELLO From %s", action_name_.c_str());
	_state = STOPPED;
	as_.start();
	_success = false;
}

Scan_Action_Server::~Scan_Action_Server() {
	// TODO Auto-generated destructor stub
}


void Scan_Action_Server::setStartScan(void (IActionServer::*activate)(bool), IActionServer* _Comm)
{
	ROS_INFO("Setting local pointer!!");
	_activateScan = activate;
	_CommActivateScan =_Comm;
}

void Scan_Action_Server::startScan(bool start)
{
	ROS_INFO("Start scanning!!");
	_CommActivateScan->startScan(true);

}
void Scan_Action_Server::executeCB(const structured_light_scanner::ScanGoalConstPtr& goal)
{
	ROS_INFO("Structured_light_scanner: Action server callback!");

	// helper variables
	ros::Time nowTime = ros::Time::now();
	ros::Rate r(5);
	_success = true;
	static int callCount = 0;
	_progress = 0;

	// print general call count
	DTI_DEBUG("Received scan request: " << ++callCount<< "\n"
				<< "Scan goal: " << goal->scan);

	_state = RECEIVED;
	startScan(true);

	//boost::thread acqThread(&Perception_3D_ctrl::aqusitionThread,this);

		//Check if the thread is started
	//	int n = (int)boost::posix_time::time_duration::ticks_per_second() / 10;
	//	boost::posix_time::time_duration delay(0,0,0,n);
	//	if(acqThread.timed_join(delay)){
	//		ROS_ERROR("Acqusisition not started!!");
	//		success = false;
	//	}

		// update the consisten variable in the feedback!
		if(_success)
		{
			feedback_.state = (int)_state;
			feedback_.scan_request_received = nowTime;
			feedback_.progress = _progress;
		}
		else{
			_state = FAILED;
		}

		/* ***********************************************/
		/* ***** Publish feedback only when active *******/
		/* ***********************************************/
		while (_success && // did start the thread
					_state < FINISHED && _state > STOPPED) // state is valid (active=1:2)
		    {
		        // check that preempt has not been requested by the client
		        if (as_.isPreemptRequested() || !ros::ok())
		        {
		          DTI_WARN(action_name_ << " --> Preempted");
		          // set the action state to preempted
		          as_.setPreempted();
		          _success = false;
		          break;
		        }
		        //Update feedback
		        feedback_.progress = _progress;
		        feedback_.state = (int)_state;

		       // DTI_TRACE(action_name_ << " --> state while active = " << feedback_.state
		       // 					   << " --> progress = " << _progress);

		        // publish the feedback.
		        as_.publishFeedback(feedback_);
		        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
		        r.sleep();
		    }



			if(_state == FINISHED)
			{
				DTI_INFO("Scanning finish!");
				_success = true;
				DTI_INFO(action_name_ << ": Finished, successfully and generated a point cloud!\n");
				result_.success = _success;
				result_.cloud_id = 100;
				result_.scan_request_received = nowTime;
				result_.scan_time = _scanTime;
				result_.pc = ros_pointcloud;
				as_.setSucceeded(result_);
				//as_.setSucceeded(SkillEngine::PrimitiveResult(result_));
			}
			else if(_state == FAILED)
			{
				DTI_INFO(action_name_ << ": Finished, but FAILED");
				result_.success = _success;
				result_.cloud_id = 100;
				result_.scan_request_received = nowTime;
				result_.scan_time = 100000;
				as_.setAborted(result_, "execution failed!");
			}
			else{
				DTI_WARN("Unknown behavior in ActionServer!!");
			}






}

} /* namespace structured_light_scanner */


/*int main(int argc, char **argv) {
	using dti_co_worker::Scan_Action_Server;

	ros::init(argc, argv, "ScanAction");

	Scan_Action_Server myServer(ros::this_node::getName());

	ros::spin();
}
*/
