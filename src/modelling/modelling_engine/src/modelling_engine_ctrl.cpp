/********************************************************************************************************************
 *
 * Copyright (c) 2014
 *
 * Danish Technological Institute (DTI)
 *
 *-------------------------------------------------------------------------------------------------------------------
 *
 * @file       main.cpp
 * @Author     Thomas Sølund (thso@dti.dk)
 * @brief
 * @details
 * @addtogroup
 *
 * $Revision: 4575 $
 * $Date: 2014-05-13 15:37:03 +0100 (Thu, 16 Jan 2014) $
 * $Author: thso $
 * $Id: main.cpp 4575 2014-05-13 14:37:03Z mmo $
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
#include "ros/ros.h"
#include <boost/thread/thread.hpp>
#include <boost/timer.hpp>
#include <signal.h>
#include <termios.h>


#include <simulated_kinect/ScanAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_msgs/ModellingResult.h>
#include <robot_srvs/PrePlanViewPoints.h>
#include <robot_srvs/NBV_MoveToNextPrePlannedPose.h>

#define KEYCODE_ESC   0x1B

boost::thread thread;
bool doRun = false;
bool scanning = false;


static struct termios cooked;
static struct termios raw;
static int kfd;

actionlib::SimpleActionClient<simulated_kinect::ScanAction> *ac;
ros::ServiceClient _prePlaCli;
ros::ServiceClient _moveToNextPose_Client;
ros::Publisher _pubClearmodel;

void modellingFisnishCB(const robot_msgs::ModellingResult::ConstPtr &msg)
{
  ROS_INFO("Modelling Finish!!");
}

void scanDoneCB(const actionlib::SimpleClientGoalState& state,const  simulated_kinect::ScanResultConstPtr& result)
{
	ROS_INFO("Action finished: %s",state.toString().c_str());
	scanning = false;

}

void close_prog(int sig){
  
   ROS_INFO("Killing modelling_engine_ctrl node!!"); 
  // Closing down!!
  doRun = false;
 // thread.join();
  delete ac;
  ros::shutdown();
  
  exit(0);
  
}

void console_thread()
{
  ROS_INFO("Starting console listner thread!!");
  
  doRun = true;
  int8_t c; // key container
  
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
   
  while(doRun) 
  {
    if(read(kfd, &c, 1) < 0){
	perror("read():");
	exit(-1);
	
      }
    
    switch(c)
    {
      			
      case 's':
         std::cout << "Starting modelling!" << std::endl;
	 if(!scanning){
	  if(ac->isServerConnected()){

	    scanning = true;
	    ROS_INFO("3D Modeling Engine: Scan action server started, sending goal.");
	    // send a goal to the action
	    simulated_kinect::ScanGoal goal;
	    goal.scan = "take scan"; 
	    ac->sendGoal(goal,boost::bind(&scanDoneCB,_1,_2));

	    //wait for the action to return
	    bool finished_before_timeout = ac->waitForResult(ros::Duration(30.0));

	    if (!finished_before_timeout)
	    {
		ROS_INFO("Modeling Engine Ctrl: Action client time out!!!");
	    
	    }
	  
	  }
	 }else
	    ROS_INFO("3D Modeling Engine: Scanning launched. Please wait!");
       break;
      case 'h':
        std::cout << "-------------------Help menu: ------------------------" << std::endl;
        std::cout << "\t 's' = start modelling                              " << std::endl;
        std::cout << "\t 'e' = stop/end modelling                           " << std::endl;
        std::cout << "\t 'p' = pre-plan view points                         " << std::endl;
        break;
	
      case 'e':
	std::cout << "Stop modelling" << std::endl;
      
       
        break;
	
      case 'c':
      {
	std::cout << "Clear model" << std::endl;
       robot_msgs::ModellingResult msg;
       _pubClearmodel.publish(msg);
      }
        break;
	
      case 'm':
      {
          std::cout << "Move to next pose" << std::endl;
	  robot_srvs::NBV_MoveToNextPrePlannedPose srv;
	  if(!_moveToNextPose_Client.call(srv)){
	     ROS_ERROR("Modelling_engine_ctrl: Error when calling moveToNextPrePlannedPose service!!");
	  }
	  
      }
        break;
	
      case 'p':
      {
        std::cout << "Pre-Plan view points" << std::endl;
	robot_srvs::PrePlanViewPoints srv;
	srv.request.object_orgin.position.x = 0.3087;
	srv.request.object_orgin.position.y = 0.0035;
	srv.request.object_orgin.position.z = 0.08415;
	srv.request.object_orgin.orientation.w = -0.000188185;
	srv.request.object_orgin.orientation.x = 0.923956;
	srv.request.object_orgin.orientation.y = 0.382499;
	srv.request.object_orgin.orientation.z = -7.79049e-05;
	
	
	if(!_prePlaCli.call(srv)){
	  ROS_ERROR("Modelling_engine_ctrl: Error when calling PrePlan service!!");
	  break;
	}
       
         ROS_INFO_STREAM(srv.response.num_view_points << " different view point generated");
      }
        break;
      
      case KEYCODE_ESC:
	    raise(SIGTERM);
	    std::cout << "Abort / Cancel / Reset";
	break;
	
      default:
	std::cout << "Unknown cmd! " << std::endl;;
	std::cout << "-------------------Help menu: ------------------------" << std::endl;
        std::cout << "\t 's' = start modelling                              " << std::endl;
        std::cout << "\t 'e' = stop/end modelling                           " << std::endl;
        std::cout << "\t 'p' = pre-plan view points                         " << std::endl;
      
    }
    std::cout.flush();
    boost::thread::sleep(boost::get_system_time()+boost::posix_time::milliseconds(500));
  
  }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "modelling_engine_ctrl");
	ros::NodeHandle n("~");
	
	signal(SIGTERM,close_prog);
	signal(SIGINT,close_prog);
	
	//Action client
	ac = new actionlib::SimpleActionClient<simulated_kinect::ScanAction>(n,"/ROS_Sensor/takeScan", true);

	ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
	ac->waitForServer(); //will wait for infinite timer
  
	ros::Subscriber _sub = n.subscribe("/modelling_engine/modellingResult",10,modellingFisnishCB);
	_prePlaCli = n.serviceClient<robot_srvs::PrePlanViewPoints>("/next_best_view_planner_node/preplan");
	_moveToNextPose_Client = n.serviceClient<robot_srvs::NBV_MoveToNextPrePlannedPose>("/next_best_view_planner_node/move_to_next_preplanned_pose");
	_pubClearmodel = n.advertise<robot_msgs::ModellingResult>( "/modelling_engine/clearModel",1);
	thread = boost::thread(console_thread);
	
	
	ROS_INFO("Modelling_engine_ctrl is running!!");
	ros::spin();


	return 0;
}
