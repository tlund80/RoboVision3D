/********************************************************************************************************************
 *
 * Copyright (c) 2013
 *
 * Danish Technological Institute (DTI)
 *
 *-------------------------------------------------------------------------------------------------------------------
 *
 * @file       ScanActionClient.cpp
 * @Author     Thomas Sølund (thso@dti.dk)
 * @brief
 * @details
 * @addtogroup
 *
 * $Revision: 4075 $
 * $Date: 2013-08-04 12:17:47 +0100 (Tue, 12 Nov 2013) $
 * $Author: thso $
 * $Id: ScanActionClient.cpp 4075 2013-08-04 11:17:47Z thso $
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
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <structured_light_scanner/ScanAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_ScanAction_server");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<structured_light_scanner::ScanAction> ac("/structured_light_scanner/structured_light_scanner", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  structured_light_scanner::ScanGoal goal;
  structured_light_scanner::ScanFeedback feedback;
  goal.scan = "Dette er dit goal!!";
  ac.sendGoal(goal);

 // while(ac.)
  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();

    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
