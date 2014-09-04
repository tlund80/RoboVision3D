/********************************************************************************************************************
 *
 * Copyright (c) 2013
 *
 * Danish Technological Institute (DTI)
 *
 *-------------------------------------------------------------------------------------------------------------------
 *
 * @file       Perception_3D.cpp
 * @Author     Thomas Sølund (thso@dti.dk)
 * @brief
 * @details
 * @addtogroup
 *
 * $Revision: 4075 $
 * $Date: 2013-04-15 12:17:47 +0100 (Tue, 12 Nov 2013) $
 * $Author: thso $
 * $Id: Perception_3D.cpp 4075 2013-04-15 11:17:47Z thso $
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
//ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//OpenCv includes
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

//Qt includes
#include <QApplication>
#include <QtGui>
#include <QImage>

//C includes
#include <cmath>
#include <csignal>
#include <sys/time.h>

//Boost includes
#include <boost/thread/thread.hpp>

//Own includes
#include <structured_light_scanner/Imagebuffer.h>
#include <structured_light_scanner/IActionServer.h>
#include <structured_light_scanner/Perception3Dctrl.h>
#include <structured_light_scanner/Projector_controller/CamTrigger.h>
#include <structured_light_scanner/Projector_controller/ProjectorUI.h>

boost::thread run_thread;

int main(int argc, char **argv)
{
	ROS_INFO("****************************************************");
	ROS_INFO("This node controlling the DELL M110 mini projector  ");
	ROS_INFO("Projects random dot patterns and gray coded patterns");
	ROS_INFO("****************************************************");

	ros::init(argc, argv, "structured_light_scanner");
	ros::NodeHandle nh("~");


	structured_light_scanner::Perception_3D_ctrl ctrl(nh);
	ctrl.loadParams();
	ctrl.Initialize();
	ctrl.setTopicSubscriber();
	ctrl.setTopicPublisher();
	ctrl.startServices();
	ctrl.startActionServer();
	ctrl.startPerception_3D(argc, argv);

	ROS_INFO("--> Closing structured_light_scanner!");
	return 0;
}
