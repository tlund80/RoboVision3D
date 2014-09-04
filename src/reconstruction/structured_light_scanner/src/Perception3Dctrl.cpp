/********************************************************************************************************************
 *
 * Copyright (c) 2013
 *
 * Danish Technological Institute (DTI)
 *
 *-------------------------------------------------------------------------------------------------------------------
 *
 * @file       Perception3Dctrl.cpp
 * @Author     Thomas Sølund (thso@dti.dk)
 * @brief
 * @details
 * @addtogroup
 *
 * $Revision: 4075 $
 * $Date: 2013-04-18 12:17:47 +0100 (Tue, 12 Nov 2013) $
 * $Author: thso $
 * $Id: Perception3Dctrl.cpp 4075 2013-04-18 11:17:47Z thso $
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
#include <structured_light_scanner/Perception3Dctrl.h>

namespace structured_light_scanner {

Perception_3D_ctrl::Perception_3D_ctrl(ros::NodeHandle _nh) {
	// TODO Auto-generated constructor stub
	nh = _nh;
	takeScan = false;
	scanFinish =  false;

}

Perception_3D_ctrl::~Perception_3D_ctrl() {
	// TODO Auto-generated destructor stub

	//delete myServer;
}

void Perception_3D_ctrl::startScan(bool _scan)
{

		ROS_INFO("Start scan called!!!!!!");
		takeScan = _scan;
}

void Perception_3D_ctrl::Initialize(void)
{
	cv::Mat cam_matrixL, cam_matrixR, distL, distR, R_L, P_L, R_R, P_R;

	calib.loadCameraParams(calib_path_left,"left_camera_name");
	calib.getIntrinsicCameraMatrixOpenCV(cam_matrixL);
	calib.getDistortionParams_OpenCV(distL);
	//calib.getProjectionMatrix_OpenCV(P_L);
	//calib.getRMatrix_OpenCV(R_L);

	width = calib.getImageWidth();
	height = calib.getImageHeight();
	recon.setImageHeight(height);
	recon.setImageWidth(width);

	std::cout << "CamL: \n" << cam_matrixL << std::endl;
	std::cout << "distL: \n" << distL << std::endl;
	//std::cout << "P_L: \n" << P_L << std::endl;
	//std::cout << "R_L: \n" << R_L << std::endl;

	calib.loadCameraParams(calib_path_right,"right_camera_name");
	calib.getIntrinsicCameraMatrixOpenCV(cam_matrixR);
	calib.getDistortionParams_OpenCV(distR);
	//calib.getProjectionMatrix_OpenCV(P_R);
	//calib.getRMatrix_OpenCV(R_R);

	std::cout << "CamR: \n" << cam_matrixR << std::endl;
	std::cout << "distR: \n" << distR << std::endl;
	//std::cout << "P_R: \n" << P_R << std::endl;
	//std::cout << "R_R: \n" << R_R << std::endl;

	cv::Mat R(3,3,CV_64FC1);
	//R.create(Size(3,3),CV_64FC1);

	R.at<double>(0,0) = 0.9336894948506391;//0.9244777461569724;//0.9738269899694305;
	R.at<double>(0,1) = 0.00207358968286116;//0.01737060432273074; //0.007250461837554449;
	R.at<double>(0,2) = 0.3580776835148842;//0.3808400700635209;//0.2271748762742498;
	R.at<double>(1,0) =  0.007085319718079838;//-0.01019481174450909;//0.002536518705310051;
	R.at<double>(1,1) = 0.9996804759527366;//0.9997306062028726;//0.9999147370895781;
	R.at<double>(1,2) =  -0.02426405249340654;//-0.02085139886748511;//-0.012223823366387738;
	R.at<double>(2,0) = -0.3580135827731275;//-0.3810996755102508;//-0.2272441350479509;
	R.at<double>(2,1) =  0.0251921857872105;//0.01539406141016944;//0.010860449080125102;
	R.at<double>(2,2) =  0.9333764665584898;//0.924405787627534;//0.9737772608415575;

	cv::Mat T(3,1,CV_64FC1);
	//T.create(Size(3,1),CV_64FC1);
	T.at<double>(0,0) =  -190.5956567567319;//-204.225626849231; //-0.20082550114496597;
	T.at<double>(1,0) =  1.503447176465363;//3.040247354346785; //0.002802067910488943;//-0.0006661421678872102;
	T.at<double>(2,0) = 38.44889066042349;//54.59155786716928;//0.019190505674472134;//  0.06940925750787658;

	//std::cout << "R: \n" << R << std::endl;
	//std::cout << "T: \n" << T << std::endl;

	recon.Init_rectification(cam_matrixL, distL, cam_matrixR, distR,R, T);
	recon.getProjectionMatrix1(P_L);
	recon.getProjectionMatrix2(P_R);
	recon.getRectificationMatrix1(R_L);
	recon.getRectificationMatrix2(R_R);

	left_pos.create(Size(height, width),CV_16S);
	left_neg.create(Size(height, width),CV_16S);
	right_pos.create(Size(height, width),CV_16S);
	right_neg.create(Size(height, width),CV_16S);


}

void Perception_3D_ctrl::LeftimageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	//sensor_msgs::CvBridge bridge;
	try
	{
		//ROS_INFO("left image received!!");
		// Convert msg to OpenCV and Halcon format
		//cv::Mat cvImage = bridge.imgMsgToCv(msg, "bgr8");
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		left_buf.send(cv_ptr->image.clone());
		//left_buf.send(cvImage.clone());

	}
	//catch (sensor_msgs::CvBridgeException& e)
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void Perception_3D_ctrl::RightimageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	//sensor_msgs::CvBridge bridge;
	try
	{
		//ROS_INFO("right image received!!");
		// Convert msg to OpenCV and Halcon format
		//Deprecated Fuerte method
		//cv::Mat cvImage = bridge.imgMsgToCv(msg, "bgr8");
		//right_buf.send(cvImage.clone());

		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		right_buf.send(cv_ptr->image.clone());

	}
	//catch (sensor_msgs::CvBridgeException& e)
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

bool Perception_3D_ctrl::startAcquisition(structured_light_scanner::StartAcquisition::Request &req, structured_light_scanner::StartAcquisition::Response &res)
{
	ROS_INFO("Perception_3D_ctrl::startAcquisition");
	//boost::thread acqThread(&Perception_3D_ctrl::aqusitionThread,this);


	ROS_INFO("Perception_3D_ctrl: Scanning ended!!");

	takeScan = true;

	res.status =  true;

	return true;
}

void Perception_3D_ctrl::setTopicSubscriber()
{
	ROS_INFO("Perception_3D: Subcribe to topics");

	it = new image_transport::ImageTransport(nh);

	_LeftImageSub = it->subscribe(left_image_topic, 1, &Perception_3D_ctrl::LeftimageCallback,this);
	_RightImageSub = it->subscribe(right_image_topic, 1, &Perception_3D_ctrl::RightimageCallback,this);

}

void Perception_3D_ctrl::setTopicPublisher()
{
	ROS_INFO("Perception_3D: Starting point cloud publisher");
	//_pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("reconstruction", 1);
	_pointCloudPub = pcl_ros::Publisher<pcl::PointXYZRGB> (nh, "reconstruction", 1);
}

void Perception_3D_ctrl::startServices()
{
	ROS_INFO("Perception_3D: Starting service startAcqusisition!");
	_serviceStartAcqusisition = nh.advertiseService("StartAcquisition", &Perception_3D_ctrl::startAcquisition, this);

}

void Perception_3D_ctrl::startActionServer()
{
	ROS_INFO("Perception_3D: Starting Action server!");

	using structured_light_scanner::Scan_Action_Server;
	myServer = new Scan_Action_Server(ros::this_node::getName(), nh);

	// Set communication functions!
	myServer->setStartScan(&IActionServer::startScan, this);

}

void Perception_3D_ctrl::loadParams()
{
	ROS_INFO("Perception_3D: Loading Node parameters");

	// Load parameters from launch file
	nh.param<std::string>("left_image_topic", left_image_topic, "/stereo/left/image_mono");
	nh.param<std::string>("right_image_topic", right_image_topic, "/stereo/right/image_mono");
	nh.param<std::string>("calib_path_left", calib_path_left, "/stereo/right/image_mono");
	nh.param<std::string>("calib_path_right", calib_path_right, "/stereo/right/image_mono");
	nh.param<std::string>("left_camera_name", left_camera_name, " ");
	nh.param<std::string>("right_camera_name", right_camera_name, " ");
	nh.param<std::string>("store_path", store_path, " ");
	nh.param<int>("num_of_patterns", num_of_patterns,10);

}

void Perception_3D_ctrl::aqusitionThread()
{
	ROS_INFO("Perception_3D: Starting aqusitionThread()");
	//a = new QApplication(_argc, _argv);
	QApplication a(_argc, _argv);

	ui = new structured_light_scanner::ProjectorUI();
	int screen_width, screen_height;
	ui->select_screen(a,screen_width, screen_height);

	ROS_INFO("Perception_3D: DTI_PROJECTOR_CTRL is running");

	perception::CamTrigger ct;
/*	ct.triggerOneshot();
	while(!(ct.triggerDone() > 0));
	left_buf.clear();
	right_buf.clear();
	ct.triggerOneshot();
	while(!(ct.triggerDone() > 0));
	left_buf.clear();
	right_buf.clear();
*/
	if( VISUALIZATION_MODE == 1 )
	{
		cv::namedWindow("white_L",CV_WINDOW_NORMAL);
		cv::namedWindow("white_R",CV_WINDOW_NORMAL);

		cv::namedWindow("black_L",CV_WINDOW_NORMAL);
		cv::namedWindow("black_R",CV_WINDOW_NORMAL);

		cv::namedWindow("Enc_L",CV_WINDOW_NORMAL);
		cv::namedWindow("Enc_R",CV_WINDOW_NORMAL);

	}
	//takeScan = true;

	//while(true)
	//{

	//	for(int i =0; i<10000000; i++);
//	}

	while(true)
	{
		//ROS_INFO("Perception_3D: scan = %d", getScan());
		//boost::posix_time::millisec delay(100);
		//boost::this_thread::sleep(delay);

		//ROS_INFO("Perception_3D: scan = %d", takeScan);

		if(takeScan){
			cv::Mat temp1, temp2;

			int state = 0;
			int code = 1;
			QImage image;

			int delay_time = 100;//100;

			int progress = 0;

			cv::Mat EncImLeft(height, width,CV_16U);
			cv::Mat EncImRight(height, width,CV_16U);

			EncImLeft.zeros(cv::Size(height, width),CV_16U);
			EncImRight.zeros(cv::Size(height, width),CV_16U);

			myServer->setScanState( STARTED);

			while(state != 7)
			{
				switch ( state ) {
				case 0:
					left_buf.clear();
					right_buf.clear();
					code=1;
					state = 1;
					progress = 100/22;
					myServer->updateProgress(progress);
				break;

			case 1: //take black image
				ROS_INFO("Displaying black image!");

				ui->create_black(image,ui->get_image_width(), ui->get_image_height());
				ui->showPattern(image);

				ui->delayMSec(delay_time);
				ct.triggerOneshot();
				while(!(ct.triggerDone() > 0));

				ui->delayMSec(delay_time);

				if(left_buf.isFull() && right_buf.isFull()){

					temp1 = left_buf.receive();
					cv::cvtColor(temp1, blackLeft,CV_RGB2GRAY);
					cv::cvtColor(right_buf.receive(), blackRight,CV_RGB2GRAY);
					left_buf.clear();
					right_buf.clear();

					cv::imwrite("/home/thomas/left_black.jpg",blackLeft);
					cv::imwrite("/home/thomas/right_black.jpg",blackRight);

					if( VISUALIZATION_MODE == 1 )
					{
						cv::imshow("black_L", blackLeft);
						cv::resizeWindow("black_L", blackLeft.cols/2, blackLeft.rows/2);
//						cv::moveWindow("black_L",WPOS1X, WPOS1Y);

						cv::imshow("black_R", blackRight);
						cv::resizeWindow("black_R", blackRight.cols/2, blackRight.rows/2);
//						cv::moveWindow("black_R",WPOS2X, WPOS2Y);

						cv::waitKey(0);
					}

					progress += 100/22;
					myServer->updateProgress(progress);

					state++;

				}else{
					ROS_INFO("re-trigger");
					state = 0;
				}
				break;

			case 2: //take white image
				ROS_INFO("Displaying white image!");

				//Display white image
				ui->create_white(image,ui->get_image_width(), ui->get_image_height());
				ui->showPattern(image);
				ui->delayMSec(delay_time*4);

				//Trigger camera
				ct.triggerOneshot();
				while(!(ct.triggerDone() > 0));
				ui->delayMSec(delay_time*2);

				if(left_buf.isFull() && right_buf.isFull()){// left_buf_size+1){

					temp2 = left_buf.receive();
					temp2.copyTo(rgb);
					cv::cvtColor(temp2, whiteLeft,CV_RGB2GRAY);
					cv::cvtColor(right_buf.receive(), whiteRight,CV_RGB2GRAY);
					left_buf.clear();
					right_buf.clear();

					rgb.convertTo(RGB, CV_8UC3);
					cv::imwrite("/home/thomas/left_white.jpg",whiteLeft);
					cv::imwrite("/home/thomas/right_white.jpg",whiteRight);

					progress += 100/22;
					myServer->updateProgress(progress);

					state ++;
					if( VISUALIZATION_MODE == 1 )
					{
						cv::imshow("white_L", whiteLeft);
						cv::resizeWindow("white_L", whiteLeft.cols/2, whiteLeft.rows/2);
						cv::moveWindow("white_L",WPOS1X, WPOS1Y);

						cv::imshow("white_R", whiteRight);
						cv::resizeWindow("white_R", whiteRight.cols/2, whiteRight.rows/2);
						cv::moveWindow("white_R",WPOS2X, WPOS2Y);

						cv::waitKey(0);
					}

					//recon.test_rectification(whiteLeft,whiteRight);
				}else ROS_INFO("re-trigger");


				break;

			case 3: //Create Occlusion mask

			//	ROS_INFO("Create Occlusion Mask");
				OMask.createOcclusionMask(blackRight, whiteRight);
				OMaskRight = OMask.getOcclusionMask();
				OMaskRightLogical =  OMask.getOcclusionMaskLogical();
		        //OMask.showOcclusionMask();

				OMask.createOcclusionMask(blackLeft, whiteLeft);
				OMaskLeft= OMask.getOcclusionMask();
				OMaskLeftLogical = OMask.getOcclusionMaskLogical();
				//OMask.showOcclusionMask();

				cv::imwrite("/home/thomas/left_OMask.jpg",OMaskLeft);
				cv::imwrite("/home/thomas/right_OMask.jpg",OMaskRight);

				progress += 100/22;
				myServer->updateProgress(progress);
				state++;

				break;

			case 4:

					//Display positive gray pattern
				//	ROS_INFO("Display positive gray pattern");
					ui->create_gray_code_positive(image,ui->get_image_width(), ui->get_image_height(),pow(2,code));
					ui->showPattern(image);
					ui->delayMSec(delay_time);

					//Trigger camera
					ct.triggerOneshot();
					while(!(ct.triggerDone() > 0));
					ui->delayMSec(delay_time);

					if(left_buf.isFull() && right_buf.isFull()){
						//We have received both left and right image

						cv::cvtColor(left_buf.receive(), left_pos,CV_RGB2GRAY);
						cv::cvtColor(right_buf.receive(), right_pos,CV_RGB2GRAY);

						if( VISUALIZATION_MODE == 1 )
						{
							cv::imshow("white_L", left_pos);
							cv::resizeWindow("white_L", left_pos.cols/2, left_pos.rows/2);
							cv::moveWindow("white_L",WPOS1X, WPOS1Y);

							cv::imshow("white_R", right_pos);
							cv::resizeWindow("white_R", right_pos.cols/2, right_pos.rows/2);
							cv::moveWindow("white_R",WPOS2X, WPOS2Y);

							cv::waitKey(0);
						}
						progress += 100/22;
						myServer->updateProgress(progress);
						state = 5;// = 0;

					}else{
						ROS_INFO("re-trigger");
						state = 4;
					}

				break;

			case 5:

				//Display negative gray pattern
				//ROS_INFO("Display negative gray pattern");
				ui->create_gray_code_negative(image,ui->get_image_width(), ui->get_image_height(),pow(2,code));
				ui->showPattern(image);
				ui->delayMSec(delay_time);

				//Trigger camera
				ct.triggerOneshot();
				while(!(ct.triggerDone() > 0));
				ui->delayMSec(delay_time);

				if(left_buf.isFull() && right_buf.isFull()){

					cv::cvtColor(left_buf.receive(), left_neg,CV_RGB2GRAY);
					cv::cvtColor(right_buf.receive(), right_neg,CV_RGB2GRAY);

					if( VISUALIZATION_MODE == 1 )
			 		{
						cv::imshow("black_L", left_neg);
						cv::resizeWindow("black_L", left_neg.cols/2, left_neg.rows/2);
						//cv::moveWindow("black_L",WPOS1X, WPOS1Y);

						cv::imshow("black_R", right_neg);
						cv::resizeWindow("black_R", right_neg.cols/2, right_neg.rows/2);
						//cv::moveWindow("black_R",WPOS2X, WPOS2Y);
						cv::waitKey(0);
			 		}

					//recon.writeImageToFile("/home/thomas/test.txt", left_neg_16U);

					recon.createEncodedImage(left_neg, left_pos,OMaskLeftLogical,EncImLeft,code -1,true);
					recon.createEncodedImage(right_neg,right_pos, OMaskRightLogical,EncImRight, code-1, false);

					//recon.writeImageToFile("/home/thomas/EncImLeft.txt", EncImLeft);

					if( VISUALIZATION_MODE == 1 )
					{
						cv::Mat Encimage_8U_left, Encimage_8U_right;

						 EncImRight.convertTo(Encimage_8U_right, CV_8U);
						 EncImLeft.convertTo(Encimage_8U_left, CV_8U);

						cv::imshow("Enc_L", Encimage_8U_left);
						cv::resizeWindow("Enc_L", Encimage_8U_left.cols/2, Encimage_8U_left.rows/2);

						cv::imshow("Enc_R", Encimage_8U_right);
						cv::resizeWindow("Enc_R", Encimage_8U_right.cols/2, Encimage_8U_right.rows/2);

						cv::waitKey(0);
					}
				//	recon.test_rectification(left_pos,right_pos);

					if(code == num_of_patterns){
						// finish
						state = 6;
						code=1;
					}else{
						progress += 100/22;
						myServer->updateProgress(progress);
						state= 4;
					}

					code++;

				}else{
					ROS_INFO("re-trigger");
					state = 5;
				}

				break;


			case 6:
			{
				//First rectify left image
				cv::Mat _rgb;
				recon.rectifyImage(RGB,_rgb);

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = recon.ComputePointCloudXYZRGB(EncImLeft, EncImRight,_rgb);

				pc->header.frame_id = "/sensor_frame";
				_pointCloudPub.publish(pc);

				//show black screen
				ui->create_black(image,ui->get_image_width(), ui->get_image_height());
				ui->showPattern(image);
				ui->delayMSec(delay_time);

				recon.cleanup();
				EncImLeft.release();
				EncImRight.release();

				progress = 100;
				myServer->updateProgress(progress);
				myServer->setScanTime(recon.getReconstructionTime());
				myServer->setPointCloud(pc);
				myServer->setScanState(FINISHED);
				state++;

				takeScan = false;
				scanFinish = true;

				//Store the point cloud
			//	pcl::io::savePCDFileBinary("scanner_cloud.pcd", *pc);

	/*			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
				viewer = recon.rgbVisualizer(pc);// .simpleVisualizer(pc);// createVisualizer( point_cloud_ptr );

				//Main loop
				while ( !viewer->wasStopped())
				{
				 viewer->spinOnce(100);
				 boost::this_thread::sleep (boost::posix_time::microseconds (100000));
				}
*/

			}
				break;

			default:
				state = 0;
			  // Code
			  break;
			}
			}
		}

		boost::posix_time::millisec msec(10);
		boost::this_thread::sleep(msec);
	}
}


void Perception_3D_ctrl::startPerception_3D(int argc, char **argv)
{
	_argc = argc;
	_argv = argv;
	ROS_INFO("Perception_3D: Starting main execution");

	left_buf.set_capacity(1);
	right_buf.set_capacity(1);

	boost::thread acqThread(&Perception_3D_ctrl::aqusitionThread,this);

//	run_thread = boost::thread(&Perception_3D_ctrl::aqusitionThread,this);

	ROS_INFO("Perception_3D: Thread finish");

	ros::spin();
}



} /* namespace perception */
