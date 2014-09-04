/********************************************************************************************************************
 *
 * Copyright (c) 2013
 *
 * Danish Technological Institute (DTI)
 *
 *-------------------------------------------------------------------------------------------------------------------
 *
 * @file       LoadCalibParameters.cpp
 * @Author     Thomas Sølund (thso@dti.dk)
 * @brief
 * @details
 * @addtogroup
 *
 * $Revision: 4075 $
 * $Date: 2013-04-01 12:17:47 +0100 (Tue, 12 Nov 2013) $
 * $Author: thso $
 * $Id: LoadCalibParameters.cpp 4075 2013-04-01 11:17:47Z thso $
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
#include <structured_light_scanner/LoadCalibParameters.h>

namespace structured_light_scanner {

LoadCalibParameters::LoadCalibParameters() {
	// TODO Auto-generated constructor stub

}

LoadCalibParameters::~LoadCalibParameters() {
	// TODO Auto-generated destructor stub
}

void LoadCalibParameters::loadCameraParams(std::string MyPath, std::string camera_name)
{
        char buffer[12800];

        std::fstream param_file;
        param_file.open(MyPath.c_str(), ios::out | ios::in);

        if (param_file.is_open()) {
                param_file.read(buffer, 12800);
                param_file.close();
        }

        // Parse calibration file
        if (camera_calibration_parsers::readCalibrationYml(MyPath,camera_name,CameraInfo) ){
                DTI_INFO("Loaded calibration for camera " << camera_name);
        }else{
                DTI_INFO("Failed to load intrinsics from camera");
        }
}

void LoadCalibParameters::getIntrinsicCameraMatrixOpenCV(cv::Mat &dst)
{
	double K[9];

	for(int i = 0;i<= (int)CameraInfo.K.size()-1;i++){
		  K[i] =  CameraInfo.K.at(i);
		 // std::cout << (double)CameraInfo.K.at(i) << std::endl;
    }

	cv::Mat test(3,3,CV_64FC1,K);
	dst = test.clone();
}

void LoadCalibParameters::getDistortionParams_OpenCV(cv::Mat &dst)
{
	double D[5];
	for(int i = 0;i<= (int)CameraInfo.D.size()-1;i++){
	 	  D[i] =  (double)CameraInfo.D.at(i);
	 }

	cv::Mat test(5,1,CV_64FC1,D);
	dst = test.clone();
}

void LoadCalibParameters::getProjectionMatrix_OpenCV(cv::Mat &dst)
{
	double P[12];
	for(int i = 0;i<= (int)CameraInfo.P.size()-1;i++){
	 	  P[i] =  (double)CameraInfo.P.at(i);
	 }

	cv::Mat test(4,3,CV_64FC1,P);
	dst = test.clone();
}

void LoadCalibParameters::getRMatrix_OpenCV(cv::Mat &dst)
{
	double R[9];
	for(int i = 0;i<= (int)CameraInfo.R.size()-1;i++){
	 	  R[i] =  (double)CameraInfo.R.at(i);
	 }

	cv::Mat test(3,3,CV_64FC1,R);
	dst = test.clone();
}



} /* namespace structured_light_scanner */
