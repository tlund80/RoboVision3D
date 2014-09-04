/********************************************************************************************************************
 *
 * Copyright (c) 2013
 *
 * Danish Technological Institute (DTI)
 *
 *-------------------------------------------------------------------------------------------------------------------
 *
 * @file       OcclusionMask.cpp
 * @Author     Thomas Sølund (thso@dti.dk)
 * @brief
 * @details
 * @addtogroup
 *
 * $Revision: 4075 $
 * $Date: 2013-04-01 12:17:47 +0100 (Tue, 12 Nov 2013) $
 * $Author: thso $
 * $Id: OcclusionMask.cpp 4075 2013-04-01 11:17:47Z thso $
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
#include <structured_light_scanner/OcclusionMask.h>

//namespace structured_light_scanner {

OcclusionMask::OcclusionMask() {
	// TODO Auto-generated constructor stub

}

OcclusionMask::~OcclusionMask() {
	// TODO Auto-generated destructor stub
}

void OcclusionMask::showOcclusionMask()
{
	if(!OccMask.empty())
	{
		cv::namedWindow("Occlusion Mask",CV_WINDOW_NORMAL);
		cv::imshow("Occlusion Mask", OccMask);
		cv::resizeWindow("Occlusion Mask", OccMask.cols, OccMask.rows);
	    cv::moveWindow("Occlusion Mask",WPOS3X, WPOS3Y);

		cv::waitKey();
	}

}

void OcclusionMask::createOcclusionMask(cv::Mat& black, cv::Mat& white)
{
	cv::Mat thres_img;
	thresholdImage(white, thres_img);
	cv::subtract(thres_img,black, OccMask);

}

void OcclusionMask::thresholdImage(cv::Mat& src, cv::Mat& dst)
{
	double min, max;
	int minInd, maxInd;

	try{

	cv::minMaxIdx(src,&min, &max, &minInd, &maxInd, cv::Mat());
	//std::cout << "min: " << min << " max: " << max << std::endl;
	double thres_value = max*0.04;//max*0.04;
	cv::threshold(src,dst,thres_value,255,0);

	}catch(cv::Exception &e){
		 std::cout << "In OcclusionMask::thresholdImage(): " << e.what() << std::endl;
	}

}

cv::Mat OcclusionMask::getOcclusionMaskLogical(){

	cv::Mat img;

	OccMask.convertTo(img,CV_64F, 1./255);

	cv::Mat ret(img.rows, img.cols, CV_16S);

	for(int  u= 0; u< img.rows; u++){
		double* row_src = img.ptr<double>(u);
		short* row_dst = ret.ptr<short>(u);
		  for(int v = 0; v< img.cols; v++)
		   {
			  row_dst[v] = (short)ceil(row_src[v]);
		   }
	}

	return ret;
}

//} /* namespace structured_light_scanner */
