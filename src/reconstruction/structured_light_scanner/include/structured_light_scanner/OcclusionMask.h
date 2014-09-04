/*
 * OcclusionMask.h
 *
 *  Created on: Apr 1, 2013
 *      Author: thomas
 */

#ifndef OCCLUSIONMASK_H_
#define OCCLUSIONMASK_H_

//=============================================================================
// OPENCV Includes
//=============================================================================
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <math.h>
//namespace perception {

// OPENCV WINDOWS LOCATIONS
//***************************************
#define WPOS1X 20
#define WPOS1Y 10
#define WPOS2X 420
#define WPOS2Y 10
#define WPOS3X 820
#define WPOS3Y 10

class OcclusionMask {
public:
	OcclusionMask();
	virtual ~OcclusionMask();

	void createOcclusionMask(cv::Mat& black, cv::Mat& white);
	cv::Mat getOcclusionMask(){return OccMask;};
	cv::Mat getOcclusionMaskLogical();
	void showOcclusionMask();

private:
	void thresholdImage(cv::Mat& src, cv::Mat& dst);

	cv::Mat OccMask;
};

//} /* namespace perception */
#endif /* OCCLUSIONMASK_H_ */
