/*
 * OcclusionMask.h
 *
 *  Created on: Apr 1, 2013
 *      Author: thomas
 */

#ifndef OCCLUSION_MASK_H_
#define OCCLUSION_MASK_H_

//=============================================================================
// OPENCV Includes
//=============================================================================
#include <opencv2/opencv.hpp>

#include <math.h>
//namespace perception {

class OcclusionMask {
public:
	OcclusionMask();
	virtual ~OcclusionMask();

	void addBlackImage(const cv::Mat& black);
	void addWhiteImage(const cv::Mat& white);

	bool createOcclusionMask();
	bool hasWhiteImage() { return !white_.empty(); };
	bool hasBlackImage() { return !black_.empty(); };
	const cv::Mat& getWhiteImage() { return white_; };

	const cv::Mat& getOcclusionMask() { return occ_mask_; };
	void showOcclusionMask();
	void reset();

private:
	cv::Mat occ_mask_;
	cv::Mat white_;
	cv::Mat black_;
};

//} /* namespace perception */
#endif /* OCCLUSION_MASK_H_ */
