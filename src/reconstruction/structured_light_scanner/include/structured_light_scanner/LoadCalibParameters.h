/********************************************************************************************************************
 *
 * @file		LoadCalibParameters.h
 * @author		Thomas Sølund (thso@teknologisk.dk)
 * @date		2013-01-30
 * @version		1.0
 * @brief		Class to load various calibration files
 *
*********************************************************************************************************************/

#ifndef LOADCALIBPARAMETERS_H_
#define LOADCALIBPARAMETERS_H_

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "Logger/DTILogger.h"

#include <camera_calibration_parsers/parse_yml.h>
#include "sensor_msgs/CameraInfo.h"

// File IO
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>


namespace structured_light_scanner {

class LoadCalibParameters {
public:
	LoadCalibParameters();
	virtual ~LoadCalibParameters();

	void loadCameraParams(std::string MyPath, std::string camera_name);
	boost::array<double, 9> getIntrinsicCameraMatrix(void){return CameraInfo.K;};
	//void getIntrinsicCameraMatrixOpenCV(cv::Mat &dst);
	void getIntrinsicCameraMatrixOpenCV(cv::Mat &dst);

	void getDistortionParams_OpenCV(cv::Mat &dst);
	void getProjectionMatrix_OpenCV(cv::Mat &dst);
	void getRMatrix_OpenCV(cv::Mat &dst);
//	boost::array<double, 5> getDistortionParams(void){return CameraInfo.D;};

	int getImageHeight(void){return (int)CameraInfo.height;};
	int getImageWidth(void){return (int)CameraInfo.width;};

private:

	sensor_msgs::CameraInfo CameraInfo;
};

} /* namespace structured_light_scanner */
#endif /* LOADCALIBPARAMETERS_H_ */
