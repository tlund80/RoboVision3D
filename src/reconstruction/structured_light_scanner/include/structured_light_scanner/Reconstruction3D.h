/*
 * Reconstruction3D.h
 *
 *  Created on: Apr 18, 2013
 *      Author: thomas
 */

#ifndef RECONSTRUCTION3D_H_
#define RECONSTRUCTION3D_H_

//=============================================================================
// OPENCV Includes
//=============================================================================
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

//=============================================================================
// PCL Includes
//=============================================================================
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
//=============================================================================
// System Includes
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <assert.h>
//#include <algorithm>
//#include <sys/time.h>
#include <iostream>

#include <math.h>
#include <cmath>

#include "Imagebuffer.h"

namespace structured_light_scanner {

class Reconstruction_3D {
public:
	Reconstruction_3D();
	virtual ~Reconstruction_3D();
	void cleanup();

	void createEncodedImage(cv::Mat& negative, cv::Mat& positive, cv::Mat& OMask, cv::Mat& EncodedImage, unsigned int code, bool isLeft);
	bool STR_Interpolate(int row, int col, const cv::Mat& EncImage, bool isLeft);
	void Init_rectification(cv::Mat camera_matrix1,
		   				    cv::Mat distortion_1,
							cv::Mat camera_matrix2,
							cv::Mat distortion_2,
							cv::Mat R,
							cv::Mat T);

	void Init_rectification2(cv::Mat camera_matrix1,
		     				 cv::Mat distortion_1,
							 cv::Mat camera_matrix2,
							 cv::Mat distortion_2,
							 cv::Mat R_L,
							 cv::Mat R_R);

	pcl::PointCloud<pcl::PointXYZ>::Ptr ComputePointCloudXYZ(cv::Mat& EncImLeft, cv::Mat& EncImRight);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ComputePointCloudXYZRGB(cv::Mat& EncImLeft, cv::Mat& EncImRight, cv::Mat& rgb_image);

	cv::Mat dti_triangulation(cv::Point3d u,       //homogenous image point (u,v,1)
	   	       	   	   	   	  cv::Matx34d P,       //camera 1 matrix
	   	       	   	   	   	  cv::Point3d u1,      //homogenous image point in 2nd camera
	   	       	   	   	   	  cv::Matx34d P1       //camera 2 matrix
	   	       	   	   	   	  );
	cv::Mat LinearLSTriangulation(cv::Point3d u,       //homogenous image point (u,v,1)
								  cv::Matx34d P,       //camera 1 matrix
								  cv::Point3d u1,      //homogenous image point in 2nd camera
	                  	   	      cv::Matx34d P1       //camera 2 matrix
	                              );

	cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d u,    //homogenous image point (u,v,1)
	                                            	cv::Matx34d P,    //camera 1 matrix
	                                            	cv::Point3d u1,   //homogenous image point in 2nd camera
	                                            	cv::Matx34d P1    //camera 2 matrix
													);
	void triangulate(cv::Matx31d u,    //homogenous image point (u,v,1)
	       			 cv::Matx34d P,    //camera 1 matrix
	    			 cv::Matx31d u1,   //homogenous image point in 2nd camera
	    			 cv::Matx34d P1    //camera 2 matrix
	    			);

	bool rectifyImage(cv::Mat& src, cv::Mat& dst);

	// Debugging functions!!!
	void test_rectification(cv::Mat& left, cv::Mat& right);
	void writeImageToFile(std::string name, cv::Mat& image);
	void writeDoubleImageToFile(std::string name, cv::Mat& image);
	void writefloatImageToFile(std::string name, cv::Mat& image);
	void writeByteImageToFile(std::string name, cv::Mat& image);

	void setImageHeight(int image_height){img_height = image_height;};
	void setImageWidth(int image_width){img_width = image_width;};

	boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVisualizer (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

	void getProjectionMatrix1(cv::Mat& _P1){P1.copyTo(_P1);};
	void getProjectionMatrix2(cv::Mat& _P2){P2.copyTo(_P2);};

	void getRectificationMatrix1(cv::Mat& _R1){R1.copyTo(_R1);};
	void getRectificationMatrix2(cv::Mat& _R2){R2.copyTo(_R2);};

	double getReconstructionTime(void){ return reconstruction_time;};

private:

	int img_height,img_width;
	cv::Mat map1x, map1y, map2x, map2y;
	vector<cv::Mat> diffImLeft;
	vector<cv::Mat> diffImRight;
	unsigned int num_of_pattern;
	double offset;
	double reconstruction_time;

	//Result of rectification
	cv::Mat R1,R2, P1, P2, Q;

	// cv::Mat EncImLeft;
	// cv::Mat EncImRight;

	int sign(int x);
	static bool SortEdge(const std::vector<double>& a, const std::vector<double>& b);
};

} /* namespace perception */
#endif /* RECONSTRUCTION3D_H_ */
