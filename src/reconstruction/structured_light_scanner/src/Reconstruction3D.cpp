/********************************************************************************************************************
 *
 * Copyright (c) 2013
 *
 * Danish Technological Institute (DTI)
 *
 *-------------------------------------------------------------------------------------------------------------------
 *
 * @file       Reconstruction3D.cpp
 * @Author     Thomas Sølund (thso@dti.dk)
 * @brief
 * @details
 * @addtogroup
 *
 * $Revision: 4075 $
 * $Date: 2013-04-18 12:17:47 +0100 (Tue, 12 Nov 2013) $
 * $Author: thso $
 * $Id: Reconstruction3D.cpp 4075 2013-04-18 11:17:47Z thso $
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
#include <structured_light_scanner/Reconstruction3D.h>

namespace structured_light_scanner {

Reconstruction_3D::Reconstruction_3D() {
	// TODO Auto-generated constructor stub


	// EncImLeft = cv::Mat(img_height, img_width,CV_16U);
	// EncImLeft.zeros(cv::Size(img_height, img_width),CV_16U);

    // EncImRight = cv::Mat(img_height, img_width,CV_16U);
	// EncImRight.zeros(cv::Size(img_height, img_width),CV_16U);

}

Reconstruction_3D::~Reconstruction_3D() {
	// TODO Auto-generated destructor stub
}

void Reconstruction_3D::cleanup()
{
	diffImLeft.clear();
	diffImRight.clear();

}

void Reconstruction_3D::createEncodedImage(cv::Mat& negative, cv::Mat& positive, cv::Mat& OMask, cv::Mat& EncodedImage, int unsigned code, bool isLeft)
{
	 cv::Mat rect_img(img_height,img_width,CV_16S);
	 cv::Mat sub_img(img_height,img_width,CV_16S);
	 cv::Mat diff_img(img_height,img_width,CV_16S);
	 cv::Mat gauss_blur(img_height,img_width,CV_16S);

	 cv::Mat binary_img(img_height,img_width,CV_16S);
	 cv::Mat out_img(img_height,img_width,CV_16U);
	 cv::Mat temp(img_height,img_width,CV_16U);

	 cv::subtract(positive, negative, sub_img, cv::Mat(), CV_16S);

	 rect_img = sub_img.mul(OMask,1);

	 if(isLeft)cv::remap(rect_img, gauss_blur,map1x, map1y, CV_INTER_LINEAR);
	 else cv::remap(rect_img, gauss_blur,map2x, map2y, CV_INTER_LINEAR);

	 cv::GaussianBlur(gauss_blur, diff_img,cv::Size(3,3),0.9);

	 if(isLeft) diffImLeft.push_back(diff_img);
	 else diffImRight.push_back(diff_img);

	cv::threshold(diff_img,binary_img,1,255,CV_THRESH_BINARY);
	cv::Mat bin_16U;
	binary_img.convertTo(bin_16U,CV_16U, 1./255);

	out_img = pow(2,code) * bin_16U;

	cv::add(out_img, EncodedImage,temp);
	temp.copyTo(EncodedImage);

}

bool Reconstruction_3D::rectifyImage(cv::Mat& src, cv::Mat& dst){

	try{
		cv::remap(src, dst,map1x, map1y, CV_INTER_LINEAR);
	}catch(cv::Exception &e){
		std::cout << e.what() << std::endl;
	}
	return true;
}

int Reconstruction_3D::sign(int x)
{
	if (x > 0) return 1;
	if (x == 0) return 0;
	if (x < 0) return -1;
}

bool Reconstruction_3D::STR_Interpolate(int row, int col, const cv::Mat& EncImage, bool isLeft)
{
	offset = 0;

	cv::Matx<double, 4, 2> MatA;
	MatA.val[0] = -1; MatA.val[1] = 0;
	MatA.val[2] = 0; MatA.val[3] = 1;
	MatA.val[4] = 1; MatA.val[5] = 1;
	MatA.val[6] = 2; MatA.val[7] = 1;

	double cBit = log2(abs((short)EncImage.at<short>(row,col)- (short)EncImage.at<short>(row,col+1)));

	if(cBit - floor(cBit) != 0){
		return false;
	}
	if(cBit == 0){
		return false;
	}

	if(cBit > diffImLeft.size() || cBit > diffImRight.size()){
		return false;
	}


  	cv::Mat dif;
  	if(isLeft){
  		dif= diffImLeft.at(cBit-1);
  	}else{
  		dif= diffImRight.at(cBit-1);
  	}

  	cv::Matx<double, 4, 1> Matdata;
	short* row_ptr = dif.ptr<short>(row);

	Matdata.val[0] = (double)row_ptr[col-1];
	Matdata.val[1] = (double)row_ptr[col];
	Matdata.val[2] = (double)row_ptr[col+1];
	Matdata.val[3] = (double)row_ptr[col+2];

	if(sign(Matdata.val[0]) == sign(Matdata.val[3]))
	{
		return false;
	}

	cv::Mat x;
	cv::solve(MatA,Matdata, x,cv::DECOMP_NORMAL);

	if(abs(x.at<double>(0))<5){
		return false;
	}

	offset = -x.at<double>(1)/x.at<double>(0);

	if(sign(Matdata.val[1]) != sign(Matdata.val[2])){
		offset = Matdata.val[1]/(Matdata.val[1]-Matdata.val[2]); 			//linear interpolation
	}

	if(sign(Matdata.val[0]) != sign(Matdata.val[1])){
			offset = Matdata.val[0]/((Matdata.val[0]-Matdata.val[2])-1); 	//linear interpolation
	}

	if(sign(Matdata.val[2]) != sign(Matdata.val[3])){
			offset = Matdata.val[2]/((Matdata.val[2]-Matdata.val[3])+1); 	//linear interpolation
	}

	return true;
}

void Reconstruction_3D::Init_rectification(cv::Mat camera_matrix1,
										   cv::Mat distortion_1,
										   cv::Mat camera_matrix2,
										   cv::Mat distortion_2,
										   cv::Mat R,
										   cv::Mat T)
{
	cv::Size newImageSize;
	cv::Size imageSize = cv::Size(img_width, img_height);

	try{
		 cv::stereoRectify(camera_matrix1,
				 	 	   distortion_1,
				 	 	   camera_matrix2,
				 	 	   distortion_2,
				 	 	   imageSize,
				 	 	   R,
				 	 	   T,
				 	 	   R1,
				 	 	   R2,
				 	 	   P1,
				 	 	   P2,
				 	 	   Q,
				 	 	   0,
				 	 	   -1);



/*	R1.at<double>(0,0) = 0.9915798396746804;
	R1.at<double>(0,1) = -0.001571727592909963; //0.007250461837554449;
	R1.at<double>(0,2) = 0.1294872627832891;//0.2271748762742498;
	R1.at<double>(1,0) =   0.002692474963888934;//0.002536518705310051;
	R1.at<double>(1,1) =0.9999604136387074;//0.9999147370895781;
	R1.at<double>(1,2) =  -0.008480668256346722;//-0.012223823366387738;
	R1.at<double>(2,0) = -0.1294688075534169;//-0.2272441350479509;
	R1.at<double>(2,1) =  0.008757900883148949;//0.010860449080125102;
	R1.at<double>(2,2) =  0.9915448184740906;//0.9737772608415575;
*/


/*	P1.at<double>(0,0) = 793.5253191883772;
	P1.at<double>(0,1) = 0;
	P1.at<double>(0,2) = 208.9929695129395;
	P1.at<double>(0,3) = 0;
	P1.at<double>(1,0) = 0;
	P1.at<double>(1,1) = 793.5253191883772;
	P1.at<double>(1,2) = 230.6744804382324;
	P1.at<double>(1,3) = 0;
	P1.at<double>(2,0) = 0;
	P1.at<double>(2,1) = 0;
	P1.at<double>(2,2) = 1;
	P1.at<double>(2,3) = 0;

	P2.at<double>(0,0) = 793.5253191883772;
	P2.at<double>(0,1) = 0;
	P2.at<double>(0,2) =  590.356273651123;
	P2.at<double>(0,3) = -167765.568271577;
	P2.at<double>(1,0) = 0;
	P2.at<double>(1,1) = 793.5253191883772;
	P2.at<double>(1,2) = 230.6744804382324;
	P2.at<double>(1,3) = 0;
	P2.at<double>(2,0) = 0;
	P2.at<double>(2,1) = 0;
	P2.at<double>(2,2) = 1;
	P2.at<double>(2,3) = 0;

	Q.at<double>(0,0) = 1;
	Q.at<double>(0,1) = 0;
	Q.at<double>(0,2) = 0;
	Q.at<double>(0,3) = -208.9929695129395;
	Q.at<double>(1,0) = 0;
	Q.at<double>(1,1) = 1;
	Q.at<double>(1,2) = 0;
	Q.at<double>(1,3) = -230.6744804382324;
	Q.at<double>(2,0) = 0;
	Q.at<double>(2,1) = 0;
	Q.at<double>(2,2) = 0;
	Q.at<double>(2,3) = 793.5253191883772;
	Q.at<double>(3,0) = 0;
	Q.at<double>(3,1) = 0;
	Q.at<double>(3,2) = 0.004729965316266968;
	Q.at<double>(3,3) = 1.80383520147058;
*/
		 //cv::Rodrigues(R1,R1_rod);
		 //cv::Rodrigues(R2,R2_rod);
		 std::cout << "R1:\n" << R1 << std::endl;
		 //std::cout << "R1_rodrigues:\n" << R1_rod << std::endl;
		 std::cout << "P1:\n" << P1 << std::endl;

		 std::cout << "R2:\n" << R2 << std::endl;
		 //std::cout << "R2_rodrigues:\n" << R2_rod << std::endl;
		 std::cout << "P2:\n" << P2 << std::endl;

		 std::cout << "Q:\n" << Q << std::endl;

		 cv::initUndistortRectifyMap(camera_matrix1,
				 	 	 	 	 	 distortion_1,
				 	 	 	 	 	 R1,
				 	 	 	 	 	 P1,
				 	 	 	 	 	 imageSize,
				 	 	 	 	 	 CV_16SC2,
				 	 	 	 	 	 map1x,
				 	 	 	 	 	 map1y);

		 cv::initUndistortRectifyMap(camera_matrix2,
				 	 	 	 	 	 distortion_2,
		 			 	 	 	 	 R2,
		 			 	 	 	 	 P2,
		 			 	 	 	 	 imageSize,
		 			 	 	 	 	 CV_16SC2,
		 			 	 	 	 	 map2x,
		 			 	 	 	 	 map2y);
		 }catch(cv::Exception &E){
			 std::cout << "in stereoRectify " << E.what() << std::endl;
		 }

}

void Reconstruction_3D::Init_rectification2(cv::Mat camera_matrix1,
										   cv::Mat distortion_1,
										   cv::Mat camera_matrix2,
										   cv::Mat distortion_2,
										   cv::Mat R_L,
										   cv::Mat R_R)
{
	cv::Size newImageSize;
	cv::Size imageSize = cv::Size(img_width, img_height);

	try{

		 cv::initUndistortRectifyMap(camera_matrix1,
				 	 	 	 	 	 distortion_1,
				 	 	 	 	 	 R_L,
				 	 	 	 	 	 cv::getOptimalNewCameraMatrix(camera_matrix1,distortion_1,imageSize,1,imageSize,0),
				 	 	 	 	 	 imageSize,
				 	 	 	 	 	 CV_16SC2,
				 	 	 	 	 	 map1x,
				 	 	 	 	 	 map1y);

		 cv::initUndistortRectifyMap(camera_matrix2,
				 	 	 	 	 	 distortion_2,
		 			 	 	 	 	 R_R,
		 			 	 	 	     cv::getOptimalNewCameraMatrix(camera_matrix2,distortion_2,imageSize,1,imageSize,0),
		 			 	 	 	 	 imageSize,
		 			 	 	 	 	 CV_16SC2,
		 			 	 	 	 	 map2x,
		 			 	 	 	 	 map2y);
		 }catch(cv::Exception &E){
			 std::cout << "in stereoRectify " << E.what() << std::endl;
		 }

}

pcl::PointCloud<pcl::PointXYZ>::Ptr Reconstruction_3D::ComputePointCloudXYZ(cv::Mat& EncImLeft, cv::Mat& EncImRight)
{
	std::cout << "Creating Point Cloud..." <<std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

	vector<vector<double> > Edge1;
	vector<vector<double> > Edge2;
	int point_cnt = 0;

	std::cout << "Searching for correspondences" << std::endl;
	struct timeval tod1;
	double t2 = 0.0, t1 = 0.0;

	gettimeofday(&tod1,NULL);
	t1 = tod1.tv_sec + tod1.tv_usec / 1000000.0;

	for(int u = 0; u< EncImLeft.rows; u++){
	 	unsigned short* lPtr = EncImLeft.ptr<unsigned short>(u);
	 	unsigned short* rPtr = EncImRight.ptr<unsigned short>(u);
	 		 for(int  v= 0; v< EncImLeft.cols; v++)
	 		 {
	 			 if(lPtr[v] != lPtr[v+1]){
	 				 if(STR_Interpolate(u,v,EncImLeft, true)){

	 				    vector<double> xL;
	 					xL.push_back((double)lPtr[v]);
	 					xL.push_back((double)lPtr[v+1]);
	 					xL.push_back( (double)(v + offset));
	 					Edge1.push_back(xL);
	 				 }

	 			 }

	 			 if(rPtr[v] != rPtr[v+1]){
	 				 if(STR_Interpolate(u,v,EncImRight, false)){
	 					vector<double> xR;
	 					xR.push_back((double)rPtr[v]);
	 					xR.push_back((double)rPtr[v+1]);
	 					xR.push_back((double)(v + offset));
	 				    Edge2.push_back(xR);
	 				}


	 			 }

	 		 }

	 		std::sort(Edge1.begin(),Edge1.end(),SortEdge);
	 		std::sort(Edge2.begin(),Edge2.end(),SortEdge);

	 		if(!Edge1.empty() && !Edge2.empty()){
	 			unsigned int idx1=0;
	 			unsigned int idx2=0;
	 			int Fact= pow(2,9+1); //NB: compute nBit (9) instead of hardcoding

	 			while(idx1< Edge1.size() && idx2 < Edge2.size()){
	 				vector<double> p1 = Edge1.at(idx1);
	 				vector<double> p2 = Edge2.at(idx2);

	 				int Num1 = p1[0]*Fact + p1[1];
	 				int Num2 = p2[0]*Fact + p2[1];

	 				if(Num1<Num2)idx1=idx1+1;
	 				if(Num1>Num2)idx2=idx2+1;

	 				if(Num1 == Num2){

	 					cv::Point3d img_point1;
	 					img_point1.x = u;//100;// u;
	 					img_point1.y = p1[2];//259.550;//p1[2];
	 					img_point1.z = 1;

	 					cv::Point3d img_point2;
	 					img_point2.x = u;//100;// u;
	 					img_point2.y = p2[2];//270.7304;//p2[2];
	 					img_point2.z = 1;

	 					//cv::Mat p3d = LinearLSTriangulation(img_point1,P1,img_point2,P2);
	 				//	cv::Mat p3d = LinearLSTriangulation(img_point1,cam1,img_point2,cam1);
	 				//	cv::Mat p3d =  IterativeLinearLSTriangulation(img_point1,P1,img_point2,P2);
	 					cv::Mat p3d = dti_triangulation(img_point1,P1,img_point2,P2);

	 					pcl::PointXYZ point;
	 					point.x = p3d.at<double>(0)/1000;
	 					point.y = p3d.at<double>(1)/1000;
	 					point.z = p3d.at<double>(2)/1000;

	 					point_cloud_ptr->points.push_back(point);

	 					point_cnt++;

	 					idx1=idx1+1;
	 					idx2=idx2+1;

	 					p1.clear();
	 					p2.clear();
	 				}
	 			}
	 		}
	 		Edge1.clear();
	 		Edge2.clear();

	 }

	gettimeofday(&tod1,NULL);
	t2 = tod1.tv_sec + tod1.tv_usec / 1000000.0;
	reconstruction_time = 1000.0*(t2 - t1);

	cout << "3D reconstruction time: " << reconstruction_time << " ms" << endl;

	std::cout << "point_cnt: " << point_cnt << std::endl;

	return point_cloud_ptr;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr Reconstruction_3D::ComputePointCloudXYZRGB(cv::Mat& EncImLeft, cv::Mat& EncImRight, cv::Mat& rgb_image)
{
	std::cout << "Creating Point Cloud..." <<std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

	vector<vector<double> > Edge1;
	vector<vector<double> > Edge2;
	int point_cnt = 0;

	std::cout << "Searching for correspondences" << std::endl;
	struct timeval tod1;
	double t2 = 0.0, t1 = 0.0;

	gettimeofday(&tod1,NULL);
	t1 = tod1.tv_sec + tod1.tv_usec / 1000000.0;

	for(int u = 0; u< EncImLeft.rows; u++){
	 	unsigned short* lPtr = EncImLeft.ptr<unsigned short>(u);
	 	unsigned short* rPtr = EncImRight.ptr<unsigned short>(u);
	 		 for(int  v= 0; v< EncImLeft.cols; v++)
	 		 {
	 			 if(lPtr[v] != lPtr[v+1]){
	 				 if(STR_Interpolate(u,v,EncImLeft, true)){

	 				    vector<double> xL;
	 					xL.push_back((double)lPtr[v]);
	 					xL.push_back((double)lPtr[v+1]);
	 					xL.push_back( (double)(v + offset));
	 					xL.push_back( (double)(v));
	 					Edge1.push_back(xL);
	 					xL.clear();
	 				 }

	 			 }

	 			 if(rPtr[v] != rPtr[v+1]){
	 				 if(STR_Interpolate(u,v,EncImRight, false)){
	 					vector<double> xR;
	 					xR.push_back((double)rPtr[v]);
	 					xR.push_back((double)rPtr[v+1]);
	 					xR.push_back( (double)(v + offset));
	 					xR.push_back( (double)(v));
	 				    Edge2.push_back(xR);
	 				    xR.clear();
	 				}
	 			 }


	 		 }

	 		std::sort(Edge1.begin(),Edge1.end(),SortEdge);
	 		std::sort(Edge2.begin(),Edge2.end(),SortEdge);

	 		if(!Edge1.empty() && !Edge2.empty()){
	 			int idx1=0;
	 			int idx2=0;
	 			int Fact= pow(2,9+1); //NB: compute nBit (9) instead of hardcoding

	 			while(idx1< Edge1.size() && idx2 < Edge2.size()){
	 				vector<double> p1 = Edge1.at(idx1);
	 				vector<double> p2 = Edge2.at(idx2);

	 				int Num1 = p1[0]*Fact + p1[1];
	 				int Num2 = p2[0]*Fact + p2[1];

	 				if(Num1<Num2)idx1=idx1+1;
	 				if(Num1>Num2)idx2=idx2+1;

	 				if(Num1 == Num2){

	 					cv::Point3d img_point1;
	 					img_point1.x = u;//100;// u;
	 					img_point1.y = p1[2];//259.550;//p1[2];
	 					img_point1.z = 1;

	 					cv::Point3d img_point2;
	 					img_point2.x = u;//100;// u;
	 					img_point2.y = p2[2];//270.7304;//p2[2];
	 					img_point2.z = 1;

	 					//cv::Mat p3d = LinearLSTriangulation(img_point1,P1,img_point2,P2);
	 				//	cv::Mat p3d = LinearLSTriangulation(img_point1,cam1,img_point2,cam1);
	 				//	cv::Mat p3d =  IterativeLinearLSTriangulation(img_point1,P1,img_point2,P2);
	 					cv::Mat p3d = dti_triangulation(img_point1,P1,img_point2,P2);

	 					// divide with 1000 to convert from m to mm
	 					pcl::PointXYZRGB point;
	 					point.x = p3d.at<double>(0)/1000;
	 					point.y = p3d.at<double>(1)/1000;
	 					point.z = p3d.at<double>(2)/1000;
	 					cv::Vec3b color = rgb_image.at<cv::Vec3b>(u,p1[3]);
	 					point.b = (unsigned int)color.val[0];
	 					point.g = (unsigned int)color.val[1];
	 					point.r = (unsigned int)color.val[2];

	 					//std::cout << "x: " << point.x << " y: " << point.y << " z: " << point.z << std::endl;
	 					point_cloud_ptr->points.push_back(point);

	 					point_cnt++;

	 					idx1=idx1+1;
	 					idx2=idx2+1;
	 				}
	 			}
	 		}
	 		Edge1.clear();
	 		Edge2.clear();

	 }

	gettimeofday(&tod1,NULL);
	t2 = tod1.tv_sec + tod1.tv_usec / 1000000.0;
	reconstruction_time = 1000.0*(t2 - t1);

	cout << "3D reconstruction time: " << reconstruction_time << " ms" << endl;

	std::cout << "point_cnt: " << point_cnt << std::endl;

	return point_cloud_ptr;
}

cv::Mat Reconstruction_3D::dti_triangulation(cv::Point3d u,       //homogenous image point (u,v,1)
   	       	   	   	   	   	   	   	   	   	 cv::Matx34d P,       //camera 1 matrix
   	       	   	   	   	   	   	   	   	   	 cv::Point3d u1,      //homogenous image point in 2nd camera
   	       	   	   	   	   	   	   	   	   	 cv::Matx34d P1       //camera 2 matrix
   	       	   	   	   	   	   	   	   	   	   )
{
	cv::Matx44d B;
	B.zeros();

	cv::Matx<double, 1, 4> B_1 = (P.row(0)- u.y*P.row(2));
	cv::Matx<double, 1, 4> B_2 = (P.row(1)- u.x*P.row(2));
	cv::Matx<double, 1, 4> B_3 = (P1.row(0)- u1.y*P1.row(2));
	cv::Matx<double, 1, 4> B_4 = (P1.row(1)- u1.x*P1.row(2));

	B.val[0] = B_1.val[0]; B.val[1] = B_1.val[1]; B.val[2] = B_1.val[2]; B.val[3] = B_1.val[3];
	B.val[4] = B_2.val[0]; B.val[5] = B_2.val[1]; B.val[6] = B_2.val[2]; B.val[7] = B_2.val[3];
	B.val[8] = B_3.val[0]; B.val[9] = B_3.val[1]; B.val[10] = B_3.val[2]; B.val[11] = B_3.val[3];
	B.val[12] = B_4.val[0]; B.val[13] = B_4.val[1]; B.val[14] = B_4.val[2]; B.val[15] = B_4.val[3];

	cv::SVD *svd = new cv::SVD(B, cv::SVD::FULL_UV);
	cv::Mat U = svd->u;
	cv::Mat vt = svd->vt;
	cv::Mat w = svd->w;

	cv::Mat Q = vt.row(3)/ vt.at<double>(3,3);

	delete svd;

	return Q;

}
/*
/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
cv::Mat Reconstruction_3D::LinearLSTriangulation(cv::Point3d u,       //homogenous image point (u,v,1)
                   	   	       cv::Matx34d P,       //camera 1 matrix
                   	   	       cv::Point3d u1,      //homogenous image point in 2nd camera
                   	   	       cv::Matx34d P1       //camera 2 matrix
                                   	   	   )
{
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    cv::Matx43d A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),
    			  u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),
                  u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),
                  u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2)
                  );

    cv::Matx41d B(-(u.x*P(2,3) -P(0,3)),
    			  -(u.y*P(2,3) -P(1,3)),
    			  -(u1.x*P1(2,3) -P1(0,3)),
    			  -(u1.y*P1(2,3) -P1(1,3)));

    cv::Mat X;
    cv::solve(A,B,X,cv::DECOMP_SVD);

    return X;
}

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
cv::Mat_<double> Reconstruction_3D::IterativeLinearLSTriangulation(cv::Point3d u,    //homogenous image point (u,v,1)
                                            	cv::Matx34d P,          //camera 1 matrix
                                            	cv::Point3d u1,         //homogenous image point in 2nd camera
                                            	cv::Matx34d P1          //camera 2 matrix

) {
   // double wi = 1, wi1 = 1;
    cv::Mat_<double> X(4,1);
  /*  for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
        cv::Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;

        //recalculate weights
        double p2x = cv::Mat_<double>(cv::Mat_<double>(P).row(2)*X)(0);
        double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);

        //breaking point
        if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

        wi = p2x;
        wi1 = p2x1;

        //reweight equations and solve
        cv::Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,
        			(u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,
                    (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1,
                    (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                  );
        cv::Mat_<double> B = (cv::Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                          	  	  	  	  	  	  	  	  -(u.y*P(2,3)  -P(1,3))/wi,
                          	  	  	  	  	  	  	  	  -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                          	  	  	  	  	  	  	  	  -(u1.y*P1(2,3)    -P1(1,3))/wi1
        													);

        cv::solve(A,B,X_,cv::DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;
    }
    */
    return X;
}

void Reconstruction_3D::triangulate(cv::Matx31d u,    //homogenous image point (u,v,1)
    	cv::Matx34d P,          //camera 1 matrix
    	cv::Matx31d u1,         //homogenous image point in 2nd camera
    	cv::Matx34d P1          //camera 2 matrix
    	)
{
	cv::Mat t;
	try{
		cv::triangulatePoints(P,P1,u,u1,t);
	}catch(cv::Exception &e){
		std::cout << e.what() << " in triangulatePoints()" << std::endl;
	}
}

bool Reconstruction_3D::SortEdge(const std::vector<double>& a, const std::vector<double>& b)
{
	return a[0] < b[0];
}

//------------------------------------------------------------//
//			       DEBUGGING FUNCTIONS						  //
//------------------------------------------------------------//


boost::shared_ptr<pcl::visualization::PCLVisualizer> Reconstruction_3D::simpleVisualizer (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->resetCameraViewpoint("sample cloud");
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Reconstruction_3D::rgbVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->resetCameraViewpoint("sample cloud");

  return (viewer);
}

void Reconstruction_3D::test_rectification(cv::Mat& left, cv::Mat& right)
{
	 cv::Mat leftr, rightr, leftr_rgb, rightr_rgb;
	 cv::remap(left, leftr,map1x, map1y, CV_INTER_LINEAR);
	 cv::remap(right, rightr,map2x, map2y, CV_INTER_LINEAR);

	 cv::cvtColor(leftr, leftr_rgb, CV_GRAY2BGR );
	 cv::cvtColor(rightr, rightr_rgb, CV_GRAY2BGR );

	 cv::Size sz1 = left.size();
	 cv::Size sz2 = right.size();

	 cv::Mat im3(sz1.height, sz1.width+sz2.width, leftr_rgb.type());

	 cv::Mat left1(im3, cv::Rect(0, 0, sz1.width, sz1.height));
	 leftr_rgb.copyTo(left1);
	 cv::Mat right1(im3, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
	 rightr_rgb.copyTo(right1);

	 //Add horizontal lines to the screen
	 for( int j = 0; j < im3.rows; j += 50 ){
		 cv::line(im3,cv::Point(0,j),cv::Point(im3.cols,j),cv::Scalar(0,200,0),1,CV_AA);
	 	 }

	cv::namedWindow("Epipolar lines",CV_WINDOW_NORMAL|CV_WINDOW_FREERATIO);
	cv::imshow("Epipolar lines", im3);
	cv::resizeWindow("Epipolar lines", im3.cols/2, im3.rows/2);
	//cv::moveWindow("approximateZeroFilter",WPOS3X, WPOS3Y);
	cv::waitKey(0);
}


void Reconstruction_3D::writeImageToFile(std::string name, cv::Mat& image)
{
	ofstream myfile;
	myfile.open(name.c_str());

	 for(int u = 0; u< image.rows; u++){
		  short* lPtr = image.ptr<  short>(u);
		 for(int  v= 0; v< image.cols; v++)
		 {
		//if((short)(lPtr[v]) != 0)
				myfile << ( short)(lPtr[v])  << " ";

		 }
		 	 myfile << std::endl;
	//	 std::cout << "pixels in row: " << image.cols << std::endl;
	}
		 myfile.close();
}
void Reconstruction_3D::writeDoubleImageToFile(std::string name, cv::Mat& image)
{
	ofstream myfile;
	myfile.open(name.c_str());

	 for(int u = 0; u< image.rows; u++){
		  double* lPtr = image.ptr<double>(u);
		 for(int  v= 0; v< image.cols; v++)
		 {
			// cv::Vec3f vec = image.at<cv::Vec3f>(u,v);
		//if((short)(lPtr[v]) != 0)
				myfile << (double)(lPtr[v])  << " ";
			// myfile << vec.val[1]  << " ";

		 }
		 	 myfile << std::endl;
	//	 std::cout << "pixels in row: " << image.cols << std::endl;
	}
		 myfile.close();
}

void Reconstruction_3D::writefloatImageToFile(std::string name, cv::Mat& image)
{
	ofstream myfile;
	myfile.open(name.c_str());

	 for(int u = 0; u< image.rows; u++){
		  float* lPtr = image.ptr<float>(u);
		 for(int  v= 0; v< image.cols; v++)
		 {
				myfile << (float)(lPtr[v])  << " ";
		 }
		 	 myfile << std::endl;
	}
		 myfile.close();
}

void Reconstruction_3D::writeByteImageToFile(std::string name, cv::Mat& image)
{
	ofstream myfile;
	myfile.open(name.c_str());

	 for(int u = 0; u< image.rows; u++){
		  uchar* lPtr = image.ptr<  uchar>(u);
		 for(int  v= 0; v< image.cols; v++)
		 {
				myfile << ( int)(lPtr[v])  << " ";

		 }
		 	 myfile << std::endl;
	}
		 myfile.close();
}

} /* namespace perception */
