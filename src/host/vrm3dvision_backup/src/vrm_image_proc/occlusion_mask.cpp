/*
 * OcclusionMask.cpp
 *
 *  Created on: Apr 1, 2013
 *      Author: thomas
 */

#include "occlusion_mask.hpp"

//namespace perception {

OcclusionMask::OcclusionMask() {
	// TODO Auto-generated constructor stub

}

OcclusionMask::~OcclusionMask() {
	// TODO Auto-generated destructor stub
}

void OcclusionMask::reset()
{
	black_.release();
	white_.release();
	occ_mask_.release();
}

void OcclusionMask::addBlackImage(const cv::Mat& black)
{
	black.copyTo(black_);
	if (hasBlackImage() && hasWhiteImage())
	{
		createOcclusionMask();
	}
}

void OcclusionMask::addWhiteImage(const cv::Mat& white)
{
	white.copyTo(white_);
	if (hasBlackImage() && hasWhiteImage())
	{
		createOcclusionMask();
	}
}

void OcclusionMask::showOcclusionMask()
{
	if(!occ_mask_.empty())
	{
		cv::Mat img;
		cv::threshold(occ_mask_, img, 0.5, 255, CV_THRESH_BINARY);

		cv::namedWindow("Occlusion Mask",CV_WINDOW_NORMAL);
		cv::imshow("Occlusion Mask", img);
		cv::resizeWindow("Occlusion Mask", img.cols, img.rows);
		cv::waitKey();
	}
}

bool OcclusionMask::createOcclusionMask()
{
	bool ret = false;
	if (hasBlackImage() && hasWhiteImage())
	{
		// Construct occlusion mask
		cv::Mat tmp(black_.rows, black_.cols, CV_16S);

		cv::subtract(white_, black_, tmp, cv::noArray(), CV_16S);
		tmp = cv::abs(tmp);

		occ_mask_ = cv::Mat(tmp.rows, tmp.cols, CV_8U, cv::Scalar(0));

		int 	threshold(3),
				threshold_white(300),
				filter_size(3);

		for (int row = 0; row < tmp.rows; row++)
		{
			for (int col = filter_size; col < tmp.cols-filter_size; col++)
			{
				int score = 0;
	//				occ_bin.at<uchar>(row, col) = cv::saturate_cast<uchar>(occ_16s.at<short>(row, col));
				if (tmp.at<short>(row, col) > threshold || white_.at<uchar>(row, col) > threshold_white || black_.at<uchar>(row, col) > threshold_white )
				{
					for (int i = 1; i < filter_size; i++)
					{
						if (occ_mask_.at<uchar>(row, col-i) == 255)
						{
							score++;
						}
						if (tmp.at<short>(row, col+i) > threshold || white_.at<uchar>(row, col+i) > threshold_white || black_.at<uchar>(row, col+i) > threshold_white)
						{
							score++;
						}
					}
					if (score >= filter_size-1)
						occ_mask_.at<uchar>(row, col) = 255;
				}
				else
				{
					for (int i = 1; i < filter_size; i++)
					{
						if (occ_mask_.at<uchar>(row, col-i) == 0)
						{
							score++;
						}
						if (!(tmp.at<short>(row, col+i) > threshold || white_.at<uchar>(row, col+i) > threshold_white || black_.at<uchar>(row, col+i) > threshold_white))
						{
							score++;
						}
					}
					if (score >= filter_size-1)
						occ_mask_.at<uchar>(row, col) = 0;
				}
			}
		}

		cv::medianBlur(occ_mask_, occ_mask_, filter_size+4);

	//		cv::adaptiveThreshold(occ_bin, occ_bin, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, 11, 2);
	//
	//		int erosion_size = 2;
	//		cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
	//		                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
	//		                                       cv::Point( erosion_size, erosion_size ) );
	//		cv::erode(occ_bin, occ_bin, element);

		int dilate_size = 5;

		cv::dilate(occ_mask_, occ_mask_, cv::getStructuringElement( cv::MORPH_RECT,
	            cv::Size( 2*dilate_size + 1, 2*dilate_size+1 ),
	            cv::Point( dilate_size, dilate_size ) ));
	}
	return ret;
}

//bool OcclusionMask::createOcclusionMask()
//{
//	static int threshold = 50;
//	bool ret;
//	if (hasBlackImage() && hasWhiteImage())
//	{
//		cv::Mat tmp = cv::Mat(black_.rows,black_.cols,CV_8U);
//		for (int x = 0; x < black_.rows; x++)
//		{
//			bool left_limit_found = false;
//			bool right_limit_found = false;
//			int left_limit = 0;
//			int right_limit = black_.cols;
//			for (int y = 0; y < black_.cols && !left_limit_found; y++)
//			{
//				if (white_.at<uchar>(x,y) > black_.at<uchar>(x,y) + threshold)
//				{
//					left_limit_found = true;
//					left_limit = y;
//				}
//				tmp.at<uchar>(x,y) = 0;
//			}
//			for (int y = black_.cols; y > left_limit && !right_limit_found; y--)
//			{
//				if (white_.at<uchar>(x,y) > black_.at<uchar>(x,y) + threshold)
//				{
//					right_limit_found = true;
//					right_limit = y;
//				}
//				tmp.at<uchar>(x,y) = 0;
//			}
//
//			if (left_limit_found && right_limit_found)
//			{
//				for(int y = left_limit; y <= right_limit; y++)
//				{
//					tmp.at<uchar>(x,y) = 1;
//				}
//			}
//			else
//			{
//				for(int y = 0; y <= black_.cols; y++)
//				{
//					tmp.at<uchar>(x,y) = 0;
//				}
//			}
//		}
//		tmp.copyTo(occ_mask_);
//		ret = true;
//	}
//	else
//	{
//		ret = false;
//	}
//	return ret;
//}

//} /* namespace perception */
