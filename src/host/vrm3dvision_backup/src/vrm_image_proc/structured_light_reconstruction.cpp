/********************************************************************************************************************
 *
 * \file                structured_light_reconstruction.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-12-02
 * \version             0.1
 * \brief
 *
*********************************************************************************************************************/
#include "structured_light_reconstruction.hpp"

#define WPOS1X 0
#define WPOS1Y 10
#define WPOS2X 1200
#define WPOS2Y 10
#define WPOS3X 1280
#define WPOS3Y 10

#define WPOS4X 0
#define WPOS4Y 500
#define WPOS5X 640
#define WPOS5Y 500

namespace vrm3dvision {

	const int StructuredLightReconstruction::index_lggc_10[] = {534,895,775,414,1015,654,774,111,533,172,52,413,292,173,293,956,233,352,232,593,1016,353,473,112,714,715,51,594,835,834,474,955,
			671,8,912,551,128,791,911,792,190,309,189,550,973,310,430,69,370,9,369,250,129,490,610,249,851,852,732,731,972,491,611,68,
			807,144,24,143,264,927,23,384,806,445,325,686,565,446,566,205,506,625,505,866,265,626,746,385,987,988,324,867,84,83,747,204,
			670,7,367,6,127,790,910,247,669,308,188,549,428,789,429,548,849,488,368,729,608,489,609,248,850,307,187,730,971,970,66,67,
			535,896,776,415,536,655,295,656,54,717,53,958,837,174,294,957,234,897,777,114,1017,354,1018,113,235,716,596,595,836,355,475,476,
			398,759,639,278,879,518,158,519,941,36,940,277,700,37,157,820,97,760,96,1001,880,217,337,1000,578,579,459,458,699,218,338,819,
			262,623,503,142,263,926,22,383,805,444,804,685,564,925,21,684,985,624,504,865,744,81,745,864,986,443,323,322,563,82,202,203,
			399,280,640,279,400,39,159,520,942,581,461,822,701,38,702,821,98,761,641,1002,881,762,882,521,99,580,460,1003,220,219,339,340,
			1013,894,230,893,1014,653,773,110,532,171,531,412,291,652,772,411,712,351,231,592,471,832,472,591,713,170,50,49,290,833,953,954,
			876,213,93,756,877,996,636,997,395,514,394,755,154,515,635,274,575,214,94,455,334,695,815,454,32,33,937,936,153,696,816,273,
			1012,349,229,892,469,108,228,109,1011,650,530,891,770,651,771,410,711,350,710,47,470,831,951,590,168,169,529,48,289,288,952,409,
			125,486,366,5,126,245,909,246,668,787,667,4,427,788,908,547,848,487,847,728,607,968,64,727,305,306,186,185,426,969,65,546,
			740,621,981,620,741,380,500,861,259,922,258,139,18,379,499,138,439,78,982,319,198,559,199,862,440,921,801,800,17,560,680,681,
			397,758,638,757,878,517,637,998,396,35,939,276,155,516,156,275,576,215,95,456,335,216,336,999,577,34,938,457,698,697,817,818,
			261,622,502,141,742,381,501,382,260,923,803,140,19,924,20,683,984,79,983,320,743,80,200,863,441,442,802,321,562,561,201,682,
			124,485,365,484,605,244,364,725,123,786,666,3,906,243,907,2,303,966,846,183,606,967,63,726,304,785,665,184,425,424,544,545,
			329,690,570,689,810,449,569,930,328,991,871,208,87,448,88,207,28,147,27,388,811,148,268,931,509,510,870,389,630,629,269,750,
			672,553,913,552,673,312,432,793,191,854,734,71,974,311,431,70,371,10,914,251,130,11,131,794,372,853,733,252,493,492,612,613,
			808,145,25,688,809,928,568,929,327,990,326,687,86,447,567,206,507,146,26,387,266,627,267,386,508,989,869,868,85,628,748,749,
			945,826,162,825,946,585,705,42,464,103,463,344,223,584,704,343,644,283,163,524,403,284,404,43,645,102,1006,525,766,765,885,886,
			56,417,297,416,537,176,296,657,55,718,598,959,838,175,839,478,779,898,778,115,538,899,1019,658,236,237,597,116,357,356,1020,477,
			193,554,434,73,674,313,433,314,192,855,735,72,975,856,976,615,916,555,915,796,675,12,132,795,373,374,254,253,494,13,133,614,
			57,418,298,961,58,177,841,178,600,719,599,960,359,720,840,479,780,419,299,660,539,900,540,659,781,238,118,117,358,901,1021,1022,
			944,281,161,824,401,40,160,41,943,582,462,823,222,583,703,342,643,282,642,523,402,763,883,522,100,101,1005,1004,221,764,884,341,
			330,691,571,210,331,450,90,451,873,992,872,209,632,993,89,752,29,692,572,933,812,149,813,932,30,511,391,390,631,150,270,751,
			875,212,92,211,332,995,91,452,874,513,393,754,633,994,634,753,574,693,573,934,333,694,814,453,31,512,392,935,152,151,271,272,
			467,348,708,347,468,107,227,588,1010,649,1009,890,769,106,226,889,166,829,709,46,949,830,950,589,167,648,528,527,768,287,407,408,
			466,827,707,346,947,586,706,587,465,104,1008,345,224,105,225,888,165,828,164,45,948,285,405,44,646,647,1007,526,767,286,406,887,
			739,76,980,619,196,859,979,860,738,377,257,618,497,378,498,137,438,77,437,318,197,558,678,317,919,920,256,799,16,15,679,136,
			194,75,435,74,195,858,978,315,737,376,736,617,496,857,977,616,917,556,436,797,676,557,677,316,918,375,255,798,495,14,134,135,
			602,963,843,962,59,722,842,179,601,240,120,481,360,721,361,480,301,420,300,661,60,421,541,180,782,239,119,662,903,902,542,1023,
			603,964,844,483,604,723,363,724,122,241,121,482,905,242,362,1,302,965,845,182,61,422,62,181,783,784,664,663,904,423,543,0};

	const int StructuredLightReconstruction::index_lggc_9[] = {391,310,50,481,220,139,221,480,390,197,49,196,219,26,48,367,504,311,163,482,505,140,334,141,277,424,276,83,106,425,447,254,
			8,181,179,180,9,10,350,351,407,408,66,67,236,409,237,238,293,294,292,465,122,123,463,464,406,295,65,466,235,124,64,125,
			136,309,307,308,137,138,478,479,23,24,194,195,364,25,365,366,421,422,420,81,250,251,79,80,22,423,193,82,363,252,192,253,
			7,438,178,97,348,267,349,96,6,325,177,324,347,154,176,495,120,439,291,98,121,268,462,269,405,40,404,211,234,41,63,382,
			392,53,51,52,393,394,222,223,279,280,450,451,108,281,109,110,165,166,164,337,506,507,335,336,278,167,449,338,107,508,448,509,
			263,182,434,353,92,11,93,352,262,69,433,68,91,410,432,239,376,183,35,354,377,12,206,13,149,296,148,467,490,297,319,126,
			135,54,306,225,476,395,477,224,134,453,305,452,475,282,304,111,248,55,419,226,249,396,78,397,21,168,20,339,362,169,191,510,
			264,437,435,436,265,266,94,95,151,152,322,323,492,153,493,494,37,38,36,209,378,379,207,208,150,39,321,210,491,380,320,381,
			502,199,161,370,331,28,160,29,389,198,388,369,218,27,47,368,503,312,162,483,332,313,333,142,104,85,275,84,105,426,446,255,
			373,200,372,371,202,201,31,30,88,87,259,258,89,428,430,429,486,485,145,484,315,314,144,143,487,86,146,257,316,427,317,256,
			501,328,500,499,330,329,159,158,216,215,387,386,217,44,46,45,102,101,273,100,443,442,272,271,103,214,274,385,444,43,445,384,
			118,327,289,498,459,156,288,157,5,326,4,497,346,155,175,496,119,440,290,99,460,441,461,270,232,213,403,212,233,42,62,383,
			245,72,244,243,74,73,415,414,472,471,131,130,473,300,302,301,358,357,17,356,187,186,16,15,359,470,18,129,188,299,189,128,
			374,71,33,242,203,412,32,413,261,70,260,241,90,411,431,240,375,184,34,355,204,185,205,14,488,469,147,468,489,298,318,127,
			246,455,417,114,75,284,416,285,133,454,132,113,474,283,303,112,247,56,418,227,76,57,77,398,360,341,19,340,361,170,190,511,
			117,456,116,115,458,457,287,286,344,343,3,2,345,172,174,173,230,229,401,228,59,58,400,399,231,342,402,1,60,171,61,0};

	const int StructuredLightReconstruction::index_lggc_8[] = {248,197,249,44,161,110,58,109,225,174,122,173,56,5,57,108,97,46,250,45,184,133,185,236,120,69,121,172,33,238,186,237,
			247,196,144,195,182,27,79,130,246,91,143,194,55,4,208,3,118,219,15,66,183,132,80,131,119,68,16,67,54,155,207,2,
			227,176,228,23,162,111,59,214,226,175,123,22,35,240,36,87,98,47,251,150,163,112,164,215,99,48,100,151,34,239,187,86,
			76,25,229,24,77,26,78,129,141,90,142,193,140,89,37,88,13,218,14,65,12,217,165,216,204,153,101,152,205,154,206,1,
			95,198,146,43,160,7,211,212,224,71,19,20,159,6,210,107,96,199,147,148,31,134,82,235,223,70,18,171,32,135,83,84,
			94,93,145,42,181,28,232,233,245,92,40,41,158,157,209,106,117,220,168,169,30,29,81,234,222,221,17,170,53,156,104,105,
			74,177,125,126,9,8,60,213,73,72,124,21,138,241,189,190,201,200,252,149,10,113,61,62,202,49,253,254,137,136,188,85,
			75,178,230,127,180,179,231,128,244,243,39,192,139,242,38,191,116,115,167,64,11,114,166,63,203,50,102,255,52,51,103,0};

	const int StructuredLightReconstruction::index_lggc_7[] = {37,36,122,79,6,49,91,48,102,17,59,16,5,4,90,47,38,81,123,80,69,68,26,111,101,100,58,15,70,113,27,112,
			120,35,121,78,23,66,108,109,119,34,76,77,88,3,89,46,55,98,12,13,24,67,25,110,56,99,57,14,87,2,44,45,
			104,19,105,62,7,50,92,93,103,18,60,61,72,115,73,30,39,82,124,125,8,51,9,94,40,83,41,126,71,114,28,29,
			21,20,106,63,22,65,107,64,118,33,75,32,117,116,74,31,54,97,11,96,53,52,10,95,85,84,42,127,86,1,43,0};

	const int StructuredLightReconstruction::index_brgc[] = {	682,683,681,680,685,684,686,687,677,676,678,679,674,675,673,672,693,692,694,695,690,691,689,688,698,699,697,696,701,700,702,703,
			661,660,662,663,658,659,657,656,666,667,665,664,669,668,670,671,650,651,649,648,653,652,654,655,645,644,646,647,642,643,641,640,
			725,724,726,727,722,723,721,720,730,731,729,728,733,732,734,735,714,715,713,712,717,716,718,719,709,708,710,711,706,707,705,704,
			746,747,745,744,749,748,750,751,741,740,742,743,738,739,737,736,757,756,758,759,754,755,753,752,762,763,761,760,765,764,766,767,
			597,596,598,599,594,595,593,592,602,603,601,600,605,604,606,607,586,587,585,584,589,588,590,591,581,580,582,583,578,579,577,576,
			618,619,617,616,621,620,622,623,613,612,614,615,610,611,609,608,629,628,630,631,626,627,625,624,634,635,633,632,637,636,638,639,
			554,555,553,552,557,556,558,559,549,548,550,551,546,547,545,544,565,564,566,567,562,563,561,560,570,571,569,568,573,572,574,575,
			533,532,534,535,530,531,529,528,538,539,537,536,541,540,542,543,522,523,521,520,525,524,526,527,517,516,518,519,514,515,513,512,
			853,852,854,855,850,851,849,848,858,859,857,856,861,860,862,863,842,843,841,840,845,844,846,847,837,836,838,839,834,835,833,832,
			874,875,873,872,877,876,878,879,869,868,870,871,866,867,865,864,885,884,886,887,882,883,881,880,890,891,889,888,893,892,894,895,
			810,811,809,808,813,812,814,815,805,804,806,807,802,803,801,800,821,820,822,823,818,819,817,816,826,827,825,824,829,828,830,831,
			789,788,790,791,786,787,785,784,794,795,793,792,797,796,798,799,778,779,777,776,781,780,782,783,773,772,774,775,770,771,769,768,
			938,939,937,936,941,940,942,943,933,932,934,935,930,931,929,928,949,948,950,951,946,947,945,944,954,955,953,952,957,956,958,959,
			917,916,918,919,914,915,913,912,922,923,921,920,925,924,926,927,906,907,905,904,909,908,910,911,901,900,902,903,898,899,897,896,
			981,980,982,983,978,979,977,976,986,987,985,984,989,988,990,991,970,971,969,968,973,972,974,975,965,964,966,967,962,963,961,960,
			1002,1003,1001,1000,1005,1004,1006,1007,997,996,998,999,994,995,993,992,1013,1012,1014,1015,1010,1011,1009,1008,1018,1019,1017,1016,1021,1020,1022,1023,
			341,340,342,343,338,339,337,336,346,347,345,344,349,348,350,351,330,331,329,328,333,332,334,335,325,324,326,327,322,323,321,320,
			362,363,361,360,365,364,366,367,357,356,358,359,354,355,353,352,373,372,374,375,370,371,369,368,378,379,377,376,381,380,382,383,
			298,299,297,296,301,300,302,303,293,292,294,295,290,291,289,288,309,308,310,311,306,307,305,304,314,315,313,312,317,316,318,319,
			277,276,278,279,274,275,273,272,282,283,281,280,285,284,286,287,266,267,265,264,269,268,270,271,261,260,262,263,258,259,257,256,
			426,427,425,424,429,428,430,431,421,420,422,423,418,419,417,416,437,436,438,439,434,435,433,432,442,443,441,440,445,444,446,447,
			405,404,406,407,402,403,401,400,410,411,409,408,413,412,414,415,394,395,393,392,397,396,398,399,389,388,390,391,386,387,385,384,
			469,468,470,471,466,467,465,464,474,475,473,472,477,476,478,479,458,459,457,456,461,460,462,463,453,452,454,455,450,451,449,448,
			490,491,489,488,493,492,494,495,485,484,486,487,482,483,481,480,501,500,502,503,498,499,497,496,506,507,505,504,509,508,510,511,
			170,171,169,168,173,172,174,175,165,164,166,167,162,163,161,160,181,180,182,183,178,179,177,176,186,187,185,184,189,188,190,191,
			149,148,150,151,146,147,145,144,154,155,153,152,157,156,158,159,138,139,137,136,141,140,142,143,133,132,134,135,130,131,129,128,
			213,212,214,215,210,211,209,208,218,219,217,216,221,220,222,223,202,203,201,200,205,204,206,207,197,196,198,199,194,195,193,192,
			234,235,233,232,237,236,238,239,229,228,230,231,226,227,225,224,245,244,246,247,242,243,241,240,250,251,249,248,253,252,254,255,
			85,84,86,87,82,83,81,80,90,91,89,88,93,92,94,95,74,75,73,72,77,76,78,79,69,68,70,71,66,67,65,64,
			106,107,105,104,109,108,110,111,101,100,102,103,98,99,97,96,117,116,118,119,114,115,113,112,122,123,121,120,125,124,126,127,
			42,43,41,40,45,44,46,47,37,36,38,39,34,35,33,32,53,52,54,55,50,51,49,48,58,59,57,56,61,60,62,63,
			21,20,22,23,18,19,17,16,26,27,25,24,29,28,30,31,10,11,9,8,13,12,14,15,5,4,6,7,2,3,1,0};

	// Returns -1 for negative, 0 for 0 and +1 for positive input
	template <typename T> inline int sign(T val) {
	    return (T(0) < val) - (val < T(0));
	}

	StructuredLightReconstruction::StructuredLightReconstruction() :
			occ_diff_threshold_(4),
			occ_score_threshold_(5),
			current_sequence_id_(-1),
			next_image_id_(0),
			num_layers_(9),
			num_exposures_(1),
			visualization_(false),
			save_next_sequence_(false),
			last_cloud_saved_(false),
			pc_color_option_(PC_COLOR_NONE),
			using_large_gap_gc_(false),
			use_ambient_occlusion_mask_(false),
			remove_outliers_(false),
			remove_dominant_plane_(false),
			mean_k_(50),
			std_dev_thresh_(5),
			certainty_type_(0)
	{
		for(int i = CamIndex::LEFT; i <= CamIndex::RIGHT; i++)
		{
			cam_info_[i].active = false;
		}

		existing_edges_.resize(961);
		certainty_points_.resize(961);

		save_certainty_values = false;
		if(save_certainty_values)
		{
			data_dump_.open("/home/kent/data_dump.txt", std::ios::out);
			data_dump_certainty_.open("/home/kent/data_dump_certainty.txt", std::ios::out);
			data_dump_certainty_.setf(std::ios_base::fixed);
			data_dump_certainty_ << std::setprecision(8);
		}
	}

	void StructuredLightReconstruction::initCamInfo(const std::string& calibration_url)
	{
		cv::FileStorage fs;
		fs.open(calibration_url.c_str(), cv::FileStorage::READ);

		if (fs.isOpened())
		{
			cv::Size image_size;
			fs["imageWidth"] >> image_size.width;
			fs["imageHeight"] >> image_size.height;

			// Initialize camera matrixes
			for(int i = CamIndex::LEFT; i <= CamIndex::RIGHT; i++)
			{
				cam_info_[i].size.width = image_size.width;//cam_info.width;
				cam_info_[i].size.height = image_size.height;//cam_info.height;
				cam_info_[i].enc_img[0] = cv::Mat(cam_info_[i].size,CV_16U, cv::Scalar(0));
				cam_info_[i].enc_img[1] = cv::Mat(cam_info_[i].size,CV_16U, cv::Scalar(0));
				cam_info_[i].neg_buffer_img = cv::Mat(cam_info_[i].size,CV_16S);
				cam_info_[i].pos_buffer_img = cv::Mat(cam_info_[i].size,CV_16S);
				cam_info_[i].occlusion_mask[0] = cv::Mat(cam_info_[i].size,CV_8U, cv::Scalar(0));
				cam_info_[i].occlusion_mask[1] = cv::Mat(cam_info_[i].size,CV_8U, cv::Scalar(0));
				cam_info_[i].min = cv::Mat(cam_info_[i].size, CV_8U, cv::Scalar(255));
				cam_info_[i].max = cv::Mat(cam_info_[i].size, CV_8U, cv::Scalar(0));
				cam_info_[i].T[0] = cv::Mat::eye(4,4, CV_64F);
				cam_info_[i].T[1] = cv::Mat::eye(4,4, CV_64F);
			}

			fs["K_left"] >> cam_info_[CamIndex::LEFT].K;
			fs["K_center"] >> cam_info_[CamIndex::CENTER].K;
			fs["K_right"] >> cam_info_[CamIndex::RIGHT].K;

			fs["D_left"] >> cam_info_[CamIndex::LEFT].D;
			fs["D_center"] >> cam_info_[CamIndex::CENTER].D;
			fs["D_right"] >> cam_info_[CamIndex::RIGHT].D;

			fs["T_lr_lc"] >> cam_info_[CamIndex::LEFT].T[0];
			fs["T_lr_lc"] >> cam_info_[CamIndex::CENTER].T[0];
			fs["T_lr_cr"] >> cam_info_[CamIndex::CENTER].T[1];
			fs["T_lr_cr"] >> cam_info_[CamIndex::RIGHT].T[1];

			fs["R_lc"] >> cam_info_[CamIndex::LEFT].R[0];
			fs["R_lr"] >> cam_info_[CamIndex::LEFT].R[1];
			fs["R_cl"] >> cam_info_[CamIndex::CENTER].R[0];
			fs["R_cr"] >> cam_info_[CamIndex::CENTER].R[1];
			fs["R_rl"] >> cam_info_[CamIndex::RIGHT].R[0];
			fs["R_rc"] >> cam_info_[CamIndex::RIGHT].R[1];

			fs["P_lc"] >> cam_info_[CamIndex::LEFT].P[0];
			fs["P_lr"] >> cam_info_[CamIndex::LEFT].P[1];
			fs["P_cl"] >> cam_info_[CamIndex::CENTER].P[0];
			fs["P_cr"] >> cam_info_[CamIndex::CENTER].P[1];
			fs["P_rl"] >> cam_info_[CamIndex::RIGHT].P[0];
			fs["P_rc"] >> cam_info_[CamIndex::RIGHT].P[1];

			for(int cam = CamIndex::LEFT; cam <= CamIndex::RIGHT; cam++)	// Initialize rectification map for all pairs (L-R, L-C, C-R)
			{
				for(int i = 0; i < 2; i++)
				{
					cv::initUndistortRectifyMap(cam_info_[cam].K, cam_info_[cam].D, cam_info_[cam].R[i],
							cam_info_[cam].P[i], cam_info_[cam].size, CV_16SC2,
							cam_info_[cam].rect_map_x[i], cam_info_[cam].rect_map_y[i]);
				}
			}

			// Initialize maps between stereo sets
			cv::Mat pixel_location_src(cam_info_[0].size, CV_32FC2);
			for (int i = 0; i < pixel_location_src.size().height; i++) {
				for (int j = 0; j < pixel_location_src.size().width; j++) {
					pixel_location_src.at<cv::Point2f>(i,j) = cv::Point2f(j,i);
				}
			}

			// Construct map from Left-Center to Left
			cv::Mat map_lc_l(cam_info_[0].size, CV_32FC2);
			cam_info_[CamIndex::LEFT].rect_map_to_anchor = cv::Mat(cam_info_[0].size, CV_32FC2);
			for (int i = 0; i < pixel_location_src.size().height; i++)
			{
				cv::Mat src(pixel_location_src.row(i));
				cv::Mat dst(map_lc_l.row(i));
				cv::undistortPoints(src, dst, cam_info_[CamIndex::LEFT].K, cam_info_[CamIndex::LEFT].D, cam_info_[CamIndex::LEFT].R[1], cam_info_[CamIndex::LEFT].P[1]);
			}
			cv::remap(map_lc_l, cam_info_[CamIndex::LEFT].rect_map_to_anchor,  cam_info_[CamIndex::LEFT].rect_map_x[0], cam_info_[CamIndex::LEFT].rect_map_y[0], CV_INTER_LINEAR);

			// Construct map from Right-Center to Right
			cv::Mat map_rc_r(cam_info_[0].size, CV_32FC2);
			cam_info_[CamIndex::RIGHT].rect_map_to_anchor = cv::Mat(cam_info_[0].size, CV_32FC2);
			for (int i = 0; i < pixel_location_src.size().height; i++)
			{
				cv::Mat src(pixel_location_src.row(i));
				cv::Mat dst(map_rc_r.row(i));
				cv::undistortPoints(src, dst, cam_info_[CamIndex::RIGHT].K, cam_info_[CamIndex::RIGHT].D, cam_info_[CamIndex::RIGHT].R[0], cam_info_[CamIndex::RIGHT].P[0]);
			}
			cv::remap(map_rc_r, cam_info_[CamIndex::RIGHT].rect_map_to_anchor,  cam_info_[CamIndex::RIGHT].rect_map_x[1], cam_info_[CamIndex::LEFT].rect_map_y[1], CV_INTER_LINEAR);


			cv::FileStorage fs_out;
			fs.open("/home/jeppe/left_calib.xml", cv::FileStorage::WRITE);
			if (fs.isOpened())
			{
			    fs << "image_size" << image_size;
			    cv::Mat tmp_cam_matrix = cam_info_[0].P[1](cv::Rect(0,0,3,3));
			    tmp_cam_matrix.at<double>(2,2) = 1;
			    fs << "camera_matrix" << tmp_cam_matrix;
			    cv::Mat dummy_dist(1,5,CV_32F, cv::Scalar(0));
			    fs << "distortion_coefficients" << dummy_dist;
			}


		}
		else
		{
			ROS_ERROR_STREAM("Unable to load camera calibration in OpenCV format - path: " << calibration_url);
		}
	}

	void StructuredLightReconstruction::initialize(const std::string& calibration_path, const std::vector<std::string>& calib_names, const std::string& camera_frame, bool visualization, const std::string& pc_color_option, bool remove_outliers, bool remove_dominant_plane, double std_dev_thresh, int mean_k, int certainty_type)
	{
		calibration_path_ = calibration_path;
		calib_file_names_ = calib_names;

		std::stringstream path;
		path << calibration_path_ << calib_file_names_[3] << ".yaml";
		initCamInfo(path.str());

		camera_frame_ = "/vrm_stereo_camera";

		if (boost::iequals(pc_color_option, "exposure"))
		{
			pc_color_option_ = PC_COLOR_EXPO;
		}
		else if (boost::iequals(pc_color_option, "camera"))
		{
			pc_color_option_ = PC_COLOR_CAM_SET;
		}
		else if (boost::iequals(pc_color_option, "certainty"))
		{
			pc_color_option_ = PC_COLOR_CERTAINTY;
		}
		else
		{
			pc_color_option_ = PC_COLOR_NONE;
		}

		visualization_ = visualization;
		if (visualization_)
			initVisualization();

		remove_outliers_ = remove_outliers;
		remove_dominant_plane_ = remove_dominant_plane;
		mean_k_ = mean_k;
		certainty_type_ = certainty_type;
		std_dev_thresh_ = (float)std_dev_thresh;
	}



	bool StructuredLightReconstruction::saveNextSequence(const std::string& folder_name)
	{
		std::string tmp_path;
		bool ret = false;
		if (save_next_sequence_ == false)
		{
			if (folder_name.at(0) != '/')
			{
				tmp_path = ros::package::getPath("vrm3dvision") + std::string("/") + folder_name;
			}
			else
			{
				tmp_path = folder_name;
			}
			if (!boost::filesystem::exists(tmp_path))
			{
				if (tmp_path.at(tmp_path.size()-1) != '/')
				{
					tmp_path.push_back('/');
				}
				if (boost::filesystem::create_directories(tmp_path))
				{
					save_next_sequence_ = true;
					save_path_ = tmp_path;
					std::stringstream header_path;
					header_path << save_path_ << "header.txt";
					image_headers_.open(header_path.str().c_str(), std::ios::out);

					// Copy calibration files to folder
					std::stringstream calibration_file_source;
					if (calibration_path_.empty())
					{
						calibration_file_source << getenv("HOME") << "/.ros/camera_info/";
					}
					else
					{
						calibration_file_source << calibration_path_;
					}

					for(int i = 0; i < calib_file_names_.size(); i++)
					{
						std::stringstream left_calibration_file_source;
						left_calibration_file_source << calibration_file_source.str() << calib_file_names_.at(i) << ".yaml";
						std::stringstream left_calibration_file_dist;
						left_calibration_file_dist << save_path_ << calib_file_names_.at(i) << ".yaml";
						boost::filesystem3::copy(left_calibration_file_source.str().c_str(),left_calibration_file_dist.str().c_str());
					}

					ROS_INFO_STREAM("The next sequence of images will be saved to: " << tmp_path << "<camera>_<imageid>.jpg");
					ret = true;
				}
				else
				{
					ROS_ERROR_STREAM("Failed to create folder: " << tmp_path);
				}
			}
			else
			{
				ROS_ERROR_STREAM("Folder already exists.. Images will not be saved..! - try new path to unexisting folder");
			}
		}
		else
		{
			ROS_ERROR_STREAM("A image sequence is currently being saved - have patience..!");
		}
		return ret;
	}



	void StructuredLightReconstruction::saveImages(const vrm_protocol::image_group& ig)
	{
		if (save_next_sequence_)
		{
			std::stringstream ss_header;
			ss_header << "header_" << ig.header.image_id() <<
					"_" << ig.header.exposure_id() << ".txt";
			image_headers_ << ss_header.str() << std::endl;
			std::string header_path = save_path_ + ss_header.str();
			std::ofstream header(header_path, std::ios::out | std::ios::binary);
			ig.header.SerializeToOstream(&header);
			header.close();

			if(ig.header.has_left_img())
			{
				std::stringstream ss_left;
				ss_left << "left_" << ig.header.image_id() <<
						"_" << ig.header.exposure_id() << ".jpg";
				image_headers_ << ss_left.str() << std::endl;
				std::string left_path = save_path_ + ss_left.str();
				cv::imwrite(left_path.c_str(), ig.left_image);
			}
			if(ig.header.has_right_img())
			{
				std::stringstream ss_right;
				ss_right << "right_" << ig.header.image_id() <<
						"_" << ig.header.exposure_id() << ".jpg";
				image_headers_ << ss_right.str() << std::endl;
				std::string right_path = save_path_ + ss_right.str();
				cv::imwrite(right_path.c_str(), ig.right_image);
			}
			if(ig.header.has_color_img())
			{
				std::stringstream ss_color;
				ss_color << "color_" << ig.header.image_id() <<
						"_" << ig.header.exposure_id() << ".jpg";
				image_headers_ << ss_color.str() << std::endl;
				std::string color_path = save_path_ + ss_color.str();
				cv::imwrite(color_path.c_str(), ig.color_image);
			}
			if(last_cloud_saved_)
			{
				last_cloud_saved_ = false;
				save_next_sequence_ = false;
				image_headers_.close();
			}
		}
	}



	void StructuredLightReconstruction::initVisualization()
	{
		cv::namedWindow("Input images",CV_WINDOW_NORMAL);
		cv::resizeWindow("Input images", 3*320, 240);
		cv::moveWindow("Input images",WPOS1X, WPOS1Y);

		cv::namedWindow("Occlusion masks", CV_WINDOW_NORMAL);
		cv::resizeWindow("Occlusion masks", 2*320, 3*240);
		cv::moveWindow("Occlusion masks",WPOS2X, WPOS2Y);

		cv::namedWindow("Encoded images",CV_WINDOW_NORMAL);
		cv::resizeWindow("Encoded images", 2*320, 3*240);
		cv::moveWindow("Encoded images",WPOS3X, WPOS3Y);

		cv::waitKey(1);
	}

	void StructuredLightReconstruction::resetCamInfo()
	{
		for(int i = CamIndex::LEFT; i <= CamIndex::RIGHT; i++)
		{
			// Reset camera masks
			cam_info_[i].occlusion_mask[0] = cv::Mat(cam_info_[i].size,CV_8U, cv::Scalar(0));
			cam_info_[i].occlusion_mask[1] = cv::Mat(cam_info_[i].size,CV_8U, cv::Scalar(0));
			cam_info_[i].min = cv::Mat(cam_info_[i].size, CV_8U, cv::Scalar(255));
			cam_info_[i].max = cv::Mat(cam_info_[i].size, CV_8U, cv::Scalar(0));
			//cam_info_[i].mask.reset();
			// Reset encoded images
			cam_info_[i].enc_img[0] = cv::Mat(cam_info_[i].size,CV_16U, cv::Scalar(0));
			cam_info_[i].enc_img[1] = cv::Mat(cam_info_[i].size,CV_16U, cv::Scalar(0));


			// Empty vectors
			cam_info_[i].diff_img_vector[0].clear();
			cam_info_[i].diff_img_vector[0].resize(num_layers_);
			cam_info_[i].diff_img_vector[1].clear();
			cam_info_[i].diff_img_vector[1].resize(num_layers_);
			cam_info_[i].pos_img_vector[0].clear();
			cam_info_[i].pos_img_vector[0].resize(num_layers_);
			cam_info_[i].pos_img_vector[1].clear();
			cam_info_[i].pos_img_vector[1].resize(num_layers_);
			cam_info_[i].neg_img_vector[0].clear();
			cam_info_[i].neg_img_vector[0].resize(num_layers_);
			cam_info_[i].neg_img_vector[1].clear();
			cam_info_[i].neg_img_vector[1].resize(num_layers_);
		}
	}

	void StructuredLightReconstruction::updateEncodedImage(CamInfo& cam_info, int bit_level)
	{

		/******* OLD APPROACH START *******/
		/*cv::Mat sub_img;
		cv::subtract(cam_info.pos_buffer_img, cam_info.neg_buffer_img, sub_img, cv::noArray(), CV_16S);
		//cv::subtract(cam_info.pos_buffer_img, cam_info.neg_buffer_img, sub_img, cv::noArray(), CV_16S);

		// Test of differentiating images for steeper edge responses - doesn't seem to help that much
//		cv::Mat kernel = cv::Mat::ones( 1, 3, CV_16S );
//		kernel.at<short>(0,2) = 0;
//
//		cv::Point anchor(-1,-1);
//
//		cv::Mat sub_img2;
//		cv::filter2D(sub_img, sub_img2, -1, kernel, anchor, 0, cv::BORDER_DEFAULT);

		cv::Mat pos, neg;
		cam_info.pos_buffer_img.convertTo(pos, CV_8U);
		cam_info.neg_buffer_img.convertTo(neg, CV_8U);

		if(using_large_gap_gc_ || (!using_large_gap_gc_ && (bit_level > num_layers_-3)))
		{
			cv::max(cam_info.max, pos, cam_info.max);
			cv::max(cam_info.max, neg, cam_info.max);
			cv::min(cam_info.min, pos, cam_info.min);
			cv::min(cam_info.min, neg, cam_info.min);
		}

		cv::GaussianBlur(sub_img, sub_img, cv::Size(3,3), 0.9); // default 0.9

		for(int j = 0; j < 2; j++)
		{
			cv::Mat out_img;
			cv::remap(sub_img, out_img, cam_info.rect_map_x[j], cam_info.rect_map_y[j], CV_INTER_LINEAR);

			cam_info.diff_img_vector[j][bit_level-1] = out_img;

			cv::Mat thr_img;
			cv::threshold(out_img,thr_img,0,pow(2,num_layers_-bit_level),CV_THRESH_BINARY);

			cv::add(thr_img, cam_info.enc_img[j], cam_info.enc_img[j], cv::noArray(), CV_16U);
		} */
		/******* OLD APPROACH END *******/

		if(using_large_gap_gc_ || (!using_large_gap_gc_ && (bit_level > num_layers_-3)))
		{
			cv::max(cam_info.max, cam_info.pos_buffer_img, cam_info.max);
			cv::max(cam_info.max, cam_info.neg_buffer_img, cam_info.max);
			cv::min(cam_info.min, cam_info.pos_buffer_img, cam_info.min);
			cv::min(cam_info.min, cam_info.neg_buffer_img, cam_info.min);
		}

		for(int j = 0; j < 2; j++)
		{
			cv::Mat pos, neg;
			cv::remap(cam_info.pos_buffer_img, pos, cam_info.rect_map_x[j], cam_info.rect_map_y[j], CV_INTER_LINEAR);
			cv::remap(cam_info.neg_buffer_img, neg, cam_info.rect_map_x[j], cam_info.rect_map_y[j], CV_INTER_LINEAR);

			cv::GaussianBlur(pos, pos, cv::Size(3,3), 0.9); // default 0.9
			cv::GaussianBlur(neg, neg, cv::Size(3,3), 0.9); // default 0.9

			cam_info.pos_img_vector[j][bit_level-1] = pos;
			cam_info.neg_img_vector[j][bit_level-1] = neg;

			cv::Mat sub_img;
			cv::subtract(pos, neg, sub_img, cv::noArray(), CV_16S);
			//cv::GaussianBlur(sub_img, sub_img, cv::Size(3,3), 0.9); // default 0.9
			cam_info.diff_img_vector[j][bit_level-1] = sub_img;

			cv::Mat thr_img;
			cv::threshold(sub_img,thr_img,0,pow(2,num_layers_-bit_level),CV_THRESH_BINARY);

			cv::add(thr_img, cam_info.enc_img[j], cam_info.enc_img[j], cv::noArray(), CV_16U);
		}

//		ROS_INFO_STREAM("Update single encoded image took: " << (ros::Time::now().toSec() - t)*1000 << " ms");

	}


	cv::Mat StructuredLightReconstruction::convertBayerToGray8U(const cv::Mat& src)
	{
		cv::Mat dst;
		cv::cvtColor(src,dst,CV_BayerGB2GRAY);
		return dst;
	}

	cv::Mat StructuredLightReconstruction::convertBayerToGray16S(const cv::Mat& src)
	{
		cv::Mat dst;
		cv::cvtColor(src,dst,CV_BayerGB2GRAY);
		dst.convertTo(dst,CV_16S);
		return dst;
	}



	int StructuredLightReconstruction::convertBinCodeToIndex(unsigned short value, bool large_gap_gc, int num_bits)
	{
		int index;

		if(large_gap_gc)
		{
			switch(num_bits)
			{
				case 7:
					index = index_lggc_7[value%128];
					break;
				case 8:
					index = index_lggc_8[value%256];
					break;
				case 9:
					index = index_lggc_9[value%512];
					break;
				case 10:
					index = index_lggc_10[value%1024];
					break;
				default:
					ROS_ERROR_STREAM("Unknown number of bit-levels: " << num_bits << " in convertBinCodeToIndex");
					break;
			}
		}
		else
		{
			switch(num_bits)
			{
				case 7:
					index = index_brgc[(8*value)%1024]/8;
					break;
				case 8:
					index = index_brgc[(4*value)%1024]/4;
					break;
				case 9:
					index = index_brgc[(2*value)%1024]/2;
					break;
				case 10:
					index = index_brgc[value%1024];
					break;
				default:
					ROS_ERROR_STREAM("Unknown number of bit-levels: " << num_bits << " in convertBinCodeToIndex()");
					break;
			}
		}
		return index;
	}


	void StructuredLightReconstruction::longrun2rgb(int val, int bit_level, int& r, int& g, int& b)
	{
		int id = convertBinCodeToIndex(val, true, bit_level);

	    val2rgb(id, bit_level, r, g, b);
		return;
	}

	void StructuredLightReconstruction::graycode2rgb(int val, int bit_level, int& r, int& g, int& b)
	{
		int id = convertBinCodeToIndex(val, false, bit_level);

	    val2rgb(id, bit_level, r, g, b);
		return;
	}



	bool StructuredLightReconstruction::checkValidEdge(const unsigned short left_value, const unsigned short right_value)
	{
		int index_left = convertBinCodeToIndex(left_value, using_large_gap_gc_, num_layers_);
		int index_right = convertBinCodeToIndex(right_value, using_large_gap_gc_, num_layers_);
		if((index_left - index_right) == 1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}


	cv::Mat StructuredLightReconstruction::convertEncodedImage2Color(const cv::Mat& in, const bool& long_run, const int& bit_level)
	{
		cv::Mat ret(in.rows, in.cols, CV_8UC3);

		for (int row = 0; row < in.rows; row++)
		{
			for (int col = 0; col < in.cols; col++)
			{
				int r, g, b;
				if (long_run)
					longrun2rgb(in.at<ushort>(row,col), bit_level, r, g, b);
				else
					graycode2rgb(in.at<ushort>(row,col), bit_level, r, g, b);

				ret.at<cv::Vec3b>(row,col)[0] = b;
				ret.at<cv::Vec3b>(row,col)[1] = g;
				ret.at<cv::Vec3b>(row,col)[2] = r;
			}
		}

		return ret;
	}


	void StructuredLightReconstruction::visualizeImages(const vrm_protocol::image_group& ig)
	{
		if (visualization_)
		{
			const static int pos_x[] = {0,		0,	320,	0,		320,	320};
			const static int pos_y[] = {240,	0,	240,	480,	0,		480};

			cv::Mat input_images = cv::Mat::zeros(240,3*320, CV_8UC1);
			if(ig.header.has_left_img())
			{
				cv::Mat temp;
				cv::resize(ig.left_image, temp, cv::Size(320, 240));
				temp.copyTo(input_images(cv::Rect(0,0,320,240)));
			}
			if(ig.header.has_right_img())
			{
				cv::Mat temp;
				cv::resize(ig.right_image, temp, cv::Size(320, 240));
				temp.copyTo(input_images(cv::Rect(2*320,0,320,240)));
			}
			if(ig.header.has_color_img())
			{
				cv::Mat temp;
				cv::resize(convertBayerToGray8U(ig.color_image), temp, cv::Size(320, 240));
				temp.copyTo(input_images(cv::Rect(320,0,320,240)));
			}

			if (lastImageInSequence(ig))
			{
				cv::Mat occlusion_masks = cv::Mat::zeros(3*240,2*320, CV_8UC1);
				cv::Mat temp;

				for (int cam = 0; cam < 3; cam++)
				{
					if (cam_info_[cam].active)
					{
						for (int i = 0; i < 2; i++)
						{
							cam_info_[cam].occlusion_mask[i].copyTo(temp);
							cv::resize(temp, temp, cv::Size(320, 240));
							cv::threshold(temp, temp, 0.5, 255, CV_THRESH_BINARY);
							temp.copyTo(occlusion_masks(cv::Rect(pos_x[cam*2+i],pos_y[cam*2+i],320,240)));
						}
					}
				}
				cv::imshow("Occlusion masks",occlusion_masks);
			}

			if (lastImageInSequence(ig))
			{
				cv::Mat encoded_images(3*240, 2*320, CV_8UC3);
				cv::Mat temp;

				for (int cam = 0; cam < 3; cam++)
				{
					for (int i = 0; i < 2; i++)
					{
						temp = convertEncodedImage2Color(cam_info_[cam].enc_img[i], using_large_gap_gc_, num_layers_);
						//cv::imwrite("/home/jeppe/encoded.png", temp);
						cv::Mat temp2 = cv::Mat::zeros(cam_info_[cam].size, CV_8UC3);
						temp.copyTo(temp2, cam_info_[cam].occlusion_mask[i]);
						cv::resize(temp2, temp2, cv::Size(320, 240));
						temp2.copyTo(encoded_images(cv::Rect(pos_x[cam*2+i],pos_y[cam*2+i],320,240)));
					}
				}
				cv::imshow("Encoded images", encoded_images);
			}
			cv::imshow("Input images",input_images);
			cv::waitKey(1);
		}
	}



	void StructuredLightReconstruction::updateCameraStatus(int cam_index, bool has_img)
	{
		cam_info_[cam_index].active = has_img;
	}



	void StructuredLightReconstruction::clearExistingEdgesAndCloud()
	{
		pc_.points.clear();
		pc_.header.stamp = ros::Time::now().toNSec();
		pc_.header.seq = current_sequence_id_;
		pc_.header.frame_id = camera_frame_;

		pc_trimmed_.points.clear();
		pc_trimmed_.header.stamp = ros::Time::now().toNSec();
		pc_trimmed_.header.seq = current_sequence_id_;
		pc_trimmed_.header.frame_id = camera_frame_;

		for(int row = 0; row < existing_edges_.size(); row++)
		{
			existing_edges_.at(row).clear();
			certainty_points_.at(row).clear();
		}
	}



	void StructuredLightReconstruction::initSequence(const vrm_protocol::image_group& ig)
	{
		if(ig.header.exposure_id() == 1)
		{
			time = ros::Time::now().toSec();
			updateCameraStatus(CamIndex::LEFT, ig.header.has_left_img());
			updateCameraStatus(CamIndex::RIGHT, ig.header.has_right_img());
			updateCameraStatus(CamIndex::CENTER, ig.header.has_color_img());

			cam_pairs_.clear();
			CamPair temp_pair;
			if(cam_info_[CamIndex::LEFT].active && cam_info_[CamIndex::RIGHT].active)
			{
				temp_pair.left_cam = CamIndex::LEFT;
				temp_pair.right_cam = CamIndex::RIGHT;
				cam_pairs_.push_back(temp_pair);
			}
			if(cam_info_[CamIndex::LEFT].active && cam_info_[CamIndex::CENTER].active)
			{
				temp_pair.left_cam = CamIndex::LEFT;
				temp_pair.right_cam = CamIndex::CENTER;
				cam_pairs_.push_back(temp_pair);
			}
			if(cam_info_[CamIndex::CENTER].active && cam_info_[CamIndex::RIGHT].active)
			{
				temp_pair.left_cam = CamIndex::CENTER;
				temp_pair.right_cam = CamIndex::RIGHT;
				cam_pairs_.push_back(temp_pair);
			}

			num_layers_ = (ig.header.num_images() /* - 2*/ ) / 2;

			int max_layers = (ig.header.is_full_view() ? 10 : 9);
			skip_rows_factor_ = pow(2,(max_layers - num_layers_));
			num_exposures_ = ig.header.num_exposures();

			clearExistingEdgesAndCloud();

			ROS_INFO_STREAM("New SLR sequence initialized - Layers: " << num_layers_);
		}

		current_sequence_id_ = ig.header.sequence_id();
		using_large_gap_gc_ = (ig.header.pattern_type() == vrm_protocol::PATTERN_LARGE_GAP_GC ? true : false );

		if(ig.header.image_id() == 60)
		{
			use_ambient_occlusion_mask_ = true;
			next_image_id_ = -1;
		}
		else
		{
			use_ambient_occlusion_mask_ = false;
			next_image_id_ = 0;
		}
		resetCamInfo();
	}



	void StructuredLightReconstruction::saveAmbientImage(const vrm_protocol::image_group& ig)
	{
		if(cam_info_[CamIndex::LEFT].active)
			ig.left_image.convertTo(cam_info_[CamIndex::LEFT].ambient_img, CV_8U);

		if(cam_info_[CamIndex::RIGHT].active)
			ig.right_image.convertTo(cam_info_[CamIndex::RIGHT].ambient_img, CV_8U);

		if(cam_info_[CamIndex::CENTER].active)
			cam_info_[CamIndex::CENTER].ambient_img = convertBayerToGray8U(ig.color_image);
	}


	void StructuredLightReconstruction::saveNegativeBufferImage(const vrm_protocol::image_group& ig)
	{
		if(cam_info_[CamIndex::LEFT].active)
			ig.left_image.convertTo(cam_info_[CamIndex::LEFT].neg_buffer_img, CV_8U);

		if(cam_info_[CamIndex::RIGHT].active)
			ig.right_image.convertTo(cam_info_[CamIndex::RIGHT].neg_buffer_img, CV_8U);

		if(cam_info_[CamIndex::CENTER].active)
			cam_info_[CamIndex::CENTER].neg_buffer_img = convertBayerToGray8U(ig.color_image);
	}



	void StructuredLightReconstruction::savePositiveBufferImageAndUpdateEncodedImage(const vrm_protocol::image_group& ig, int bit_level)
	{
		double t = ros::Time::now().toSec();
		#pragma omp parallel num_threads(3)
		{
			if (omp_get_thread_num() == 0)
			{
				if(cam_info_[CamIndex::LEFT].active)
				{
					ig.left_image.convertTo(cam_info_[CamIndex::LEFT].pos_buffer_img, CV_8U);
					updateEncodedImage(cam_info_[CamIndex::LEFT], bit_level);
				}
			}
			else if(omp_get_thread_num() == 1)
			{
				if(cam_info_[CamIndex::RIGHT].active)
				{
					ig.right_image.convertTo(cam_info_[CamIndex::RIGHT].pos_buffer_img, CV_8U);
					updateEncodedImage(cam_info_[CamIndex::RIGHT], bit_level);
				}
			}
			else
			{
				if(cam_info_[CamIndex::CENTER].active)
				{
					cam_info_[CamIndex::CENTER].pos_buffer_img = convertBayerToGray8U(ig.color_image);
					updateEncodedImage(cam_info_[CamIndex::CENTER], bit_level);
				}
			}
		}
		ROS_INFO_STREAM("Encoded images updated for level " << bit_level << " - It took: " << (ros::Time::now().toSec() - t)*1000 << " ms");
	}



	int StructuredLightReconstruction::addImages(const vrm_protocol::image_group& ig)
	{
		//const int bit_level_order[] = {0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10}; // Original
		const int bit_level_order[] = {1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10}; // GrayCode with long run length
		//const int bit_level_order[] = {7,7,1,1,2,2,3,3,4,4,5,5,6,6,8,8,9,9,10,10}; // Without black/white

		int ret = 0;
		if(firstImage(ig))
		{
			initSequence(ig);
		}

		if(validImage(ig))
		{
			if (ambientImage(ig))
			{
				saveAmbientImage(ig);
			}
			else if (bit_level_order[ig.header.image_id()] > 0)
			{
				if (evenImageIndex(ig)) // Even indexes - save image and wait for complementary image to do calculations
				{
					saveNegativeBufferImage(ig);
				}
				else // uneven indexes - use this and last image to update encoded image
				{
					savePositiveBufferImageAndUpdateEncodedImage(ig, bit_level_order[ig.header.image_id()]);
				}

				if (lastImageInSequence(ig))
				{
					bool last_exposure = (ig.header.exposure_id() == ig.header.num_exposures());
					createPointCloud(ig.header.exposure_id(), last_exposure);
					ret = ig.header.exposure_id();
					ROS_INFO_STREAM_COND(last_exposure, "*** Creating complete point cloud took: " << (ros::Time::now().toSec() - time)*1000 << " ms ***");
				}
			}
			saveImages(ig);
			visualizeImages(ig);
			next_image_id_++;
		}
		else // Unexpected image recieved TODO HANDLE ERROR
		{
			ROS_ERROR_STREAM("ERROR: Recieved unexpected/invalid image in SLR, image id received: " << ig.header.image_id() << " Expected: " << (next_image_id_));
		}
		return ret;
	}


	void StructuredLightReconstruction::insertCertaintyPoint(const CertaintyPointEntry& entry)
	{
		certainty_points_.at(entry.row_).insert(std::pair<int, CertaintyPoint>(entry.key_, entry.cp_));
	}



	void StructuredLightReconstruction::insertExistingEdge(ExistingEdgeEntry entry)
	{
		existing_edges_.at(entry.row_).insert(std::pair<int, bool>(entry.key_, entry.object_point_));
	}


	bool insertEdge(std::map<int, double>& edgeMap, int key, double value, bool overwrite)
	{
		std::pair<std::map<int, double>::iterator, bool> ret;
		ret = edgeMap.insert(std::pair<int, double>(key, value));
		if(ret.second == false)
		{
			if(overwrite)
			{
				edgeMap.erase (ret.first);
				edgeMap.insert(std::pair<int, double>(key, value));
			}
		}
		return true;
	}

	int roundToNearest(int in, int round)
	{
		int ret;
		if((in % round) <= (round / 2))
		{
			ret = in - (in % round);   //Round down

		}
		else
		{
			ret = in - (in % round) + round;   //Round down, then up again
		}
		if (ret < 0)
			ret = 0;
		return ret;
	}

	int roundDownToNearest(int in, int round)
	{
		int ret = in - (in % round);
		if (ret < 0)
		{
			ret = 0;
		}
		return ret;   //Round down
	}

	void StructuredLightReconstruction::findEdge(int row, const cv::Mat& enc_img, const std::vector<cv::Mat>& diff_vector,
			int pair_idx, bool overwrite_edge, std::map<int,double>& edge_map)
	{
		static int Fact = pow(2,num_layers_ + 2);
		std::map<int,bool>::iterator itExisting;

		const unsigned short* ptr = enc_img.ptr<unsigned short>(row);

		for(int col = 0; col < enc_img.cols; col++)
		{
			if(ptr[col] != ptr[col+1])
			{
				if(checkValidEdge(ptr[col], ptr[col+1]))
				{
					int edge_key = ptr[col] * Fact + ptr[col+1];
					int anchor_row;
					if (cam_pairs_[pair_idx].left_cam == CamIndex::LEFT && cam_pairs_[pair_idx].right_cam == CamIndex::RIGHT) // Left-Center or Right-Center - Check if point is reconstruced in Left-Right
					{
						anchor_row = row;
					}
					else if (cam_pairs_[pair_idx].left_cam == CamIndex::LEFT && cam_pairs_[pair_idx].right_cam == CamIndex::CENTER) // Left-Center
					{
						anchor_row = int(cam_info_[CamIndex::LEFT].rect_map_to_anchor.at<cv::Point2f>(row,col).y);
						anchor_row = roundDownToNearest(anchor_row, skip_rows_factor_);
					}
					else  // Right-Center
					{
						anchor_row = int(cam_info_[CamIndex::RIGHT].rect_map_to_anchor.at<cv::Point2f>(row,col).y);
						anchor_row = roundDownToNearest(anchor_row, skip_rows_factor_);
					}

					if(certainty_type_ == 0)
					{
						bool reconstruct = false;
						itExisting = existing_edges_.at(anchor_row).find(edge_key);
						if(itExisting == existing_edges_.at(anchor_row).end())
						{
							reconstruct = true;
						}
						itExisting = existing_edges_.at(anchor_row+skip_rows_factor_).find(edge_key);
						if(itExisting == existing_edges_.at(anchor_row+skip_rows_factor_).end() || reconstruct == true)
						{
							insertEdge(edge_map, edge_key, col, overwrite_edge);
						}
					}
					else
					{
						insertEdge(edge_map, edge_key, col, overwrite_edge);
					}
				}
			}
		}
	}



	bool StructuredLightReconstruction::reconstructEdge(int row, int left_cam, int right_cam, int left_idx, int right_idx,
			const std::map<int,double>::iterator& it1, const std::map<int,double>::iterator& it2,
			std::map<int, double>& edge_map_1, std::map<int, double>& edge_map_2,
			int pair_idx, int exposure_id,
			std::vector<ExistingEdgeEntry>& temp_existing_edges,
			std::vector<CertaintyPointEntry>& temp_certainty_points, pcl::PointXYZRGB& point)
	{
		bool ret = false;

		// Interpolate both edges to obtain subpixel accuracy
		double left, right;
		double left_certainty, right_certainty;

		std::vector<int> left_edge;
		std::vector<int> right_edge;

		//data_dump_ << row << ",";

		if (JEPP_Interpolate(left, row, it1->second, cam_info_[left_cam], left_idx, left_certainty, left_edge))
		{
			if (JEPP_Interpolate(right, row, it2->second, cam_info_[right_cam], right_idx, right_certainty, right_edge))
			{
				//data_dump_ << std::endl;

				// Triangulate 3D point
				cv::Mat img_point_1(2,1,CV_64F);
				cv::Mat img_point_2(2,1,CV_64F);
				img_point_1.at<double>(0,0) = it1->second + left;
				img_point_1.at<double>(1,0) = row;

				img_point_2.at<double>(0,0) = it2->second + right;
				img_point_2.at<double>(1,0) = row;

				if ((left_cam == CamIndex::CENTER && right_cam == CamIndex::RIGHT))
				{
					point = triangulationQ(img_point_2, cam_info_[right_cam].P[right_idx], img_point_1, cam_info_[left_cam].P[left_idx]);
				}
				else
				{
					point = triangulationQ(img_point_1, cam_info_[left_cam].P[left_idx], img_point_2, cam_info_[right_cam].P[right_idx]);
				}

//				 Transform points to common frame
				if (!(left_cam == CamIndex::LEFT && right_cam == CamIndex::RIGHT)) // Only transform points in relevant cam pairs
				{
					transformPoint(cam_info_[left_cam].T[left_idx], point);
				}

				if ((left_cam == CamIndex::LEFT && right_cam == CamIndex::CENTER)) // Left-Center
				{
					cv::Mat extra_trans = (cv::Mat_<double>(4,4) << 	    0.999985,  -0.00013366,   0.00544276,  -0.00283085,
																		 0.000131616,            1,  0.000376236, -0.000208208,
																		 -0.00544281, -0.000375505,     0.999985,   0.00101507,
																				   0,            0,            0,            1

													 );
					transformPoint(extra_trans, point);

//					cv::Mat extra_trans2 = (cv::Mat_<double>(4,4) << 	1.00001, -0.000748808,  0.000670064, -0.000637962,
//							 0.000749008,            1, -1.75516e-05, -4.98591e-05,
//							-0.000670403,  1.84403e-05,     0.999996,  1.91835e-05,
//							           0,            0,            0,            1
//													 );
//					transformPoint(extra_trans2, point);

				}
				else if ((left_cam == CamIndex::CENTER && right_cam == CamIndex::RIGHT))
				{
					cv::Mat extra_trans = (cv::Mat_<double>(4,4) << 	    0.999986,  0.000135047,  -0.00555809,   0.00298596,
																		 -0.00013653,            1, -0.000271215,  0.000155716,
																		  0.00555805,  0.000271996,     0.999986, -0.000860318,
																				   0,            0,            0,            1
 );
					transformPoint(extra_trans, point);

//					cv::Mat extra_trans2 = (cv::Mat_<double>(4,4) << 	   1,  0.000678826, -0.000881565,  0.000716475,
//							-0.000679014,            1, -0.000245403,  0.000164491,
//							 0.000881319,   0.00024602,            1,  0.000103178,
//							           0,            0,            0,            1
//					 );
//					transformPoint(extra_trans2, point);
				}

				double certainty = (left_certainty + right_certainty)/2.0;
				if(left_cam == CamIndex::CENTER || right_cam == CamIndex::CENTER)
				{
					certainty *= 0.9;
				}

				if(pc_color_option_ == PC_COLOR_CAM_SET)
				{
					setPointColorFromIdx(point, pair_idx);
				}
				else if(pc_color_option_ == PC_COLOR_EXPO)
				{
					setPointColorFromIdx(point, exposure_id-1);
				}
				else if(pc_color_option_ == PC_COLOR_CERTAINTY)
				{
					int r,g,b;
					valToJet(certainty, r, g ,b);
					point.r = r;
					point.g = g;
					point.b = b;
				}
				else
				{
					setPointColorFromIdx(point,0);
				}


				// Update reconstructed edge in list of existing edges
				int anchor_row;
				if ((left_cam == CamIndex::LEFT && right_cam == CamIndex::RIGHT)) // Left-Center or Right-Center - Check if point is reconstruced in Left-Right
				{
					insertExistingEdge(ExistingEdgeEntry(row, it1->first, false));
					if (certainty_type_ != 0)
					{
						insertCertaintyPoint(CertaintyPointEntry(row, it1->first, certainty, point, left_edge, right_edge, 0, exposure_id));
					}
				}
				else if ((left_cam == CamIndex::LEFT && right_cam == CamIndex::CENTER)) // Left-Center
				{
					anchor_row = int(cam_info_[CamIndex::LEFT].rect_map_to_anchor.at<cv::Point2f>(row,int(it1->second)).y + 0.49999);
					anchor_row = roundToNearest(anchor_row, skip_rows_factor_);
					temp_existing_edges.push_back(ExistingEdgeEntry(anchor_row, it1->first, false));
					if (certainty_type_ != 0)
					{
						temp_certainty_points.push_back(CertaintyPointEntry(anchor_row, it1->first, certainty, point, left_edge, right_edge, 1, exposure_id));
					}
				}
				else  // Right-Center
				{
					anchor_row = int(cam_info_[CamIndex::RIGHT].rect_map_to_anchor.at<cv::Point2f>(row,int(it2->second)).y + 0.49999);
					anchor_row = roundToNearest(anchor_row, skip_rows_factor_);
					temp_existing_edges.push_back(ExistingEdgeEntry(anchor_row, it2->first, false));
					if (certainty_type_ != 0)
					{
						temp_certainty_points.push_back(CertaintyPointEntry(anchor_row, it2->first, certainty, point, left_edge, right_edge, 2, exposure_id));
					}
				}

				// Delete reconstructed edges
				edge_map_2.erase(it2);

				ret = true;
			}
		}
		return ret;
	}

	void StructuredLightReconstruction::transformPoint(const cv::Mat& T, pcl::PointXYZRGB& point)
	{
		pcl::PointXYZRGB tmp;
		tmp.x = T.at<double>(0,0)*point.x + T.at<double>(0,1)*point.y + T.at<double>(0,2)*point.z + T.at<double>(0,3);
		tmp.y = T.at<double>(1,0)*point.x + T.at<double>(1,1)*point.y + T.at<double>(1,2)*point.z + T.at<double>(1,3);
		tmp.z = T.at<double>(2,0)*point.x + T.at<double>(2,1)*point.y + T.at<double>(2,2)*point.z + T.at<double>(2,3);
		point = tmp;
	}

	void StructuredLightReconstruction::savePointCloudToPCD(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
			const std::string& path, const std::string& filename, int exposure_id, bool save_without_color)
	{
        if (!cloud.points.empty())
        {
			std::stringstream ss;
			ss << path << filename << "_" << exposure_id << ".pcd";
			if (!save_without_color)
			{
				pcl::io::savePCDFileASCII(ss.str(), cloud);
			}
			else
			{
				pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
				cloud_xyz.resize(cloud.size());
				for (int i = 0; i < cloud.size(); i++)
				{
					cloud_xyz.points[i].x = cloud.points[i].x;
					cloud_xyz.points[i].y = cloud.points[i].y;
					cloud_xyz.points[i].z = cloud.points[i].z;
				}
				pcl::io::savePCDFileASCII(ss.str(), cloud_xyz);
			}
        }
        else
        {
        	ROS_ERROR("Failed to save point cloud - it was empty");
        }
	}

	void StructuredLightReconstruction::savePointClouds(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const pcl::PointCloud<pcl::PointXYZRGB>& cloud_trimmed, int exposure_id)
	{
		if (save_next_sequence_)
		{
			savePointCloudToPCD(cloud, save_path_, "point_cloud", exposure_id, false);
			savePointCloudToPCD(cloud, save_path_, "point_cloud_no_color", exposure_id, true);
			if(exposure_id == num_exposures_)
			{
				last_cloud_saved_ = true;
				savePointCloudToPCD(cloud_trimmed, save_path_, "point_cloud_trimmed", exposure_id, false);
				savePointCloudToPCD(cloud_trimmed, save_path_, "point_cloud_trimmed_no_color", exposure_id, true);
			}
		}
	}

	void StructuredLightReconstruction::setPointColorFromIdx(pcl::PointXYZRGB& point, int color_idx)
	{
		static int colors[][3] = {	{255,	255,	255}, 	// White
									{255,	0,		0},		// Red
									{0,		255,	0},		// Green
									{0,		0,		255}};	// Blue
		if (color_idx > 3)
			color_idx = 3;
		point.r = colors[color_idx][0];
		point.g = colors[color_idx][1];
		point.b = colors[color_idx][2];
	}



	void StructuredLightReconstruction::applyOcclusionMask(int exposure_id)
	{
		for(int cam_idx = 0; cam_idx < 3; cam_idx++)
		{
			if (cam_info_[cam_idx].active)
			{
				std::string prefix;

				if(save_next_sequence_)
				{
					if(cam_idx == CamIndex::LEFT)
					{
						prefix = "temp_left_";
					}
					else if( cam_idx == CamIndex::RIGHT)
					{
						prefix = "temp_right_";
					}
					else
					{
						prefix = "temp_color_";
					}
					cv::Mat enc_0 = convertEncodedImage2Color(cam_info_[cam_idx].enc_img[0], using_large_gap_gc_, num_layers_);
					cv::Mat enc_1 = convertEncodedImage2Color(cam_info_[cam_idx].enc_img[1], using_large_gap_gc_, num_layers_);
					cv::Mat occ_0 = cv::Mat::ones(cam_info_[cam_idx].size, CV_8U);
					cv::Mat occ_1 = cv::Mat::ones(cam_info_[cam_idx].size, CV_8U);
					cv::Mat occ_0_remapped, occ_1_remapped;
					cv::remap(occ_0, occ_0_remapped, cam_info_[cam_idx].rect_map_x[0], cam_info_[cam_idx].rect_map_y[0], CV_INTER_LINEAR);
					cv::remap(occ_1, occ_1_remapped, cam_info_[cam_idx].rect_map_x[1], cam_info_[cam_idx].rect_map_y[1], CV_INTER_LINEAR);

					cv::Mat temp_0 = cv::Mat::zeros(cam_info_[cam_idx].size, CV_16U);
					cv::Mat temp_1 = cv::Mat::zeros(cam_info_[cam_idx].size, CV_16U);
					enc_0.copyTo(temp_0, occ_0_remapped);
					enc_1.copyTo(temp_1, occ_1_remapped);

					std::stringstream path0, path1;
					path0 << save_path_ << prefix << "enc_occ_none_" << exposure_id << "_0.png";
					path1 << save_path_ << prefix << "enc_occ_none_" << exposure_id << "_1.png";
					cv::imwrite(path0.str(), temp_0);
					cv::imwrite(path1.str(), temp_1);
				}

				if(use_ambient_occlusion_mask_)
				{
					cv::Mat direct = cam_info_[cam_idx].max - cam_info_[cam_idx].min;
					cv::Mat indirect = cam_info_[cam_idx].min - cam_info_[cam_idx].ambient_img;
					cv::Mat temp_diff = direct - 0.5*indirect;
					cv::Mat temp;

					for (int i = 0; i < 2; i++)
					{
						cv::remap(temp_diff, temp, cam_info_[cam_idx].rect_map_x[i], cam_info_[cam_idx].rect_map_y[i], CV_INTER_LINEAR);
						cv::threshold(temp, cam_info_[cam_idx].occlusion_mask[i], 5, 255, CV_THRESH_BINARY);

						/*int erosion_size = 1;
						cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
																	   cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
														   cv::Point( erosion_size, erosion_size ) );
						cv::erode(cam_info_[cam_idx].occlusion_mask[i], cam_info_[cam_idx].occlusion_mask[i], element);
						cv::dilate(cam_info_[cam_idx].occlusion_mask[i], cam_info_[cam_idx].occlusion_mask[i], element);*/

						if(save_next_sequence_)
						{
							std::stringstream path_min, path_max, path_direct, path_indirect;
							path_min << save_path_ << prefix << "min_" << exposure_id << ".png";
							path_max << save_path_ << prefix << "max_" << exposure_id << ".png";
							path_direct << save_path_ << prefix << "direct_" << exposure_id << ".png";
							path_indirect << save_path_ << prefix << "indirect_" << exposure_id << ".png";
							cv::imwrite(path_min.str(), cam_info_[cam_idx].min);
							cv::imwrite(path_max.str(), cam_info_[cam_idx].max);
							cv::imwrite(path_direct.str(), direct);
							cv::imwrite(path_indirect.str(), indirect);

							cv::Mat enc = convertEncodedImage2Color(cam_info_[cam_idx].enc_img[i], using_large_gap_gc_, num_layers_);
							cv::Mat temp_enc = cv::Mat::zeros(cam_info_[cam_idx].size, CV_16U);
							enc.copyTo(temp_enc, cam_info_[cam_idx].occlusion_mask[i]);

							std::stringstream path;
							path << save_path_ << prefix << "enc_occ_" << exposure_id << "_" << i << ".png";
							cv::imwrite(path.str(), temp_enc);

							cv::Mat temp_direct;
							cv::remap(direct, temp_direct, cam_info_[cam_idx].rect_map_x[i], cam_info_[cam_idx].rect_map_y[i], CV_INTER_LINEAR);
							cv::threshold(temp_direct, temp_direct, 10, 255, CV_THRESH_BINARY);

							temp_enc = cv::Mat::zeros(cam_info_[cam_idx].size, CV_16U);
							enc.copyTo(temp_enc, temp_direct);
							std::stringstream path_simple;
							path_simple << save_path_ << prefix << "enc_occ_simple_" << exposure_id << "_" << i << ".png";
							cv::imwrite(path_simple.str(), temp_enc);
						}

						cv::Mat temp = cv::Mat::zeros(cam_info_[cam_idx].size, CV_16U);
						cam_info_[cam_idx].enc_img[i].copyTo(temp, cam_info_[cam_idx].occlusion_mask[i]);
						temp.copyTo(cam_info_[cam_idx].enc_img[i]);
					}
				}
				else
				{
					cv::Mat direct = cam_info_[cam_idx].max - cam_info_[cam_idx].min;
					cv::Mat temp;

					for (int i = 0; i < 2; i++)
					{
						cv::remap(direct, temp, cam_info_[cam_idx].rect_map_x[i], cam_info_[cam_idx].rect_map_y[i], CV_INTER_LINEAR);
						cv::threshold(temp, cam_info_[cam_idx].occlusion_mask[i], 10, 255, CV_THRESH_BINARY);

						cv::Mat temp = cv::Mat::zeros(cam_info_[cam_idx].size, CV_16U);
						cam_info_[cam_idx].enc_img[i].copyTo(temp, cam_info_[cam_idx].occlusion_mask[i]);
						temp.copyTo(cam_info_[cam_idx].enc_img[i]);
					}
				}
			}
		}
	}


	void computePlaneError(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		pcl::PointCloud<pcl::PointXYZRGB> new_cloud;

		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.01, 0.60);
		pass.filter (new_cloud);

		/*pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (0.03, 0.07);
		pass.filter (new_cloud);

		pass.setInputCloud (new_cloud.makeShared());
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (-0.02, 0.02);
		pass.filter (new_cloud);*/

//		pcl::PassThrough<pcl::PointXYZRGB> pass;
//		pass.setInputCloud (cloud);
//		pass.setFilterFieldName ("x");
//		pass.setFilterLimits (0.01, 0.09);
//		pass.filter (new_cloud);
//
//		pass.setInputCloud (new_cloud.makeShared());
//		pass.setFilterFieldName ("y");
//		pass.setFilterLimits (-0.04, 0.04);
//		pass.filter (new_cloud);

		if(!cloud->empty())
		{
			pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (new_cloud.makeShared()));

			// Create the RANSAC object
			pcl::RandomSampleConsensus<pcl::PointXYZRGB> sac (model, 0.005);
			bool result = sac.computeModel ();

			Eigen::VectorXf coeff;
			sac.getModelCoefficients (coeff);

			std::vector<int> inliers;
			sac.getInliers (inliers);
			std::cout << "Number of inliers: " << inliers.size() << std::endl;

			Eigen::VectorXf coeff_refined;
			model->optimizeModelCoefficients (inliers, coeff, coeff_refined);

			std::vector<double> distances;
			model->getDistancesToModel(coeff_refined, distances);

			double error_sum = 0;
			for(int i = 0; i < distances.size(); i++)
			{
				error_sum += distances[i]*distances[i];
			}

			std::cout << "Number of points  : " << distances.size() << std::endl;
			std::cout << "Standard deviation: " << (sqrt(error_sum/distances.size())) << std::endl;

			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			coefficients->values.resize(4);
			coefficients->values[0] = coeff_refined[0];
			coefficients->values[1] = coeff_refined[1];
			coefficients->values[2] = coeff_refined[2];
			coefficients->values[3] = coeff_refined[3];

			double a = -0.00592134 , b = 0.0366744, c = 0.99931, d = -0.545443;

			double new_total_error = 0;
			double old_total_error = 0;
			for(int i = 0; i < new_cloud.points.size(); i++)
			{
				new_total_error += fabs((coefficients->values[0]*new_cloud.points[i].x + coefficients->values[1]*new_cloud.points[i].y + coefficients->values[2]*new_cloud.points[i].z + coefficients->values[3])/
						(sqrt(coefficients->values[0]*coefficients->values[0] + coefficients->values[1]*coefficients->values[1] + coefficients->values[2]*coefficients->values[2])));
				old_total_error += fabs((a*new_cloud.points[i].x + b*new_cloud.points[i].y + c*new_cloud.points[i].z + d)/
						(sqrt(a*a + b*b + c*c)));
			}

			double new_avg_error = new_total_error/new_cloud.points.size();
			double old_avg_error = old_total_error/new_cloud.points.size();

			ROS_WARN_STREAM("Old coefficients: a = " << a << " b = " << b << " c = " << c << " d = " << d);
			ROS_WARN_STREAM("New coefficients: a = " << coefficients->values[0] << " b = " << coefficients->values[1] << " c = " << coefficients->values[2] << " d = " << coefficients->values[3]);
			ROS_WARN_STREAM("No. points: " << new_cloud.points.size() << " Old avg. error: " << old_avg_error << " New avg. error: " << new_avg_error);



			/*for(int i = 0; i < new_cloud.points.size(); i++)
			{
				double plane_z =(-coeff_refined[0]*new_cloud.points[i].x - coeff_refined[1]*new_cloud.points[i].y - coeff_refined[3])/coeff_refined[2];
				if(new_cloud.points[i].z > plane_z)
				{
					new_cloud.points[i].z = distances[i];
				}
				else
				{
					new_cloud.points[i].z = -distances[i];
				}
			}*/

			std::stringstream ss;
			ss << "/home/jeppe/patch_error.pcd";
			pcl::io::savePCDFileASCII(ss.str(), new_cloud);
		}
	}



	void StructuredLightReconstruction::valToJet(double v, int& r, int& g, int& b)
	{
		if(v < 0.0)
			v = 0.0;
		if(v > 1.0)
			v = 1.0;

		r = 0; g = 0; b = 0;

		double dv = 1;

		if(v <= dv/8.0)
		{
			b = 1024*v + 127;
		}
		else if( v > dv/8.0 && v <= 3.0*dv/8.0)
		{
			b = 255;
			g = 1024*v - 127;
		}
		else if( v > 3.0*dv/8.0 && v <= 5.0*dv/8.0)
		{
			r = 1024*v - 384;
			g = 255;
			b = -1024*v + 640;
		}
		else if( v > 5.0*dv/8.0 && v <= 7.0*dv/8.0)
		{
			r = 255;
			g = -1024*v + 896;
		}
		else if( v > 7.0*dv/8.0 && v <= 1.0)
		{
			r = -1024*v + + 1152;
		}
	}

	void StructuredLightReconstruction::saveCertaintyData(const CertaintyPoint& cp)
	{
		if(cp.point_.x > 0.01 && cp.point_.x < 0.09 && cp.point_.y > -0.04 && cp.point_.y < 0.04)
		{
			// static double a = -0.00576578, b = 0.0367201, c = 0.999309, d = -0.545408;
			static double a = -0.00603465, b = 0.0366186, c = 0.999311, d = -0.545436;

			static double div = sqrt(a*a+b*b+c*c);

			for(int j = 0; j < cp.left_edge_.size(); j++)
				data_dump_certainty_ << cp.left_edge_[j] << ",";

			for(int k = 0; k < cp.right_edge_.size(); k++)
				data_dump_certainty_ << cp.right_edge_[k] << ",";

			data_dump_certainty_ << cp.cam_pair_ << ",";
			double point_error = fabs((a*cp.point_.x + b*cp.point_.y + c*cp.point_.z + d)/(div));
			data_dump_certainty_ << point_error << ",";
			data_dump_certainty_ << cp.point_.x << "," << cp.point_.y << "," << cp.point_.z << ",";
		}
	}

	double StructuredLightReconstruction::addPoint(CertaintyPoint& cp)
	{
		if(save_certainty_values)
		{
			saveCertaintyData(cp);
		}

		pc_.points.push_back(cp.point_);
		return cp.certainty_;
	}



	void StructuredLightReconstruction::createPointCloud(int exposure_id, bool last_exposure)
	{
		applyOcclusionMask(exposure_id);

		/*std::ofstream diff("/home/kent/diff_img_bit3_plane_row420_left.txt", std::ios::out);
		int cols = cam_info_[CamIndex::LEFT].diff_img_vector[0][3].cols;
		for(int i = 0; i < cols; i++)
		{
			diff << cam_info_[CamIndex::LEFT].diff_img_vector[0][3].at<short>(420,i) << ",";
		}
		diff.close();

		std::ofstream pos("/home/kent/pos_img_bit3_plane_row420_left.txt", std::ios::out);
		for(int i = 0; i < cols; i++)
		{
			pos << (int) cam_info_[CamIndex::LEFT].pos_img_vector[0][3].at<uchar>(420,i) << ",";
		}
		pos.close();

		std::ofstream neg("/home/kent/neg_img_bit3_plane_row420_left.txt", std::ios::out);
		for(int i = 0; i < cols; i++)
		{
			neg << (int) cam_info_[CamIndex::LEFT].neg_img_vector[0][3].at<uchar>(420,i) << ",";
		}
		neg.close();*/

		/*std::ofstream diff1("/home/kent/diff_img_bit9_plane_row210.txt", std::ios::out);
		int cols1 = cam_info_[CamIndex::RIGHT].diff_img_vector[0][8].cols;
		for(int i = 0; i < cols1; i++)
		{
			diff1 << cam_info_[CamIndex::RIGHT].diff_img_vector[0][8].at<short>(210,i) << ",";
		}
		diff1.close();*/

		sign_error = 0;
		mat_error = 0;
		lagr = 0;
		code_error = 0;

		double t = ros::Time::now().toSec();
		int threads = 4;
		omp_set_dynamic(0);     // Explicitly disable dynamic teams
		omp_set_num_threads(threads); // Use 4 threads for all consecutive parallel regions

		std::vector<std::vector<pcl::PointXYZRGB> > point_vector(threads);

		// Vectors for re-calibration of stereo pairs
//		std::vector<std::vector<pcl::PointXYZRGB> > point_vector_lr(threads);
//		std::vector<std::vector<pcl::PointXYZRGB> > point_vector_lc(threads);
//		std::vector<std::vector<pcl::PointXYZRGB> > point_vector_cr(threads);

		for(int pair_idx = 0; pair_idx < cam_pairs_.size(); pair_idx++)
		{
			std::vector<std::vector<ExistingEdgeEntry> > edges_tmp(threads);
			std::vector<std::vector<CertaintyPointEntry> > points_tmp(threads);

			#pragma omp parallel for
			for(int p = 0; p < threads; p++)
			{
				int left_cam = cam_pairs_[pair_idx].left_cam;
				int right_cam = cam_pairs_[pair_idx].right_cam;
				int left_idx = right_cam-1;
				int right_idx = left_cam;

				std::map<int, double> edge_map_left;
				std::map<int, double> edge_map_right;

				for(int row = p * skip_rows_factor_; row < cam_info_[left_cam].enc_img[left_idx].rows; row+=threads * skip_rows_factor_)
				{
					findEdge(row, cam_info_[left_cam].enc_img[left_idx], cam_info_[left_cam].diff_img_vector[left_idx], pair_idx, true, edge_map_left);
					findEdge(row, cam_info_[right_cam].enc_img[right_idx], cam_info_[right_cam].diff_img_vector[right_idx], pair_idx, false, edge_map_right);

					for( std::map<int,double>::iterator it_left=edge_map_left.begin(); it_left!=edge_map_left.end(); ++it_left)
					{
						std::map<int,double>::iterator it_right = edge_map_right.find(it_left->first);
						if(it_right != edge_map_right.end())
						{
							pcl::PointXYZRGB point;
							if (reconstructEdge(row, left_cam, right_cam, left_idx, right_idx, it_left, it_right,
									edge_map_left, edge_map_right, pair_idx, exposure_id, edges_tmp[p], points_tmp[p], point))
							{
								// Fill vectors for re-calibration of stereo pairs
//								if (cam_pairs_[pair_idx].left_cam == CamIndex::LEFT &&
//										cam_pairs_[pair_idx].right_cam == CamIndex::RIGHT)
//								{
//									point_vector_lr[p].push_back(point);
//								}
//								else if(cam_pairs_[pair_idx].left_cam == CamIndex::LEFT &&
//										cam_pairs_[pair_idx].right_cam == CamIndex::CENTER)
//								{
//									point_vector_lc[p].push_back(point);
//								}
//								else if (cam_pairs_[pair_idx].left_cam == CamIndex::CENTER &&
//										cam_pairs_[pair_idx].right_cam == CamIndex::RIGHT)
//								{
//									point_vector_cr[p].push_back(point);
//								}

								// Original approach
								if(certainty_type_ == 0)
								{
									point_vector[p].push_back(point);
								}
							}
						}
					}
					edge_map_left.clear();
					edge_map_right.clear();
				}
			}
			for (int i = 0; i < edges_tmp.size(); i++)
			{
				for (int j = 0; j < edges_tmp[i].size(); j++)
				{
					insertExistingEdge(edges_tmp[i][j]);
				}
			}

			for (int i = 0; i < points_tmp.size(); i++)
			{
				for (int j = 0; j < points_tmp[i].size(); j++)
				{
					insertCertaintyPoint(points_tmp[i][j]);
				}
			}
		}


		int total_size = 0;
		double avg_certainty = 0;

		// Construct point cloud from certainty points
		if (last_exposure && certainty_type_ != 0)
		{
			for(int i = 0; i < certainty_points_.size(); i++)
			{
				total_size += certainty_points_[i].size();
				for( auto it = certainty_points_[i].begin(), end = certainty_points_[i].end();
					  it != end; it = certainty_points_[i].upper_bound(it->first) )
				{
					if(certainty_type_ == 1)
					{
						// Best certainty point or average
						std::vector<CertaintyPoint> temp_points;
						int idx = 0;
						int best_idx = 0;
						double best_certainty = 0;

						for( auto itt = certainty_points_[i].lower_bound(it->first); itt != certainty_points_[i].upper_bound(it->first) && itt != end; ++itt)
						{
							temp_points.push_back(itt->second);
							if(itt->second.certainty_ > best_certainty)
							{
								best_idx = idx;
								best_certainty = itt->second.certainty_;
							}
							idx++;
						}
						avg_certainty += addPoint(temp_points[best_idx]);

						if(temp_points[best_idx].cam_pair_ == 1 || temp_points[best_idx].cam_pair_ == 2)
						{
							for(int p = 0; p < temp_points.size(); p++)
							{
								if(p != best_idx)
								{
									if((temp_points[p].cam_pair_ == temp_points[best_idx].cam_pair_) && (temp_points[p].exposure_ == temp_points[best_idx].exposure_))
									{
										avg_certainty += addPoint(temp_points[p]);
									}
								}
							}
						}
					}
					else if(certainty_type_ == 2)
					{
						// Best certainty point or average
						std::vector<CertaintyPoint> temp_points;
						CertaintyPoint best_point;
						CertaintyPoint avg_point;
						double best_certainty = 0;

						for( auto itt = certainty_points_[i].lower_bound(it->first); itt != certainty_points_[i].upper_bound(it->first) && itt != end; ++itt)
						{
						  temp_points.push_back(itt->second);
						  if(itt->second.certainty_ > best_certainty)
						  {
							  best_point = itt->second;
							  best_certainty = itt->second.certainty_;
						  }
						}

						double certainty_sum = 0;
						double certainty_value = 0;
						int num_points = 0;
						for(int i = 0; i < temp_points.size(); i++)
						{
						  if(euclideanDistance(best_point.point_, temp_points[i].point_) < 0.0003)
						  {
							  avg_point.point_.x += temp_points[i].certainty_ * temp_points[i].point_.x;
							  avg_point.point_.y += temp_points[i].certainty_ * temp_points[i].point_.y;
							  avg_point.point_.z += temp_points[i].certainty_ * temp_points[i].point_.z;
							  certainty_sum += temp_points[i].certainty_;
							  certainty_value += temp_points[i].certainty_*temp_points[i].certainty_;
							  num_points++;
						  }
						}

						avg_point.point_.x = avg_point.point_.x / certainty_sum;
						avg_point.point_.y = avg_point.point_.y / certainty_sum;
						avg_point.point_.z = avg_point.point_.z / certainty_sum;
						avg_point.certainty_ = certainty_value/certainty_sum;

						if(pc_color_option_ == PC_COLOR_CERTAINTY)
						{
							int r,g,b;
							valToJet(avg_point.certainty_, r, g ,b);
							avg_point.point_.r = r;
							avg_point.point_.g = g;
							avg_point.point_.b = b;
						}
						else
						{
							avg_point.point_.r = 255;
							avg_point.point_.g = 255;
							avg_point.point_.b = 255;
						}
						avg_certainty += addPoint(avg_point);
					}
					else if(certainty_type_ == 3)
					{
						CertaintyPoint cp;
						for( auto itt = certainty_points_[i].lower_bound(it->first); itt != certainty_points_[i].upper_bound(it->first) && itt != end; ++itt)
						{
							avg_certainty += addPoint(itt->second);
						}
					}
					else
					{
						ROS_ERROR_STREAM("Wrong certainty type: " << certainty_type_ << "!! Please specify a value between 0-3");
					}

					if(save_certainty_values)
					{
						  data_dump_certainty_ << std::endl;
					}
				}
			}
		}

		// Compute extra-transform for stereo pairs based on ICP result
//		pcl::PointCloud<pcl::PointXYZRGB> pc_lr, pc_lc, pc_cr;
//		for (size_t i = 0; i < point_vector_lr.size(); i++)
//		{
//			pc_lr.points.insert(pc_lr.points.end(), point_vector_lr[i].begin(), point_vector_lr[i].end());
//		}
//		for (size_t i = 0; i < point_vector_lc.size(); i++)
//		{
//			pc_lc.points.insert(pc_lc.points.end(), point_vector_lc[i].begin(), point_vector_lc[i].end());
//		}
//		for (size_t i = 0; i < point_vector_cr.size(); i++)
//		{
//			pc_cr.points.insert(pc_cr.points.end(), point_vector_cr[i].begin(), point_vector_cr[i].end());
//		}
//
//		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//		icp.setInputCloud(pc_cr.makeShared());
//		icp.setInputTarget(pc_lr.makeShared());
//		pcl::PointCloud<pcl::PointXYZRGB> Final;
//		icp.setMaximumIterations(1000);
//		icp.align(Final);
//		ROS_WARN_STREAM("CR: has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore());
//		ROS_WARN_STREAM(icp.getFinalTransformation());
//
//		icp.setInputCloud(pc_lc.makeShared());
//		icp.setInputTarget(pc_lr.makeShared());
//		pcl::PointCloud<pcl::PointXYZRGB> Final2;
//		icp.align(Final2);
//		ROS_WARN_STREAM("LC: has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore());
//		ROS_WARN_STREAM(icp.getFinalTransformation());


		// *** Load points using original point vectors *** //
		if(certainty_type_ == 0)
		{
			for (size_t i = 0; i < point_vector.size(); i++)
			{
				pc_.points.insert(pc_.points.end(), point_vector[i].begin(), point_vector[i].end());
			}
		}

		pc_.width = pc_.points.size();
		pc_.height = 1;

		// Compute error to plane patch
		//computePlaneError(pc_.makeShared());
		ROS_INFO_STREAM("PointCloud constructed with " << pc_.points.size() << " points from " << total_size << " total points with average certainty: " << (avg_certainty/pc_.points.size()) << " - It took: " << (ros::Time::now().toSec() - t)*1000 << " ms");

		if (last_exposure)
		{
			postProcessPointCloud();
		}

		ROS_INFO_STREAM("Sign error: " << sign_error << " Mat error: " << mat_error << " Lagrange Errors: " << lagr << " Code errors: " << code_error);

		savePointClouds(pc_, pc_trimmed_, exposure_id);
	}

	void StructuredLightReconstruction::postProcessPointCloud()
	{
		pc_trimmed_.clear();
		// Remove points which are out of the scanners range (Not strictly necessary)
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud (pc_.makeShared());
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.1, 1.0);
		pass.filter (pc_trimmed_);

		if (remove_dominant_plane_)
		{
			pcl::SACSegmentation<pcl::PointXYZRGB> seg;
			seg.setOptimizeCoefficients (true);
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (100);
			seg.setDistanceThreshold (0.003);
			seg.setInputCloud (pc_trimmed_.makeShared());

			// Obtain the plane inliers and coefficients
			pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
			pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
			seg.segment(*inliers_plane, *coefficients_plane);

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud(pc_trimmed_.makeShared());
			extract.setIndices(inliers_plane);
			extract.setNegative (true);
			extract.filter (pc_trimmed_);
		}

		if (remove_outliers_)
		{
			double t = ros::Time::now().toSec();
			int pc_size_before = pc_trimmed_.points.size();

			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
			sor.setInputCloud(pc_trimmed_.makeShared());
			sor.setMeanK(mean_k_);
			sor.setStddevMulThresh(std_dev_thresh_);
			//sor.setNegative (true); // To view outliers
			sor.filter(pc_trimmed_);

			ROS_INFO_STREAM("Outliers filtered from point cloud: " << (pc_size_before -  pc_trimmed_.points.size()) << " Points - from " << pc_size_before << " - It took: " << (ros::Time::now().toSec() - t)*1000 << " ms - MeanK: " << mean_k_ << " StdDev: " << std_dev_thresh_);
		}
	}

	/**
	 Triangulation using "Q" matrix assuming epipolar constraint is satisfied, which it is when the cameras are rectified
	 */
	pcl::PointXYZRGB StructuredLightReconstruction::triangulationQ(const cv::Mat& u,       //homogenous image point (u,v,1)
																				const cv::Mat& P,       //camera 1 matrix
																				const cv::Mat& u1,      //homogenous image point in 2nd camera
																				const cv::Mat& P1       //camera 2 matrix
																	)
	{
		double w = (u.at<double>(0,0) - u1.at<double>(0,0)) * (-1 / P1.at<double>(0,3)) * P.at<double>(0,0);

		pcl::PointXYZRGB ret;
		ret.x = (u.at<double>(0,0) - P.at<double>(0,2)) / w;
		ret.y = (u.at<double>(1,0) - P.at<double>(1,2)) / w;
		ret.z = P.at<double>(0,0) / w;

		return ret;
	}


	double computeCubicValue(double p[4], double x)
	{
			return p[1] + 0.5 * x*(p[2] - p[0] + x*(2.0*p[0] - 5.0*p[1] + 4.0*p[2] - p[3] + x*(3.0*(p[1] - p[2]) + p[3] - p[0])));
	}

	double cubicRoot(double y[4])
	{
		double a = -0.5*y[0] + 1.5*y[1] - 1.5*y[2] + 0.5*y[3];
		double b = y[0] - 2.5*y[1] + 2*y[2] - 0.5*y[3];
		double c = 0.5*(y[2]-y[0]);
		double d = y[1];

		double p = -b/(3*a);
		double q = p*p*p + ((b*c-3*a*d)/(6*a*a));
		double r = c/(3*a);

		return pow(q + sqrt(q*q+pow(r-p*p,3.0)),1/3) + pow(q - sqrt(q*q+pow(r-p*p,3.0)),1/3) + p;
	}

	double cubicInterpolation(double y[4])
	{
		double a[4];

		const int MAXIT=30;
		const double xacc = 1e-8;
		double xl(0.0),xh(1.0),del;

		double fl = y[1];
		double fh = y[2];

		if (fl*fh > 0.0)
		{
			return (-1.0);
		}

		if (fl > 0.0)
		{
			std::swap(xl,xh);
			std::swap(fl,fh);
		}
		double dx=xh-xl;
		for (int j = 0; j < MAXIT; j++)
		{
			double rtf=xl+dx*fl/(fl-fh);
			double f=computeCubicValue(y, rtf);
			if (f < 0.0)
			{
				del=xl-rtf;
				xl=rtf;
				fl=f;
			}
			else
			{
				del=xh-rtf;
				xh=rtf;
				fh=f;
			}
			dx=xh-xl;
			if (fabs(del) < xacc || f == 0.0)
				return rtf;
		}
		return (-1.0);
	}

	double StructuredLightReconstruction::lagrange(const double x, const cv::Mat_<double>& ylist)
	{
		double ret_val = 0;
		double xlist[4] = {-1,0,1,2};

		for(int i = 0; i < 4; i++)
		{
			double temp = 1.0;
			for(int j = 0; j < 4; j++)
			{
				if(i == j)
				{
					continue;
				}
				else
				{
					temp = temp * ((x-xlist[j])/(xlist[i]-xlist[j]));
				}
			}
			ret_val = ret_val + ylist(i)*temp;
		}
		return ret_val;
	}

	double StructuredLightReconstruction::lagrangeEdge(const cv::Mat_<double>& ylist)
	{
		const int MAXIT=30;
		const double xacc = 1e-8;
		double xl(0.0),xh(1.0),del;

		double fl = lagrange(xl, ylist);
		double fh = lagrange(xh, ylist);

		if (fl*fh > 0.0)
		{
			return (-1.0);
		}

		if (fl > 0.0)
		{
			std::swap(xl,xh);
			std::swap(fl,fh);
		}
		double dx=xh-xl;
		for (int j = 0; j < MAXIT; j++)
		{
			double rtf=xl+dx*fl/(fl-fh);
			double f=lagrange(rtf, ylist);
			if (f < 0.0)
			{
				del=xl-rtf;
				xl=rtf;
				fl=f;
			}
			else
			{
				del=xh-rtf;
				xh=rtf;
				fh=f;
			}
			dx=xh-xl;
			if (fabs(del) < xacc || f == 0.0)
				return rtf;
		}
		return (-1.0);
	}

	double averageEdgeCertainty(double avg_edge)
	{
		double avg_certainty;
		double coeff_low_0 = 0;
		double coeff_low_1 = 80;
		double coeff_high_1 = 100;
		double coeff_high_0 = 240;

		if(avg_edge <= coeff_low_0)
		{
			avg_certainty = 0.0;
		}
		else if(avg_edge >= coeff_high_0)
		{
			avg_certainty = 0.0;
		}
		else if(avg_edge > coeff_low_0 && avg_edge < coeff_low_1)
		{
			avg_certainty = -avg_edge/(coeff_low_0-coeff_low_1) + coeff_low_0/(coeff_low_0-coeff_low_1);
		}
		else if(avg_edge > coeff_high_1 && avg_edge < coeff_high_0)
		{
			avg_certainty = -avg_edge/(coeff_high_0-coeff_high_1) + coeff_high_0/(coeff_high_0-coeff_high_1);
		}
		else
		{
			avg_certainty = 1.0;
		}

		return avg_certainty;
	}

	double maxEdgeCertainty(double max)
	{
		double avg_certainty;
		double coeff_high_1 = 180;
		double coeff_high_0 = 240;

		double max_certainty = 1.0;
		if(max <= coeff_high_1)
		{
			max_certainty = 1.0;
		}
		else if(max >= coeff_high_0)
		{
			max_certainty = 0;
		}
		else
		{
			max_certainty = max/(coeff_high_1-coeff_high_0) + coeff_high_0/(coeff_high_0-coeff_high_1);
		}

		return max_certainty;
	}

	double diffEdgeCertainty(double avg_left, double avg_right)
	{
		double diff_edge = fabs(avg_right-avg_left);
		double diff_certainty = 0.0;
		double coeff_high_1 = 30;
		double coeff_high_0 = 150;

		if(diff_edge <= coeff_high_1)
		{
			diff_certainty = 1.0;
		}
		else if(diff_edge >= coeff_high_0)
		{
			diff_certainty = 0.0;
		}
		else
		{
			diff_certainty = diff_edge/(coeff_high_1-coeff_high_0) + coeff_high_0/(coeff_high_0-coeff_high_1);
		}

		return diff_certainty;
	}

	bool StructuredLightReconstruction::JEPP_Interpolate(double& offset, int row, int col, const CamInfo& cam_info, int index, double& edge_certainty, std::vector<int>& edge_values)
	{
		double cBit = num_layers_-log2(abs((short)cam_info.enc_img[index].at<short>(row,col) - (short)cam_info.enc_img[index].at<short>(row,col+1)));

		if(cBit - floor(cBit) != 0){
			return false;
		}
		if(cBit == 0 || cBit == num_layers_+1){
			return false;
		}

		const short* row_ptr = cam_info.diff_img_vector[index].at(cBit-1).ptr<short>(row);

		double edge[] = {(double)row_ptr[col-1], (double)row_ptr[col], (double)row_ptr[col+1], (double)row_ptr[col+2]};

		if(sign(edge[1]) == sign(edge[2]))
		{
			sign_error++;
			return false;
		}

		if(fabs(edge[1] - edge[2]) < 3)
		{
			mat_error++;
			return false;
		}

		// Linear interpolation
		offset = edge[1]/(edge[1]-edge[2]);

		if(certainty_type_ != 0)
		{
			const uchar* pos_row_ptr = cam_info.pos_img_vector[index].at(cBit-1).ptr<uchar>(row);
			const uchar* neg_row_ptr = cam_info.neg_img_vector[index].at(cBit-1).ptr<uchar>(row);

			int pos[] = {(int)pos_row_ptr[col-1], (int)pos_row_ptr[col], (int)pos_row_ptr[col+1], (int)pos_row_ptr[col+2]};
			int neg[] = {(int)neg_row_ptr[col-1], (int)neg_row_ptr[col], (int)neg_row_ptr[col+1], (int)neg_row_ptr[col+2]};

			for(int i = 0; i < 4; i++)
				edge_values.push_back(pos[i]);

			for(int j = 0; j < 4; j++)
				edge_values.push_back(neg[j]);

			//data_dump_ << col << "," << pos[0] << "," << pos[1] << "," << pos[2] << "," << pos[3] << ",";
			//data_dump_ << neg[0] << "," << neg[1] << "," << neg[2] << "," << neg[3] << ",";

			double avg_sum_left;
			double avg_sum_right;

			if(pos[0] > 0 && pos[3] > 0)
			{
				avg_sum_left  = ((double)pos[0] + (double)neg[0] + (double)pos[1] + (double)neg[1])/4.0;
				avg_sum_right = ((double)pos[2] + (double)neg[2] + (double)pos[3] + (double)neg[3])/4.0;
			}
			else
			{
				avg_sum_left  = ((double)pos[1] + (double)neg[1])/2;
				avg_sum_right = ((double)pos[2] + (double)neg[2])/2;
			}

			double max = 0;

			for(int i = 0; i < 4; i++)
			{
				if(pos[i] > max)
				{
					max = (double)pos[i];
				}
				if(neg[i] > max)
				{
					max = (double)neg[i];
				}
			}



			// Apply edge correcting offset
			double extra_offset = 0;
			//extra_offset= 2*(avg_sum_right-avg_sum_left)/(avg_sum_right+avg_sum_left);

			offset = offset + extra_offset;

			// Compute edge certainty
			double avg_edge = (avg_sum_right+avg_sum_left)/2.0;
			double avg_certainty = averageEdgeCertainty(avg_edge);
			double max_certainty = maxEdgeCertainty(max);
			double diff_certainty = diffEdgeCertainty(avg_sum_left, avg_sum_right);
			//double diff_sum_certainty = fabs(edge[1]-edge[2]);

//			double a = 0.64;
//			double b = 0.95;
//			double c = 0.885;

			//double a = 0.33;
			//double b = 0.75;
			//double c = 0.68;
			double a = 1.0;
			double b = 1.0;
			double c = 1.0;
			double d = 1.0;

			edge_certainty = (a*avg_certainty + b*max_certainty + c*diff_certainty)/(a+b+c);
		}
		else
		{
			edge_certainty = 1.0;
		}

		/*ROS_WARN_STREAM_THROTTLE(0.1, "Pos: " << (int)pos_row_ptr[col-1] << " , " << (int)pos_row_ptr[col] << " , " << (int)pos_row_ptr[col+1] << " , " << (int)pos_row_ptr[col+2] <<
				" Neg: " << (int)neg_row_ptr[col-1] << " , " << (int)neg_row_ptr[col] << " , " << (int)neg_row_ptr[col+1] << " , " << (int)neg_row_ptr[col+2] <<
				" Diff: " << (int)row_ptr[col-1] << " , " << (int)row_ptr[col] << " , " << (int)row_ptr[col+1] << " , " << (int)row_ptr[col+2] <<
				" Avg edge: " << (double)avg_edge << " Max: " << (double)max << " Diff edge: " << (double)diff_edge <<
				" Avg cer : " << (double)avg_certainty << " Max cer " << (double)max_certainty << " Diff certainty: " << (double)diff_certainty << " Certainty: " << (double)edge_certainty);*/

		//offset = cubicInterpolation(edge);
		//offset = cubicRoot(edge);
//		ROS_WARN_STREAM_THROTTLE(0.1,"Edge: " << edge[0] << "," << edge[1] << "," << edge[2] << "," << edge[3] <<
//				" Linear: " << offset << " Cubic num: " << cubicInterpolation(edge) << "Cubic ana: " << cubicRoot(edge));
//			offset = lagrangeEdge(Matdata); // Lagrange polynomial estimation and Regula falsi root finding
//			if (offset < 0.0)
//			{
//				return false;
//				lagr++;
//			}

			//offset = 0.5; // No subpixel estimation
		return true;
	}

} /* namespace vrm3dvision */
