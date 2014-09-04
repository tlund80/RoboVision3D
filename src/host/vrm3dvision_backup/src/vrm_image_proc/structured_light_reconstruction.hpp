/********************************************************************************************************************
 *
 * \file                structured_light_reconstruction.hpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-12-02
 * \version             0.1
 * \brief
 *
*********************************************************************************************************************/
#ifndef _STRUCTURED_LIGHT_RECONSTRUCTION_HPP_
#define _STRUCTURED_LIGHT_RECONSTRUCTION_HPP_

// ROS includes
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <ros/package.h>

// VRM communication protocol
#include <vrm_protocol/image_group_msg.hpp>
#include <vrm_global.h>

// Local includes
#include "occlusion_mask.hpp"
#include "visualization.hpp"

// Standard libraries
#include <string>
#include <vector>
#include <map>
#include <iomanip>

// OPENCV Includes
#include <opencv2/opencv.hpp>

// OpenMP includes
#include <omp.h>

// BOOST
//#define BOOST_NO_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

// PCL
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#define PC_COLOR_NONE 10
#define PC_COLOR_EXPO 11
#define PC_COLOR_CAM_SET 12
#define PC_COLOR_CERTAINTY 13


namespace vrm3dvision {

	struct CamInfo {
		cv::Mat K;
		cv::Mat D;
		cv::Mat R[2];
		cv::Mat P[2];
		cv::Mat T[2];
		cv::Mat rect_map_to_anchor;
		cv::Size size;
		cv::Mat rect_map_x[2];
		cv::Mat rect_map_y[2];
		//OcclusionMask mask;
		cv::Mat occlusion_mask[2];
		cv::Mat ambient_img;
		cv::Mat min;
		cv::Mat max;

		std::vector<cv::Mat> pos_img_vector[2];
		std::vector<cv::Mat> neg_img_vector[2];
		std::vector<cv::Mat> diff_img_vector[2];
		cv::Mat enc_img[2];
		cv::Mat pos_buffer_img;
		cv::Mat neg_buffer_img;
		bool active;
	};

	/** @class StructuredLightReconstruction
	  *
	  */
	class StructuredLightReconstruction
	{
		public:
			/// Constructor
			StructuredLightReconstruction();

			void initialize(const std::string& calibration_path, const std::vector<std::string>& calib_names, const std::string& camera_frame, bool visualization,
					const std::string& pc_color_option, bool remove_outliers, bool remove_dominant_plane, double std_dev_thresh, int mean_k, int certainty_type);

			int addImages(const vrm_protocol::image_group& ig);
			bool saveNextSequence(const std::string& folder_name);
			const pcl::PointCloud<pcl::PointXYZRGB>& getPointCloud() { return pc_; };
			const pcl::PointCloud<pcl::PointXYZRGB>& getPointCloudTrimmed() { return pc_trimmed_; };

			const cv::Mat& getLeftRectMapY() { return cam_info_[0].rect_map_y[1]; };
			const cv::Mat& getLeftRectMapX() { return cam_info_[0].rect_map_x[1]; };

			void clearExistingEdgesAndCloud();
			std::vector<std::map<int,bool> >& getExistingEdges() { return existing_edges_;};

		private:
			enum CamIndex {
				LEFT = 0,
				CENTER = 1,
				RIGHT = 2
			};

			struct CamPair {
				CamIndex left_cam;
				CamIndex right_cam;
			};

			struct ExistingEdgeEntry {
				int row_;
				int key_;
				bool object_point_;
				ExistingEdgeEntry(int row, int key, bool object_point)
				{
					row_ = row;
					key_ = key;
					object_point_ = object_point;
				}
			};

			struct CertaintyPoint {
				std::vector<int> left_edge_;
				std::vector<int> right_edge_;
				int cam_pair_;
				int exposure_;
				double certainty_;
				pcl::PointXYZRGB point_;
			};

			struct CertaintyPointEntry {
				int row_;
				int key_;
				CertaintyPoint cp_;
				CertaintyPointEntry(int row, int key, double certainty, const pcl::PointXYZRGB point, const std::vector<int>& left_edge, const std::vector<int>& right_edge, int cam_pair, int exposure)
				{
					row_ = row;
					key_ = key;
					cp_.certainty_ = certainty;
					cp_.point_ = point;
					cp_.left_edge_ = left_edge;
					cp_.right_edge_ = right_edge;
					cp_.cam_pair_ = cam_pair;
					cp_.exposure_ = exposure;
				}
			};



			void initCamInfo(const std::string& calibration_url);

			// Visualization
			void initVisualization();
			void visualizeImages(const vrm_protocol::image_group& ig);
			int convertBinCodeToIndex(unsigned short value, bool large_gap_gc, int num_bits);
			cv::Mat convertEncodedImage2Color(const cv::Mat& in, const bool& long_run, const int& bit_level);
			void graycode2rgb(int val, int bit_level, int& r, int& g, int& b);
			void longrun2rgb(int val, int bit_level, int& r, int& g, int& b);

			void resetCamInfo();
			void initSequence(const vrm_protocol::image_group& ig);
			void updateCameraStatus(int cam_index, bool has_img);
			void saveBlackOcclusionImage(const vrm_protocol::image_group& ig);
			void saveWhiteOcclusionImage(const vrm_protocol::image_group& ig);
			void saveAmbientImage(const vrm_protocol::image_group& ig);
			void saveNegativeBufferImage(const vrm_protocol::image_group& ig);
			void savePositiveBufferImageAndUpdateEncodedImage(const vrm_protocol::image_group& ig, int bit_level);
			void updateEncodedImage(CamInfo& cam_info, int bit_level);
			void createPointCloud(int exposure_id, bool last_exposure);
			void saveImages(const vrm_protocol::image_group& ig);

			inline bool firstImage(const vrm_protocol::image_group& ig) { return 	(ig.header.image_id() == 0 && ig.header.has_ambient_img() == false) ||
																					(ig.header.image_id() == 60 && ig.header.has_ambient_img() == true); };
			inline bool blackOcclusionImage(const vrm_protocol::image_group& ig) { return ig.header.image_id() == 0; };
			inline bool whiteOcclusionImage(const vrm_protocol::image_group& ig) { return ig.header.image_id() == 1; };
			inline bool ambientImage(const vrm_protocol::image_group& ig) { return ig.header.image_id() == 60; };
			inline bool evenImageIndex(const vrm_protocol::image_group& ig) { return !(ig.header.image_id() % 2); };
			inline bool validImage(const vrm_protocol::image_group& ig) { return (	ig.header.sequence_id() == current_sequence_id_ &&
																					((ig.header.image_id() == next_image_id_ &&
																					ig.header.image_id() <= ig.header.num_images()-1) ||
																					ambientImage(ig))); };
			inline bool lastImageInSequence(const vrm_protocol::image_group& ig) { return ig.header.image_id() == ig.header.num_images()-1; };

			pcl::PointXYZRGB cv_triangulate(const cv::Mat& u, const cv::Mat& P, const cv::Mat& u1, const cv::Mat& P1);

			pcl::PointXYZRGB LinearLSTriangulation(const cv::Mat& u, const cv::Mat& P, const cv::Mat& u1, const cv::Mat& P1);

			pcl::PointXYZRGB IterativeLinearLSTriangulation(const cv::Mat& u, const cv::Mat& P, const cv::Mat& u1, const cv::Mat& P1);

			pcl::PointXYZRGB dti_triangulation(const cv::Mat& u, const cv::Mat& P, const cv::Mat& u1, const cv::Mat& P1);
			pcl::PointXYZRGB triangulationQ(const cv::Mat& u, const cv::Mat& P, const cv::Mat& u1, const cv::Mat& P1);

			void setPointColorFromIdx(pcl::PointXYZRGB& point, int color_idx);

			bool STR_Interpolate(double& offset, int row, int col, const CamInfo& cam);
			bool JEPP_Interpolate(double& offset, int row, int col, const CamInfo& cam_info, int index, double& edge_certainty, std::vector<int>& edge_values);

			void test_rectification(const cv::Mat& left_image, const CamInfo& left_cam_info, const cv::Mat& right_image, const CamInfo& right_cam_info);

			bool checkValidEdge(const unsigned short left_value, const unsigned short right_value);
			void findEdge(int row, const cv::Mat& enc_img, const std::vector<cv::Mat>& diff_vector, int pair_idx, bool overwrite_edge, std::map<int,double>& edge_map);
			cv::Mat convertBayerToGray16S(const cv::Mat& src);
			cv::Mat convertBayerToGray8U(const cv::Mat& src);

			bool reconstructEdge(int row, int left_cam, int right_cam, int left_idx, int right_idx,
					const std::map<int,double>::iterator& it1, const std::map<int,double>::iterator& it2,
					std::map<int, double>& edge_map_1, std::map<int, double>& edge_map_2,
					int pair_idx, int exposure_id,
					std::vector<ExistingEdgeEntry>& temp_existing_edges,
					std::vector<CertaintyPointEntry>& temp_certainty_points, pcl::PointXYZRGB& point);

			void transformPoint(const cv::Mat& T, pcl::PointXYZRGB& point);
			void insertExistingEdge(ExistingEdgeEntry entry);
			void insertCertaintyPoint(const CertaintyPointEntry& entry);

			void applyOcclusionMask(int exposure_id);

			void savePointCloudToPCD(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const std::string& path, const std::string& filename, int exposure_id, bool save_without_color = false);
			void savePointClouds(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const pcl::PointCloud<pcl::PointXYZRGB>& cloud_trimmed, int exposure_id);

			double lagrangeEdge(const cv::Mat_<double>& ylist);
			double lagrange(const double x, const cv::Mat_<double>& ylist);

			void postProcessPointCloud();
			double addPoint(CertaintyPoint& cp);
			void saveCertaintyData(const CertaintyPoint& point);
			void valToJet(double v, int& r, int& g, int& b);
			CamInfo cam_info_[3];

			std::string camera_frame_;

			// Occlusion mask variables
			int occ_diff_threshold_;
			int occ_score_threshold_;

			// Visualization variables
			bool visualization_;

			int num_layers_;
			int num_exposures_;
			int skip_rows_factor_;
			int pc_color_option_;

			// bookkeeping variables
			int current_sequence_id_;
			int next_image_id_;
			std::vector<std::map<int,bool> > existing_edges_;
			std::vector<std::multimap<int, CertaintyPoint> > certainty_points_;
			int certainty_type_;
			bool save_certainty_values;
			double time;

			// Gray code index lists
			static const int index_lggc_10[];
			static const int index_lggc_9[];
			static const int index_lggc_8[];
			static const int index_lggc_7[];
			static const int index_brgc[];

			bool using_large_gap_gc_;
			// Post processing variables;
			bool use_ambient_occlusion_mask_;
			bool remove_outliers_;
			bool remove_dominant_plane_;
			int mean_k_;
			float std_dev_thresh_;

			pcl::PointCloud<pcl::PointXYZRGB> pc_;
			pcl::PointCloud<pcl::PointXYZRGB> pc_trimmed_;

			std::vector<CamPair> cam_pairs_;

			bool save_next_sequence_;
			bool last_cloud_saved_;
			std::ofstream image_headers_;
			std::string calibration_path_;
			std::string save_path_;
			std::vector<std::string> calib_file_names_;

			// TESTING
			int sign_error;
			int mat_error;
			int code_error;
			int lagr;

			inline double squaredEuclideanDistance (const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2)
			{
				float diff_x = p2.x - p1.x, diff_y = p2.y - p1.y, diff_z = p2.z - p1.z;
				return (diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
			}

			inline double euclideanDistance (const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2)
			{
				return sqrt(squaredEuclideanDistance(p1, p2));
			}

			std::ofstream data_dump_;
			std::ofstream data_dump_certainty_;

	};

} /* namespace vrm3dvision */

#endif /* _STRUCTURED_LIGHT_RECONSTRUCTION_HPP_ */
