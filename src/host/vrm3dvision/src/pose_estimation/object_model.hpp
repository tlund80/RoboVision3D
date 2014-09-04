/********************************************************************************************************************
 *
 * \file                object_model.hpp
 * \author              Kent Hansen (kenh@teknologisk.dk)
 * \date                2014-03-18
 * \version             1.0
 * \brief               Object model class
 *
*********************************************************************************************************************/

#ifndef OBJECT_MODEL_HPP_
#define OBJECT_MODEL_HPP_

// ROS
#include <ros/ros.h>
#include <vrm3dvision/AlignmentPrerejectiveParams.h>
#include <vrm3dvision/SurfaceModelParams.h>

// PCL
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h> // Load polygon files
#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#include <Eigen/StdVector>

// Boost
#include <boost/filesystem.hpp>
#include <boost/circular_buffer.hpp>

namespace vrm3dvision {

struct view_set_comp {
  bool operator() (const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) const
  {
	  if (lhs.first>rhs.first)
		  return true;
	  if (lhs.first<rhs.first)
		  return false;
	  if (lhs.second<rhs.second)
		  return true;
	  if (lhs.second>=rhs.second)
		  return false;
	  return false;
  }
};

class ObjectModel {
public:
	ObjectModel(int tesslation_level = 1, int resolution = 350, float sphere_radius = 0.35, float field_of_view = 60, int view_buffer_size = 100);
	virtual ~ObjectModel();
	bool initFromModel(const std::string& model_name);
	bool loadRawClouds();
	bool createNewModelFromCad(const std::string& model_name, const std::string& cad_path);

	static const std::string& getPathToModels() { static std::string model_path = getenv("HOME") + std::string("/.ros/models/"); return model_path; };

	std::string& getModelName() { return model_name_; };
	std::string& getModelFolder() { return model_folder_; };
	pcl::PolygonMesh getModelMesh() { return model_mesh_; };
	int getNumViews() { return num_views_; };

	bool updateParameters(const SurfaceModelParams& smp, const AlignmentPrerejectiveParams& app);
	bool updateAlignmentPrerejectiveParams(const AlignmentPrerejectiveParams& new_params);
	bool updateSurfaceModelParams(const SurfaceModelParams& new_params);

	std::vector<int>& getViewOrder();
	float getFitness(int index);
	void addSuccededView(int index);
	bool saveViewScore();

	double getSingleExpo() { return single_expo_; };
	double getDoubleExpoLow() { return double_expo_low_; };
	double getDoubleExpoHigh() { return double_expo_high_; };
	std::string getBestExpoString();
	std::string getSingleExpoString();
	std::string getDoubleExpoString();

	void updateExposure(double single, int single_edges, double double_low, double double_high, int double_edges);
	void setSingleExpo(double expo) { single_expo_ = expo; };
	void setDoubleExpoLow(double expo) { double_expo_low_ = expo; };
	void setDoubleExpoHigh(double expo) { double_expo_high_ = expo; };


	// Poses and clouds from partial views
	std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Vector4f> > poses_;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> raw_clouds_;

	// Information for Alignment Prerejective estimator
	AlignmentPrerejectiveParams alignment_prerejective_params_;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> ap_clouds_;
	std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> feature_clouds_;
	pcl::PointCloud<pcl::PointNormal>::Ptr ap_full_cloud_;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_cloud_full_;

	// Information for Halcon surface model estimator
	SurfaceModelParams surface_model_params_;
	std::vector<int> surface_model_ids_;


private:
	bool loadCadModel(const std::string& cad_path, pcl::PolygonMesh& mesh);
    bool saveModel(const std::string& cad_path);
    bool saveModelData();
    void resetParamsDefault();
    void resetSurfaceModelParamsDefault();
    void resetAlignmentPrerejectiveParamsDefault();
    bool saveParameters();
    bool saveSurfaceModelParams();
    bool saveAlignmentPrerejectiveParams();
    bool loadParameters();
    bool loadSurfaceModelParams();
    bool loadAlignmentPrerejectiveParams();
    bool modelFolderExists();
    friend std::ostream& operator <<(std::ostream &os, const ObjectModel& om);
	friend std::istream& operator >>(std::istream &is, ObjectModel& om);

	bool loadViewScore();
	bool initViewSet();

	std::string model_name_;
	std::string model_folder_;
	pcl::PolygonMesh model_mesh_;
	int num_views_;

	// Model generation parameters
	int tesselation_level_;
	int resolution_;
	float sphere_radius_;
	float field_of_view_;
	int view_buffer_size_;

	// View set variables
	std::multiset<std::pair<int, int>, view_set_comp> view_set;
	std::multiset<std::pair<int, int>, view_set_comp>::iterator view_set_itr;
	boost::circular_buffer<int> view_buffer;
	std::vector<int> view_idx;

	double single_expo_;
	int single_edges_;
	int double_edges_;
	double double_expo_low_;
	double double_expo_high_;
};

} /* namespace vrm3dvision */

#endif /* OBJECT_MODEL_HPP_ */
