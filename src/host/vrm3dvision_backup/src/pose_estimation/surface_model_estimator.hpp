/********************************************************************************************************************
 *
 * \file                surface_model_estimator.hpp
 * \author              Kent Hansen (kenh@teknologisk.dk)
 * \date                2014-03-14
 * \version             1.0
 * \brief               Pose estimation based on surface models using Halcon
 *
*********************************************************************************************************************/

#ifndef SURFACE_MODEL_ESTIMATOR_HPP_
#define SURFACE_MODEL_ESTIMATOR_HPP_

#include "object_model.hpp"

// Halcon includes
#include <halconcpp/HalconCpp.h>

namespace vrm3dvision {

class SurfaceModelEstimator {
public:
	SurfaceModelEstimator();
	virtual ~SurfaceModelEstimator();

	bool prepareModel(ObjectModel& model);
	bool estimatePose(const pcl::PointCloud<pcl::PointXYZRGB>& input_cloud, ObjectModel& model, Eigen::Matrix4f& t);

	static bool isHalconLicenseAvailable();

private:

	bool loadSurfaceModel(const std::string& load_path);
	bool saveSurfaceModel(const std::string& save_path);

	template<class T> void convertCloudToHalcon(const pcl::PointCloud<T>& cloud, HalconCpp::HTuple& x, HalconCpp::HTuple& y, HalconCpp::HTuple& z)
	{
		for(size_t i = 0; i < cloud.size(); i++)
		{
			x.Append(cloud.points[i].x);
			y.Append(cloud.points[i].y);
			z.Append(cloud.points[i].z);
		}
	}

	bool createSurfaceModelFromPoints(const HalconCpp::HTuple& x, const HalconCpp::HTuple& y,
			const HalconCpp::HTuple& z, const SurfaceModelParams& params, HalconCpp::HTuple* model_handle);

	void createObjectModelWithNormals(const HalconCpp::HTuple& x, const HalconCpp::HTuple& y,
			const HalconCpp::HTuple& z, HalconCpp::HTuple* handle);

	void parseDetectionParameters(const SurfaceModelParams& params, HalconCpp::HTuple& gen_param_name, HalconCpp::HTuple& gen_param_value);
	void parseCreationParameters(const SurfaceModelParams& params, HalconCpp::HTuple& gen_param_name, HalconCpp::HTuple& gen_param_value);
};

} /* namespace vrm3dvision */

#endif /* SURFACE_MODEL_ESTIMATOR_HPP_ */
