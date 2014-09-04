/*
 * PCDFileHandler.h
 *
 *  Created on: Feb 11, 2013
 *      Author: thomas
 */

#ifndef PCDFILEHANDLER_H_
#define PCDFILEHANDLER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace perception {

class PCDFileHandler {
public:
	PCDFileHandler();
	virtual ~PCDFileHandler();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPCDfileXYZRGB(std::string file_path);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr loadPCDfileXYZRGBA(std::string file_path);
	pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCDfileXYZ(std::string file_path);

	void savePCDfileXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>& cloud,std::string file_path);
	void savePCDfileXYZ(pcl::PointCloud<pcl::PointXYZ>& cloud,std::string file_path);
};

} /* namespace perception */
#endif /* PCDFILEHANDLER_H_ */
