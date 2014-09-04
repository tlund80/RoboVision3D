/*
 * PCDFileHandler.cpp
 *
 *  Created on: Feb 11, 2013
 *      Author: thomas
 */

#include <pose_estimation_halcon/PCDFileHandler.h>

namespace perception {

PCDFileHandler::PCDFileHandler() {
	// TODO Auto-generated constructor stub

}

PCDFileHandler::~PCDFileHandler() {
	// TODO Auto-generated destructor stub
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCDFileHandler::loadPCDfileXYZRGB(std::string file_path)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_path, *cloud) == -1) //* load the file
	{
	  PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	}else
	{

	 std::cout << "Loaded "
	           << cloud->width * cloud->height
	           << " data points from"
	           << file_path
	           << " with the following fields: "
	           << std::endl;

	}

	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCDFileHandler::loadPCDfileXYZRGBA(std::string file_path)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (file_path, *cloud) == -1) //* load the file
	{
	  PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	}else
	{

	 std::cout << "Loaded "
	           << cloud->width * cloud->height
	           << " data points from"
	           << file_path
	           << " with the following fields: "
	           << std::endl;

	}

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCDFileHandler::loadPCDfileXYZ(std::string file_path)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_path, *cloud) == -1) //* load the file
	{
	  PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	}else
	{

	 std::cout << "Loaded "
	           << cloud->width * cloud->height
	           << " data points from"
	           << file_path
	           << " with the following fields: "
	           << std::endl;

	}

	return cloud;
}

void PCDFileHandler::savePCDfileXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>& cloud,std::string file_path)
{
	 pcl::io:: savePCDFileASCII (file_path, cloud);

	 std::cout << "Saved " << cloud.points.size ()
						   << " data points to"
						   << file_path
						   << std::endl;
}

void PCDFileHandler::savePCDfileXYZ(pcl::PointCloud<pcl::PointXYZ>& cloud,std::string file_path)
{
	 pcl::io::savePCDFileASCII (file_path, cloud);

	 std::cout << "Saved " << cloud.points.size ()
						   << " data points to"
						   << file_path
						   << std::endl;
}


} /* namespace perception */
