/*
 * ReconstructPointCloud.cpp
 *
 *  Created on: Aug 13, 2013
 *      Author: thomas
 */

#include <Modeling_Engine/ReconstructPointCloud.h>

namespace perception_3D {

ReconstructPointCloud::ReconstructPointCloud() {
	// TODO Auto-generated constructor stub

}

ReconstructPointCloud::~ReconstructPointCloud() {
	// TODO Auto-generated destructor stub
}
/////////////////////////////////////////////////////
//  Data smoothing and improved normal estimation////
/////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr ReconstructPointCloud::MLSApproximation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	  // Init object (second point type is for the normals, even if unused)
/*	  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::Normal> mls;

	  mls.setInputCloud (cloud);
	  mls.setPolynomialFit (true);
	  mls.setSearchRadius (0.03);

	  // Create a KD-Tree
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud (cloud);
	  mls.setSearchMethod (tree);

	  // Define the output and the normals
	  pcl::PointCloud<pcl::PointXYZ> mls_points;
	  pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new
	  pcl::PointCloud<pcl::Normal> ());
	  mls.setOutputNormals (mls_normals);
	  // Compute the smoothed cloud
	  mls.reconstruct (mls_points);

	  // Extra: merge fields
	  pcl::PointCloud<pcl::PointNormal> mls_cloud;
	  pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);

	// return mls_points;
*/
}
////////////////////////////////////////////////////////////////////
////Convert point cloud to mesh without modifying vertex positions//
///////////////////////////////////////////////////////////////////
pcl::PolygonMesh ReconstructPointCloud::GreedyProjectionTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);
	// Define inputs to thetriangulation structure
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	// Compute the mesh
	pcl::PolygonMesh triangles;
	gp3.reconstruct (triangles);

	return triangles;
}
/////////////////////////////////////////////////////////////////////////////
/// Simple triangulation/surface reconstruction for organized point clouds //
/////////////////////////////////////////////////////////////////////////////
pcl::PolygonMesh ReconstructPointCloud::OrganizedFastMaesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Initialize objects
	pcl::OrganizedFastMesh<pcl::PointXYZ> orgMesh;
	pcl::PolygonMesh triangles;

	// Create search tree*
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	orgMesh.setMaxEdgeLength (10);
	orgMesh.setTrianglePixelSize (1);
	orgMesh.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_ADAPTIVE_CUT );
	orgMesh.setInputCloud(cloud);
	orgMesh.setSearchMethod(tree);
	orgMesh.reconstruct(triangles);

	return triangles;
}



pcl::PointCloud<pcl::PointXYZ>::Ptr ReconstructPointCloud::loadPCDfile(std::string file_path)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_path, *cloud) == -1) //* load the file
	{
	  PCL_ERROR ("Couldn't read the pcd file\n");
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

void ReconstructPointCloud::saveToObj(const std::string file, pcl::PolygonMesh mesh)
{
	try{
		pcl::io::saveOBJFile(file,mesh);
	}catch(pcl::IOException &e)
	{
		std::cout << "Exception: In ReconstructPointCloud::saveToObj -> " << e.what() << std::endl;
	}
}

void ReconstructPointCloud::saveToVTK(const std::string file, pcl::PolygonMesh mesh)
{
	try{
		pcl::io::saveVTKFile(file,mesh);
	}catch(pcl::IOException &e)
	{
		std::cout << "Exception: In ReconstructPointCloud::saveToVTK-> " << e.what() << std::endl;
	}
}

} /* namespace perception */
