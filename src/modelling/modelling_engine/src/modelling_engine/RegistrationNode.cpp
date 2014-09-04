/*
 * RegistrationNode.cpp
 *
 *  Created on: Aug 10, 2013
 *      Author: thomas
 */

#include "Modeling_Engine/RegistrationNode.h"

namespace perception_3D {

RegistrationNode::RegistrationNode(boost::shared_ptr<perception_3D::SharedData> data_ptr, boost::shared_ptr<perception_3D::RosInterface> ros_ptr)  : _sharedData(data_ptr), _rosInterface(ros_ptr){
	// TODO Auto-generated constructor stub
	_state = NOP;
	_count = 0;
}

RegistrationNode::~RegistrationNode() {
	// TODO Auto-generated destructor stub
	stop();
}

void RegistrationNode::start(void)
{

	stop_thread = false;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	while(!stop_thread)
	{
		//DTI_INFO("Hello from Registration thread!");

		_state = _rosInterface->getScanningState();

		switch(_state){

		case INIT_REGISTRATION: //Initiale pose refinement. Using the Robot pose
			{

				if(_sharedData->getPointcloudListLength() > 0)
				{
						DTI_INFO("3D Modeling Engine: Initial registration!");
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr part (new pcl::PointCloud<pcl::PointXYZRGB>);
						// Get latest point cloud
						std::cout << "get newes scan data!!" << std::endl;
						boost::shared_ptr<perception_3D::ScanData> d = _sharedData->getNewestScanData();
						cloud = d->getPointCloud();
						removePlane(cloud,part);
						statisticalOutlierRemoval(part,0.5);
						std::cout << "Finish!!!" << std::endl;
						//Get transform
						Eigen::Matrix4f T; d->getTransform(T);
						std::cout << "Finish2!!!" << std::endl;
						// Make initial registration
						//transformed_cloud = transformCloud(cloud,T);
						Eigen::Matrix4f Y = Eigen::Matrix4f::Identity();
						transformed_cloud = transformCloud(part,Y);
						//pcl::copyPointCloud(*cloud,*transformed_cloud);

						stringstream str;
						str << "/home/thomas/leap_box/pp_";
						str << _count;
						str << ".pcd";
						string s = str.str();

						DTI_INFO("3D Modeling Engine: Saving cloud at " << s);
						pcl::io::savePCDFile(s,*transformed_cloud);
						_count++;
						
						_rosInterface->pubLastPointCloud(*part);
						
						//Jump to State REGISTRE
						_state++;
				}

			
			}
			//break;

		case REGISTRE:
			{
			DTI_INFO("3D Modeling Engine: Running Registration!!");

			//Align model and latest scan

			//if(_sharedData->getPointcloudListLength() > 0)
			//{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr total_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

				//Get model
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr model = _sharedData->getModelPointcloud();

				std::vector<int> indices1;
				pcl::removeNaNFromPointCloud(*transformed_cloud,*transformed_cloud,indices1);

				std::vector<int> indices2;
				pcl::removeNaNFromPointCloud(*model,*model,indices2);

				//if(model->points.size() > 0){

					//DownSample Cloud before registration
					//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sampled = DownSampleCloud(transformed_cloud,0.1); //2.5
					//pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_sampled = DownSampleCloud(model,0.1);

					checkForNaN(transformed_cloud);
					checkForNaN(model);

					Eigen::Matrix4f mat = Registre_views(transformed_cloud,model);
					//Eigen::Matrix4f mat = Registre_views_ICP(cloud_sampled,model_sampled);

					result_cloud = transformCloud(transformed_cloud,mat);

					//Add models
					*total_cloud = *result_cloud;
					//*total_cloud = *transformed_cloud;
					*total_cloud += *model;
			//	}else{
					//Add models
			//		*total_cloud = *transformed_cloud;
			//		*total_cloud += *model;

			//	}


				//Save Model in SharedData space
				_sharedData->setModelPointcloud(total_cloud->makeShared());

				//Publish model
				_rosInterface->pubRegisteredModel(*total_cloud);

			//}else //It is the first point cloud acquired!!
			//{
			//	DTI_INFO("3D Modeling Engine: First initial registration!!");
				//Set First scan as model without transformation
				//_sharedData->setModelPointcloud(cloud);

				//Publish model
			//	_rosInterface->pubRegisteredModel(*cloud);

			//}

				_rosInterface->setScanningState(NOP);
				_state = NOP;
			}
			break;

		case REFINE:
			DTI_INFO("3D Modeling Engine: Refining point cloud model!!");

			break;

		case RECONSTRUCT:
			DTI_INFO("3D Modeling Engine: Reconstruct model and saving triangular mesh!!!");

			break;

		case NOP:

			break;

		default:

			break;



		}
	}

}

void RegistrationNode::stop(void)
{
	stop_thread = true;
	DTI_INFO("Stopping RegistrationNode thread!!");
}

void RegistrationNode::checkForNaN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{

	for(int i = 0; i<= (int)cloud->size()-1; i++)
	{
		if(!pcl_isfinite(cloud->points[i].x) == true || !pcl_isfinite(cloud->points[i].y) == true || !pcl_isfinite(cloud->points[i].z) == true || !pcl_isfinite(cloud->points[i].r) == true || !pcl_isfinite(cloud->points[i].g) == true || !pcl_isfinite(cloud->points[i].b) == true)
		{
				std::cout << "ERROR!!!" << std::endl;
				cloud->at(i).x = 0;
				cloud->at(i).y = 0;
				cloud->at(i).z = 0;
				//cloud_sampled->points.erase(cloud_sampled->points.begin()+i);

				std::cout << " x: " << cloud->points[i].x
						  << " y: " << cloud->points[i].y
						  << " z: " << cloud->points[i].z
						  << " r: " << (int)cloud->points[i].r
						  << " g: " << (int)cloud->points[i].g
						  << " b: " << (int)cloud->points[i].b << std::endl;
		}
	}

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RegistrationNode::transformCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Matrix4f& transform)
{
	pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;

	try{
		pcl::transformPointCloud(*cloud, transformed_cloud, transform);
	}catch(pcl::PCLException &e)
	{
		DTI_ERROR("3D Modeling Engine: In RegistrationNode::transformCloud() " << e.what());
	}
	return transformed_cloud.makeShared();

}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr RegistrationNode::statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int mean)
{
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (mean);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);

	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RegistrationNode::DownSampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double leaf_size)
{
	 std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
	       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (leaf_size,leaf_size,leaf_size);
	sor.filter (*cloud_filtered);

	 std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
	       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  RegistrationNode::PassThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double min_depth, double max_depth)
{
	// Preprocess the cloud by...
	// ...removing distant points
	//const float depth_limit = 1.0;

	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (min_depth, max_depth);
	pass.filter (*cloud);

	return cloud;
}

Eigen::Matrix4f RegistrationNode::Registre_views(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_cloud)
{
	double gicp_FitnessEpsilon = 1e-6;
	double gicp_MaxCorrDist = 0.1;
	int gicp_maxIterations = 300;
	int normest_Ksearch = 50;

	Eigen::Matrix4f result;
	result(0,0) = 0; result(0,1) = 0; result(0,2) = 0; result(0,3) = 0;
	result(1,0) = 0; result(1,1) = 0; result(1,2) = 0; result(1,3) = 0;
	result(2,0) = 0; result(2,1) = 0; result(2,2) = 0; result(2,3) = 0;
	result(3,0) = 0; result(3,1) = 0; result(3,2) = 0; result(3,3) = 0;

	//std::cout << "result:" << std::endl;
	//std::cout << result << std::endl;

	ROS_INFO("Computing surface normals!");
	try{

		if(src_cloud->points.size() > 0 && target_cloud->points.size() > 0)
		{
			std::vector<int> indices1;
			pcl::removeNaNFromPointCloud(*src_cloud,*src_cloud,indices1);

			std::vector<int> indices2;
			pcl::removeNaNFromPointCloud(*target_cloud,*target_cloud,indices2);

			// Compute surface normals and curvature
			PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
			PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

			pcl::NormalEstimation<PointT, PointNormalT> norm_est;
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
			norm_est.setSearchMethod (tree);
			norm_est.setKSearch (normest_Ksearch);
			ROS_INFO("Finish initializing normal estimation!");

			norm_est.setInputCloud (src_cloud);
			ROS_INFO("src_cloud!");
			norm_est.compute (*points_with_normals_src);
			pcl::copyPointCloud (*src_cloud, *points_with_normals_src);
			ROS_INFO("Finish normal estimation for src_cloud!");

			norm_est.setInputCloud (target_cloud);
			norm_est.compute (*points_with_normals_tgt);
			pcl::copyPointCloud (*target_cloud, *points_with_normals_tgt);
			ROS_INFO("Finish normal estimation for target_cloud!");


			ROS_INFO("Refining the alignment of the point clouds using GICP...");
			pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
			ROS_INFO("Setting Euclidean Fitness Epsilon ...");
			gicp.setEuclideanFitnessEpsilon(gicp_FitnessEpsilon);
			ROS_INFO("Setting Max Correspondence Distance ...");
			gicp.setMaxCorrespondenceDistance(gicp_MaxCorrDist);
			ROS_INFO("Setting Maximum Iterations ...");
			gicp.setMaximumIterations(gicp_maxIterations);
			ROS_INFO("Setting input cloud ...");
			gicp.setInputCloud(src_cloud);//(trans_cloud);
			ROS_INFO("Setting target cloud ...");
			gicp.setInputTarget(target_cloud);

			pcl::PointCloud<pcl::PointXYZRGB> Final;
			ROS_INFO("Aligning point cloud ...");
			gicp.align(Final);

			std::cout << "has converged:" << gicp.hasConverged() << " score: " <<
					gicp.getFitnessScore() << std::endl;
			std::cout << gicp.getFinalTransformation() << std::endl;

			// apply transform to data cloud (to fit model cloud)
			// pcl::transformPointCloud(Final, *result, gicp.getFinalTransformation());

			//pcl::copyPointCloud(Final,*result);
			result = gicp.getFinalTransformation();

		}

	}catch(pcl::PCLException &e){

		std::cout << "PCL error in gicp: " << e.what() << std::endl;

	}

	return result;

}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
Eigen::Matrix4f RegistrationNode::Registre_views_ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_cloud)
{
	double icp_FitnessEpsilon = 1e-6;
	double icp_TransformationEpsilon = 1e-6;
	double icp_MaxCorrDist = 0.1;
	int icp_maxIterations = 300;
	int normest_Ksearch = 5;

	Eigen::Matrix4f result;

	result(0,0) = 0; result(0,1) = 0; result(0,2) = 0; result(0,3) = 0;
	result(1,0) = 0; result(1,1) = 0; result(1,2) = 0; result(1,3) = 0;
	result(2,0) = 0; result(2,1) = 0; result(2,2) = 0; result(2,3) = 0;
	result(3,0) = 0; result(3,1) = 0; result(3,2) = 0; result(3,3) = 0;


	if(src_cloud->points.size() > 0 && target_cloud->points.size() > 0)
	{
		std::vector<int> indices1;
		pcl::removeNaNFromPointCloud(*src_cloud,*src_cloud,indices1);

		std::vector<int> indices2;
		pcl::removeNaNFromPointCloud(*target_cloud,*target_cloud,indices2);

		// Compute surface normals and curvature
		PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
		PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

		pcl::NormalEstimation<PointT, PointNormalT> norm_est;
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
		norm_est.setSearchMethod (tree);
		norm_est.setKSearch (normest_Ksearch);

		norm_est.setInputCloud (src_cloud);
		norm_est.compute (*points_with_normals_src);
		pcl::copyPointCloud (*src_cloud, *points_with_normals_src);

		norm_est.setInputCloud (target_cloud);
		norm_est.compute (*points_with_normals_tgt);
		pcl::copyPointCloud (*target_cloud, *points_with_normals_tgt);

		// Instantiate our custom point representation (defined above) ...
		MyPointRepresentation point_representation;
		// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
		float alpha[4] = {1.0, 1.0, 1.0, 1.0};
		point_representation.setRescaleValues (alpha);

		// Align
		pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
		reg.setTransformationEpsilon (icp_TransformationEpsilon);

		// Set the maximum distance between two correspondences (src<->tgt) to 10cm
		// Note: adjust this based on the size of your datasets
		reg.setMaxCorrespondenceDistance (icp_MaxCorrDist);
		// Set the point representation
		reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

		reg.setInputCloud (points_with_normals_src);
		reg.setInputTarget (points_with_normals_tgt);

		//
		  // Run the same optimization in a loop and visualize the results
		  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
		  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
		  reg.setMaximumIterations (2);

		  for (int i = 0; i < 30; ++i)
		  {
			  PCL_INFO ("Iteration Nr. %d.\n", i);

			  // Estimate
			  reg.setInputCloud (points_with_normals_src);
			  reg.align (*reg_result);

			  //accumulate transformation between each Iteration
			  Ti = reg.getFinalTransformation () * Ti;

			  //if the difference between this transformation and the previous one
			  //is smaller than the threshold, refine the process by reducing
			  //the maximal correspondence distance
			  if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
		      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

		      prev = reg.getLastIncrementalTransformation ();


		  }

		  std::cout << "has converged:" << reg.hasConverged() << " score: " <<
		  					reg.getFitnessScore() << std::endl;
		  std::cout << Ti << std::endl;
		  result = Ti;
			//
		  // Get the transformation from target to source
		//  targetToSource = Ti.inverse();
	}

	return result;
 }


pcl::PointCloud<pcl::Normal>::Ptr RegistrationNode::EstimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius)
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (radius);

	// Compute the features
	ne.compute (*cloud_normals);
	return cloud_normals;
}

void RegistrationNode::removePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_cloud)
{
   //*********************************************************************//
   //	Plane fitting 
   /**********************************************************************/

   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
   // Create the segmentation object
   pcl::SACSegmentation<pcl::PointXYZRGB> seg;
   // Optional
   seg.setOptimizeCoefficients (true);
   // Mandatory
   seg.setModelType (pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setDistanceThreshold (0.005);
 
   seg.setInputCloud (src_cloud);
   seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
   {
     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
     //return (-1);
   }

   std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                       << coefficients->values[1] << " "
                                       << coefficients->values[2] << " "
                                       << coefficients->values[3] << std::endl;

  
   //*********************************************************************//
   //	Extract Indices
   /**********************************************************************/

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // Extract the inliers
  extract.setInputCloud (src_cloud);
  extract.setIndices(inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_f);

  pcl::copyPointCloud(*cloud_f, *target_cloud);
  
}

} /* namespace perception */
