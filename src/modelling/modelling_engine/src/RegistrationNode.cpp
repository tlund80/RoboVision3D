/*
 * RegistrationNode.cpp
 *
 *  Created on: Aug 10, 2013
 *      Author: thomas
 */

#include "modelling_engine/RegistrationNode.h"

namespace modelling_engine {

RegistrationNode::RegistrationNode(boost::shared_ptr<SharedData> data_ptr, boost::shared_ptr<RosInterface> ros_ptr, bool removePlane, bool preProcess)  
	: _sharedData(data_ptr), 
	_rosInterface(ros_ptr),
	_doRemoveTable(removePlane),
	_doPreProcessCloud(preProcess){
	// TODO Auto-generated constructor stub
	_state = NOP;
	_count = 0;
	_reconstruct.reset(new ReconstructPointCloud());

}

RegistrationNode::~RegistrationNode() {
	// TODO Auto-generated destructor stub
	stop();
}

void RegistrationNode::start(void)
{

	stop_thread = false;

	pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr total_cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr result_cloud (new pcl::PointCloud<PointT>);
	
	//Create solid model from _cloud
	 pcl::PointCloud<pcl::PointNormal>::Ptr _normals(new pcl::PointCloud<pcl::PointNormal>);
	  
	pcl::PointCloud<PointT>::Ptr cloud;
	while(!stop_thread)
	{
		//DTI_INFO("Hello from Registration thread!");
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));

		_state = _rosInterface->getScanningState();

		switch(_state){

		case INIT_REGISTRATION: //Initiale pose refinement. Using the Robot pose
			{

				if(_sharedData->getPointcloudListLength() > 0)
				{
						DTI_INFO("3D Modeling Engine: Initial registration!");
						
						// Get latest point cloud
						boost::shared_ptr<ScanData> d = _sharedData->getNewestScanData();
						cloud = d->getPointCloud();
						if(_doRemoveTable)removePlane(cloud,cloud);
						if(_doPreProcessCloud)statisticalOutlierRemoval(cloud,0.5);
					
						//Get transform
						Eigen::Matrix4f T; d->getTransform(T);
		
						// Make initial registration
						transformCloud(cloud,transformed_cloud, T);
						//pcl::copyPointCloud(*cloud,*transformed_cloud);

					/*	stringstream str;
						str << "/home/thomas/leap_box/pp_";
						str << _count;
						str << ".pcd";
						string s = str.str();
					
						DTI_INFO("3D Modeling Engine: Saving cloud at " << s);
						pcl::io::savePCDFile(s,*transformed_cloud);
					*/
						//_count++;
						
						_rosInterface->pubLastPointCloud(*transformed_cloud);
						
						//Jump to State REGISTRE
						//_state++;
						_rosInterface->setScanningState(REGISTRE);
						//_state = RECONSTRUCT;
				}
			}
			break;

		case REGISTRE:
			{
			DTI_INFO("3D Modeling Engine: Running Registration!!");
			
				//Align model and latest scan
				if(_sharedData->getPointcloudListLength() > 1)
				{
				      //Get model
				      pcl::PointCloud<PointT>::Ptr model = _sharedData->getModelPointcloud();
				      
				      std::cout << "Model point Cloud size: " << (int)model->size() << std::endl;

				      std::vector<int> indices1;
				      pcl::removeNaNFromPointCloud(*cloud,*cloud,indices1);

				      std::vector<int> indices2;
				      pcl::removeNaNFromPointCloud(*model,*model,indices2);

				      if(model->points.size() > 0){

					    //DownSample Cloud before registration
					    pcl::PointCloud<PointT>::Ptr cloud_sampled = DownSampleCloud(transformed_cloud,0.001); //2.5
					    pcl::PointCloud<PointT>::Ptr model_sampled = DownSampleCloud(model,0.001);

					   // checkForNaN(cloud);
					    //checkForNaN(model);

					    //Eigen::Matrix4f mat = Registre_views(cloud_sampled,model_sampled);
					    Eigen::Matrix4f mat, initial_mat;
					    //initial_alignment(cloud_sampled,model_sampled,initial_mat,false);
					    //transformCloud(cloud_sampled,cloud_sampled,mat);
					    if(Registre_views_ICP(cloud_sampled,model_sampled, mat))
					    {
					      DTI_INFO("Aligning Point Cloud with the model!!");
					    //  transformCloud(cloud,result_cloud,initial_mat);
					      transformCloud(transformed_cloud,result_cloud,mat);
					      
					      //Add models
					      *total_cloud = *result_cloud;
					      //*total_cloud = *transformed_cloud;
					      *total_cloud += *model;
					      
					      //use voxelGrid to remove redundant points
					     // _reconstruct->MLSApproximation(total_cloud,total_cloud);
					       total_cloud = DownSampleCloud(total_cloud,0.0008);
					      
					      
					    }
				      }
				      
				        _rosInterface->setScanningState(NOP);
					 _state = NOP;
					
				    //  _rosInterface->setScanningState(RECONSTRUCT);
				    //  _state = RECONSTRUCT;
				
					  
				}else{
			
					//Add first point cloud to the model
					DTI_INFO("First Model Point Cloud!!");
					*total_cloud = *transformed_cloud;
					
					_rosInterface->setScanningState(RECONSTRUCT);
				        _state = RECONSTRUCT;
				}

				//Save Model in SharedData space
				_sharedData->setModelPointcloud(total_cloud->makeShared());

				//Publish model
				_rosInterface->pubRegisteredModel(*total_cloud);
				
				robot_msgs::ModellingResult msg;
				msg.coverage = 0;
				msg.model_points = int(total_cloud->size());
				msg.scan_num_in_model = _sharedData->getPointcloudListLength();
				_rosInterface->pubFinishMsg(msg);

				
			}
		
			break;
			
		case RECONSTRUCT:
			DTI_INFO("3D Modeling Engine: Reconstruct model to a triangular mesh!!!");
  
				 if(total_cloud->isOrganized())
				 {
				   DTI_INFO("3D Modeling Engine: Point Cloud is organized!! Using OrganizedFastMesh method for reconstruction!");
				   
				 }else{
				    DTI_INFO("3D Modeling Engine: Point Cloud is not organized!! Using GreedyProjectionTriangulation method for reconstruction!");
				//   EstimateNormals(total_cloud, _normals, 50);
				   
				 //  std::cout << "Cloud with normals has: " << (int)_normals->size() << " points!!" << std::endl; 
				   
				    DTI_INFO("3D Modeling Engine: Reconstruction started......!");
				   //pcl::PolygonMesh mesh = _reconstruct->MarchingCubes(_normals,0.05,0.05,0.05,0.5);
				  //  pcl::PolygonMesh mesh = _reconstruct->GreedyProjectionTriangulation(_normals,0.05);
				    
				  // std::cout << "Size of mesh: " << mesh.cloud.width * mesh.cloud.height << std::endl;
				  // _reconstruct->saveToObj("danfoss_trafo.obj",mesh);
				   DTI_INFO("3D Modeling Engine: Reconstruction done......!");
				  
				 }
				 
			
				
			
				  _rosInterface->setScanningState(NOP);
				 _state = NOP;
			break;
			

		case REFINE:
			DTI_INFO("3D Modeling Engine: Refining mesh model!!");

			break;
			
		case FINALIZE:
			DTI_INFO("3D Modeling Engine: Finalize mesh model!!");

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
			//	std::cout << "ERROR!!!" << std::endl;
				cloud->at(i).x = 0;
				cloud->at(i).y = 0;
				cloud->at(i).z = 0;
				//cloud_sampled->points.erase(cloud_sampled->points.begin()+i);

			//	std::cout << " x: " << cloud->points[i].x
			//			  << " y: " << cloud->points[i].y
			//			  << " z: " << cloud->points[i].z
			//			  << " r: " << (int)cloud->points[i].r
			//			  << " g: " << (int)cloud->points[i].g
			//			  << " b: " << (int)cloud->points[i].b << std::endl;
		}
	}

}

void RegistrationNode::transformCloud(pcl::PointCloud<PointT>::Ptr &cloud,pcl::PointCloud<PointT>::Ptr &tar, Eigen::Matrix4f& transform)
{
	pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT>);

	try{
		pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
	}catch(pcl::PCLException &e)
	{
		DTI_ERROR("3D Modeling Engine: In RegistrationNode::transformCloud() " << e.what());
	}
	pcl::copyPointCloud(*transformed_cloud,*tar);

}

pcl::PointCloud<PointT>::Ptr RegistrationNode::statisticalOutlierRemoval(pcl::PointCloud<PointT>::Ptr cloud, int mean)
{
	 pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (mean);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);

	return cloud_filtered;
}

pcl::PointCloud<PointT>::Ptr RegistrationNode::DownSampleCloud(pcl::PointCloud<PointT>::Ptr cloud, double leaf_size)
{
	 std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
	       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

	 pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	// Create the filtering object
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (leaf_size,leaf_size,leaf_size);
	sor.filter (*cloud_filtered);

	 std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
	       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

	return cloud_filtered;
}

pcl::PointCloud<PointT>::Ptr  RegistrationNode::PassThroughFilter(pcl::PointCloud<PointT>::Ptr cloud, double min_depth, double max_depth)
{
	// Preprocess the cloud by...
	// ...removing distant points
	//const float depth_limit = 1.0;

	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (min_depth, max_depth);
	pass.filter (*cloud);

	return cloud;
}

bool RegistrationNode::Registre_views(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_cloud, Eigen::Matrix4f& transform)
{
	double gicp_FitnessEpsilon = 1e-6;
	double gicp_MaxCorrDist = 0.1;
	int gicp_maxIterations = 300;
	int normest_Ksearch = 50;

/*	Eigen::Matrix4f result;
	result(0,0) = 0; result(0,1) = 0; result(0,2) = 0; result(0,3) = 0;
	result(1,0) = 0; result(1,1) = 0; result(1,2) = 0; result(1,3) = 0;
	result(2,0) = 0; result(2,1) = 0; result(2,2) = 0; result(2,3) = 0;
	result(3,0) = 0; result(3,1) = 0; result(3,2) = 0; result(3,3) = 0;
*/
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
			gicp.setInputSource(src_cloud);//(trans_cloud);
			ROS_INFO("Setting target cloud ...");
			gicp.setInputTarget(target_cloud);

			pcl::PointCloud<pcl::PointXYZRGB> Final;
			ROS_INFO("Aligning point cloud ...");
			gicp.align(Final);

			if(gicp.hasConverged())
			{
			  std::cout << "has converged:" << gicp.hasConverged() << " score: " <<
					gicp.getFitnessScore() << std::endl;
			  std::cout << gicp.getFinalTransformation() << std::endl;

			  transform = gicp.getFinalTransformation();
			  return true;
			}else
			  return false;

		}

	}catch(pcl::PCLException &e){

		std::cout << "PCL error in gicp: " << e.what() << std::endl;

	}
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
bool RegistrationNode::Registre_views_ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_cloud, Eigen::Matrix4f& transform)
{
	double icp_FitnessEpsilon = 1e-6;
	double icp_TransformationEpsilon = 1e-6;
	double icp_MaxCorrDist = 0.1;
	int icp_maxIterations = 300;
	int normest_Ksearch = 50;

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

		reg.setInputSource (points_with_normals_src);
		reg.setInputTarget (points_with_normals_tgt);

		//reg.setEuclideanFitnessEpsilon(icp_FitnessEpsilon);
		//
		  // Run the same optimization in a loop and visualize the results
		  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
		  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
		  reg.setMaximumIterations (icp_maxIterations);

		  for (int i = 0; i < 30; ++i)
		  {
			  PCL_INFO ("Iteration Nr. %d.\n", i);

			  // Estimate
			  reg.setInputSource (points_with_normals_src);
			  reg.align (*reg_result);

			  //accumulate transformation between each Iteration
			  Ti = reg.getFinalTransformation () * Ti;

			  //if the difference between this transformation and the previous one
			  //is smaller than the threshold, refine the process by reducing
			  //the maximal correspondence distance
			  if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ()){
			    double max = reg.getMaxCorrespondenceDistance () - 0.001;
			    PCL_INFO ("Old MaxCorrespondenceDistance: %f\n", reg.getMaxCorrespondenceDistance ());     
			    
			 //   if(max > 0.0){
			      PCL_INFO ("Decreasing MaxCorrespondenceDistance: %f\n", max);
			      reg.setMaxCorrespondenceDistance (max);
			  //  }
			  }

		      prev = reg.getLastIncrementalTransformation ();


		  }

		  if(reg.hasConverged())
		  {
		      std::cout << "has converged:" << reg.hasConverged() << " score: " <<
		  					reg.getFitnessScore() << std::endl;
		      std::cout << Ti << std::endl;
		      transform = Ti;
		      return true;
		  }else
		      return false;
	}

	return false;
 }

void RegistrationNode::initial_alignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_cloud, Eigen::Matrix4f &final_transform, bool downsample)
{
	  float max_correspondence_distance_ = 5.0;
	  int nr_iterations_ =  500;
	  float min_sample_distance_ = 0.005;


	  // Downsample for consistency and speed
	    // \note enable this for large datasets
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
	    if (downsample)
	    {
	      grid.setLeafSize (0.005, 0.005, 0.005);
	      grid.setInputCloud (src_cloud);
	      grid.filter (*src);

	      grid.setInputCloud (target_cloud);
	      grid.filter (*tgt);

	      std::cout << "Filtered cloud contains " << src->size () << " data points" << std::endl;
	    }
	    else
	    {
	      src = src_cloud;
	      tgt = target_cloud;
	    }

	    
	    int normest_Ksearch = 20;
	    pcl::console::print_info("Computing Surface Normales!\n");

	    pcl::PointCloud<pcl::Normal>::Ptr norm_src (new pcl::PointCloud<pcl::Normal>);
	    pcl::PointCloud<pcl::Normal>::Ptr norm_tgt (new pcl::PointCloud<pcl::Normal>);

	    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
	    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	    norm_est.setSearchMethod (tree);
	    norm_est.setKSearch (normest_Ksearch);

	    norm_est.setInputCloud (src);
	    norm_est.compute (*norm_src);
	    std::cout << "Computed normals: " << norm_src->size () << std::endl;
	    		//pcl::copyPointCloud (*src_sampled, *points_with_normals_src);
   	    norm_est.setInputCloud (tgt);
   	    norm_est.compute (*norm_tgt);
	    std::cout << "Computed normals: " << norm_tgt->size () << std::endl;

   	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_src (new pcl::PointCloud<pcl::PointXYZRGB>);
   	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_tgt (new pcl::PointCloud<pcl::PointXYZRGB>);

	    PCL_INFO ("FPFH - started\n");
	    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> pfh_est_src;
	    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_pfh_src (new pcl::search::KdTree<pcl::PointXYZRGB>());
	    pfh_est_src.setSearchMethod (tree_pfh_src);
	    pfh_est_src.setKSearch(20);
	    //pfh_est_src.setSearchSurface (keypoints_src);
	    pfh_est_src.setInputNormals (norm_src);
	    // pfh_est_src.setInputCloud (src_sampled);
	    pfh_est_src.setInputCloud (src);

	    pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_src (new pcl::PointCloud<pcl::FPFHSignature33>);
	    PCL_INFO ("	FPFH - Compute Source\n");
	    pfh_est_src.compute (*pfh_src);
	    PCL_INFO ("	FPFH - finished\n");

	    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> pfh_est_tgt;
	    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_pfh_tgt (new pcl::search::KdTree<pcl::PointXYZRGB>());
	    pfh_est_tgt.setSearchMethod (tree_pfh_tgt);
	    pfh_est_tgt.setKSearch(20);
	  //  pfh_est_tgt.setSearchSurface (tgt);
	    pfh_est_tgt.setInputNormals (norm_tgt);
	    // pfh_est_tgt.setInputCloud (tar_sampled);
	    pfh_est_tgt.setInputCloud (tgt);

	    pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_tgt (new pcl::PointCloud<pcl::FPFHSignature33>);
	    PCL_INFO ("	FPFH - Compute Target\n");
	    pfh_est_tgt.compute (*pfh_tgt);
	    PCL_INFO ("	FPFH - finished\n");


	  pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia_;

	  // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm

	  sac_ia_.setMinSampleDistance (min_sample_distance_);
	  sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
	  sac_ia_.setMaximumIterations (nr_iterations_);
	  sac_ia_.setInputCloud (src);
	  sac_ia_.setSourceFeatures (pfh_src);
	  sac_ia_.setInputTarget (tgt);
	  sac_ia_.setTargetFeatures (pfh_tgt);

	  pcl::PointCloud<pcl::PointXYZRGB> registration_output;
	  sac_ia_.align (registration_output);


	  float fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
	  // Print the alignment fitness score (values less than 0.00002 are good)
	  printf ("Best fitness score: %f\n", fitness_score);
	  Eigen::Matrix4f f_transformation = sac_ia_.getFinalTransformation ();

	  std::cout << f_transformation << std::endl;

	  final_transform = f_transformation;
//	  pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
	  //cloudNext is my target cloud
//	  pcl::transformPointCloud(*tar,cloud_transformed,final_transformation);
	  
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

void RegistrationNode::EstimateNormals(pcl::PointCloud<PointT>::Ptr &src_cloud,pcl::PointCloud<pcl::PointNormal>::Ptr &target_cloud, double neighbors)
{
	// Normal estimation*
	pcl::NormalEstimation<PointT, pcl::PointNormal> n;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
		 
	tree->setInputCloud (src_cloud);
	n.setInputCloud (src_cloud);
	n.setSearchMethod (tree);
	n.setKSearch(neighbors);
	n.compute (*cloud_with_normals);
	
	PCL_INFO("Finish normal estimation for model. Size: %d\n", (int)cloud_with_normals->size());
	
	pcl::copyPointCloud(*cloud_with_normals, *target_cloud);
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
