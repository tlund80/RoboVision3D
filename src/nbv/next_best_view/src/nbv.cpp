/*
 * nbv.cpp
 *
 *  Created on: Jun 20, 2013
 *      Author: thomas
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

//Octomap includes
#include <octomap/octomap.h>
//octomap_msgs
//#include <octomap_ros/OctomapBinary.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

// OctoMap_ros includes
//#include <octomap_ros/conversions.h>

//PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/features/boundary.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/console/print.h>
#include <pcl/console/time.h>

//pcl_ros include
#include <pcl_ros/publisher.h>

#include <vector>
#include <boost/foreach.hpp>

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointXYZRGBNormal PointNormalRGBT;

class Nbv
{
protected:
  ros::NodeHandle nh_;

  //parameters
  std::string input_cloud_topic_;
  std::string octomap_topic_;
  std::string output_octree_topic_;
  std::string output_pose_topic_;
  std::string sensor_frame_; //name of tf sensor frame

  double leaf_size_, octree_res_, octree_maxrange_;
  int level_, free_label_, occupied_label_, unknown_label_;
  bool check_centroids_;
  bool visualize_octree_;
  bool visualize_free_cells_;
  bool visualize_unknown_cells_;

  double normal_search_radius_;
  int min_pts_per_cluster_;
  double eps_angle_;
  double tolerance_;
  double boundary_angle_threshold_;
  double boundary_est_radius_;
  
  octomap::OcTree* map;

  //objects needed
  tf::TransformListener tf_listener_;
  
  //datasets
  octomap::OcTree* octree_;
  octomap::ScanGraph* octomap_graph_;
  octomap::KeyRay ray;

  sensor_msgs::PointCloud2 cloud_in_;
  geometry_msgs::PoseArray nbv_pose_array_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber octomap_sub_;
  ros::Publisher octree_pub_;
  pcl_ros::Publisher<PointNormalRGBT> border_cloud_pub_;
  pcl_ros::Publisher<PointT> cluster_cloud_pub_;
  pcl_ros::Publisher<PointT> cluster_cloud2_pub_;
  pcl_ros::Publisher<PointT> cluster_cloud3_pub_;
  ros::Publisher pose_pub_;
  
  // Publishes the octree in MarkerArray format so that it can be visualized in rviz
  ros::Publisher octree_marker_array_publisher_;
  /* The following publisher, even though not required, is used because otherwise rviz
   * cannot visualize the MarkerArray format without advertising the Marker format*/
  ros::Publisher octree_marker_publisher_;
  // Marker array to visualize the octree. It displays the occuplied cells of the octree
  visualization_msgs::MarkerArray octree_marker_array_msg_;


  static bool compareClusters(pcl::PointIndices c1, pcl::PointIndices c2) { return (c1.indices.size() < c2.indices.size()); }

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg);
  void octomap_cb (const octomap_msgs::Octomap::Ptr& msg);
  void createOctree (pcl::PointCloud<PointT>& pointcloud2_pcl,  octomath::Pose6D sensor_pose);
  void findBorderPoints(pcl::PointCloud<PointT>& border_cloud, std::string frame_id);
  void findBorderPoints2(const pcl::PointCloud<PointT>::Ptr &input_points, pcl::PointCloud<pcl::Normal>::Ptr &input_normals, pcl::PointCloud<PointT>::Ptr &output);
  void computeBoundaryPoints(pcl::PointCloud<PointT>& border_cloud, pcl::PointCloud<pcl::Normal>& border_normals, 
			      std::vector<pcl::PointIndices>& clusters, pcl::PointCloud<PointT>* cluster_clouds);
  void visualizeOctree(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg, geometry_msgs::Point viewpoint);
  void castRayAndLabel(pcl::PointCloud<PointT>& cloud, octomap::pose6d origin);
  
public:
  Nbv(ros::NodeHandle &anode);
  ~Nbv();
};

Nbv::Nbv (ros::NodeHandle &anode) : nh_(anode) {

  //Initialize parameters
  nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/cloudIn"));
  nh_.param("octomap_topic", octomap_topic_, std::string("/octomapIn"));
  nh_.param("output_pose_topic", output_pose_topic_, std::string("/nbv_pose"));
   nh_.param("output_octree_topic", output_octree_topic_, std::string("/nbv_octree"));
  nh_.param("sensor_frame", sensor_frame_, std::string("/sensor_frame"));
  nh_.param("leaf_size", leaf_size_, 0.0005);
  nh_.param("octree_resolution", octree_res_, 0.0025);

  nh_.param("octree_maxrange", octree_maxrange_, -1.0);
  nh_.param("level", level_, 0);
  nh_.param("check_centroids", check_centroids_, false);
  nh_.param("free_label", free_label_, 0);
  nh_.param("occupied_label", occupied_label_, 1);
  nh_.param("unknown_label", unknown_label_, -1);
  nh_.param("visualize_octree", visualize_octree_, true);
  nh_.param("visualize_free_cells", visualize_free_cells_, true);
  nh_.param("visualize_unknown_cells", visualize_unknown_cells_, true);

  nh_.param("normal_search_radius", normal_search_radius_, 0.005);
  nh_.param("min_pts_per_cluster", min_pts_per_cluster_, 10);
  nh_.param("eps_angle", eps_angle_, 0.25);
  nh_.param("tolerance", tolerance_, 0.03);
  nh_.param("boundary_angle_threshold", boundary_angle_threshold_, 0.785);
  nh_.param("boundary_est_radius", boundary_est_radius_, 0.0005);

  //Subscribe to pointcloud topic
   cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &Nbv::cloud_cb, this);
   octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &Nbv::octomap_cb, this);

  //TODO:
  //Advertise publisher
  octree_pub_ = nh_.advertise<octomap_msgs::Octomap> (output_octree_topic_, 1);
  border_cloud_pub_ = pcl_ros::Publisher<PointNormalRGBT> (nh_, "border_cloud", 1);
  cluster_cloud_pub_ = pcl_ros::Publisher<PointT> (nh_, "cluster_cloud", 1);
  cluster_cloud2_pub_ = pcl_ros::Publisher<PointT> (nh_, "cluster_cloud2", 1);
  cluster_cloud3_pub_ = pcl_ros::Publisher<PointT> (nh_, "cluster_cloud3", 1);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseArray> (output_pose_topic_, 1);
  
  octree_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
  octree_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);


}

Nbv::~Nbv()
{
  ROS_INFO("Shutting down next_best_view node!");
}


void Nbv::octomap_cb(const octomap_msgs::Octomap::Ptr& msg)
{
   map = octomap_msgs::binaryMsgToMap(*msg);
  ROS_INFO("Received octomap with resolution = %f, TreeType = %s and Number of Leaf nodes= %d ", map->getResolution(),map->getTreeType().c_str(), (int)map->getNumLeafNodes());
  

/*   for(octomap::OcTree::leaf_iterator it = map->begin_leafs(),end=map->end_leafs(); it!= end; ++it)
  {
    octomap::OcTreeNode node = *it; 
   
      //manipulate node, e.g.:
     // std::cout << "Node center: " << it.getCoordinate() << std::endl;
     // std::cout << "Node size: " << it.getSize() << std::endl;
      std::cout << "Node value: " << node.getValue() << std::endl;
      std::cout << "Node Occupancy: " << node.getOccupancy() << std::endl;
      std::cout << "Node Log Odds: " << node.getLogOdds() << std::endl;
 
    

  }
  */
}

/**
* \brief cloud callback and the core filtering function at the same time
* \param pointcloud2_msg input point cloud to be processed
*/
void Nbv::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg) {

  pcl::PointCloud<PointT>::Ptr pointcloud2_pcl (new  pcl::PointCloud<PointT>());

  ROS_INFO("Received point cloud");

  //get the latest parameters
  nh_.getParam("normal_search_radius", normal_search_radius_);
  nh_.getParam("min_pts_per_cluster", min_pts_per_cluster_);
  nh_.getParam("eps_angle", eps_angle_);
  nh_.getParam("tolerance", tolerance_);
  nh_.getParam("leaf_size", leaf_size_);
  nh_.getParam("boundary_angle_threshold", boundary_angle_threshold_);
  nh_.getParam("boundary_est_radius", boundary_angle_threshold_);


  //Get the viewpoint from the robot!!
  tf::StampedTransform transform;
  try {
    ros::Time acquisition_time = pointcloud2_msg->header.stamp;
    ros::Duration timeout(1.0 / 30);
    tf_listener_.waitForTransform(pointcloud2_msg->header.frame_id, sensor_frame_, acquisition_time, timeout);
    tf_listener_.lookupTransform(pointcloud2_msg->header.frame_id, sensor_frame_, acquisition_time, transform);
  }
  catch (tf::TransformException& ex) {
    ROS_WARN("[next_best_view] TF exception:\n%s", ex.what());
  }
  tf::Point pt = transform.getOrigin();
  tf::Quaternion rot = transform.getRotation();
  
  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(rot).getRPY(roll, pitch, yaw);
  octomap::pose6d sensor_pose (pt.x(), pt.y(), pt.z(),roll,pitch,yaw);

  ROS_INFO("viewpoint [%f %f %f %f %f %f]", pt.x(), pt.y(), pt.z(), roll, pitch, yaw);

  //Converting PointCloud2 msg format to pcl pointcloud format in order to read the 3d data
  pcl::fromROSMsg(*pointcloud2_msg, *pointcloud2_pcl);
  
  pcl::VoxelGrid<PointT> sor;
  //voxel grid filter
  sor.setInputCloud (pointcloud2_pcl);
  sor.setLeafSize (leaf_size_,leaf_size_,leaf_size_);
  sor.filter (*pointcloud2_pcl);
  
  std::cout << "Cloud contains " << pointcloud2_pcl->size() <<  " points after Voxel Grid filtering" << std::endl;
  
  
  // create or update the octree
  if (octree_) {
     ROS_INFO("Creating OcTree!!!");
    octomap_graph_ = new octomap::ScanGraph();
    octree_ = new octomap::OcTree(octree_res_);
  }
  
   createOctree(*pointcloud2_pcl, sensor_pose);

  /*
   * assign new points to Leaf Nodes  and cast rays from sensor pos to point
   */
  castRayAndLabel(*pointcloud2_pcl, sensor_pose);
 
 /*
   * find unknown voxels with free neighbors and add them to a pointcloud
   */
  pcl::PointCloud<PointT> border_cloud;
  findBorderPoints(border_cloud, pointcloud2_msg->header.frame_id);

  // Create the filtering objects
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> nextract;
  pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
  
  //creating datasets
  pcl::PointCloud<pcl::Normal>::Ptr border_normals (new  pcl::PointCloud<pcl::Normal>());
  pcl::PointCloud<PointNormalRGBT>::Ptr cloud_PointNormal(new  pcl::PointCloud<PointNormalRGBT>()); ;

  // tree object used for search
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(border_cloud.makeShared());
  ne.setRadiusSearch(normal_search_radius_);
  //ne.setKSearch(50);
  ne.compute(*border_normals);

  //filter again to remove spurious NaNs
  pcl::PointIndices nan_indices;

  for (unsigned int i = 0; i < border_normals->points.size(); i++) {
    if (isnan(border_normals->points[i].normal[0]))
      nan_indices.indices.push_back(i);
  }
  ROS_INFO("%d NaNs found", (int)nan_indices.indices.size());
  //in pointcloud

  extract.setInputCloud(border_cloud.makeShared());
  extract.setIndices(boost::make_shared<pcl::PointIndices> (nan_indices));
  extract.setNegative(true);
  extract.filter(border_cloud);
  ROS_INFO("%d points in border cloud after filtering and NaN removal", (int)border_cloud.points.size());
  //and in the normals
  nextract.setInputCloud(border_normals);
  nextract.setIndices(boost::make_shared<pcl::PointIndices> (nan_indices));
  nextract.setNegative(true);
  nextract.filter(*border_normals);
  ROS_INFO("%d points in normals cloud after NaN removal", (int)border_normals->points.size());
  
  // tree object used for search
  pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT> ());
  tree2->setInputCloud (boost::make_shared<pcl::PointCloud<PointT> > (border_cloud));
  // Decompose a region of space into clusters based on the euclidean distance between points, and the normal
  
   std::vector<pcl::PointIndices> clusters;
   pcl::EuclideanClusterExtraction<PointT> ec;
   ec.setClusterTolerance (tolerance_); // 2cm
   ec.setMinClusterSize (min_pts_per_cluster_);
   ec.setMaxClusterSize (60000);
   ec.setSearchMethod (tree2);
   ec.setInputCloud(border_cloud.makeShared());
   ec.extract (clusters); 
   
   ROS_INFO("%d clusters found", (int)clusters.size());
    
   /*
    * compute boundary points from clusters and put them into pose array
    */
   pcl::PointCloud<PointT> cluster_clouds[3];
   computeBoundaryPoints(border_cloud, *border_normals, clusters, cluster_clouds);
    
   //publish poses
   pose_pub_.publish(nbv_pose_array_);
 
   //publish border cloud for visualization
   pcl::PointCloud<PointNormalRGBT> border_pn_cloud;
   if (border_cloud.points.size() > 0) {
     pcl::concatenateFields(border_cloud, *border_normals, border_pn_cloud);
     } else {
       border_pn_cloud.header.frame_id = border_cloud.header.frame_id;
       border_pn_cloud.header.stamp = ros::Time::now().toNSec();
       
    }
    border_cloud_pub_.publish(border_pn_cloud);	
    
    //publish the clusters for visualization
    for (unsigned int i = 0; i < 3; i++) {
     if (cluster_clouds[i].points.size() == 0) {
       cluster_clouds[i].header.frame_id = border_cloud.header.frame_id;
       cluster_clouds[i].header.stamp = ros::Time::now().toNSec();  
       }
    }
    
    cluster_cloud_pub_.publish(cluster_clouds[0]);
    cluster_cloud2_pub_.publish(cluster_clouds[1]);
    cluster_cloud3_pub_.publish(cluster_clouds[2]);
    
    // publish binary octree
   if (0) {
      octomap_msgs::Octomap octree_msg;
      octomap_msgs::binaryMapToMsg(*octree_, octree_msg);
      octree_pub_.publish(octree_msg);
    }
    
   ROS_INFO("All computed and published");

   //**********************************************************************************
   //Visualization	 
   //**********************************************************************************	  
   if (visualize_octree_){
     geometry_msgs::Point viewpoint;
     viewpoint.x = pt.x();
     viewpoint.y = pt.y();
     viewpoint.z = pt.z();
     visualizeOctree(pointcloud2_msg, viewpoint);
     
  }

}

void Nbv::computeBoundaryPoints(pcl::PointCloud<PointT>& border_cloud, pcl::PointCloud<pcl::Normal>& border_normals,
				std::vector<pcl::PointIndices>& clusters, pcl::PointCloud<PointT>* cluster_clouds)
{
	  //clear old poses
	  nbv_pose_array_.poses.clear();
	
	  if (clusters.size() > 0) {
	    ROS_INFO ("%d clusters found.", (int)clusters.size());
	    // sort the clusters according to number of points they contain
	    std::sort(clusters.begin(), clusters.end(), compareClusters);
	
	    pcl::ExtractIndices<PointT> extract;
	    pcl::ExtractIndices<pcl::Normal> nextract;
	
	    for (unsigned int nc = 0; nc < clusters.size(); nc++) {
	      //extract maximum of 3 biggest clusters
	      if (nc == 3)
	    break;
	
	      //extract a cluster
	      pcl::PointCloud<PointT> cluster_cloud;
	      extract.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (border_cloud));
	      extract.setIndices(boost::make_shared<pcl::PointIndices> (clusters.back()));
	      extract.setNegative(false);
	      extract.filter(cluster_cloud);
	      ROS_INFO ("PointCloud representing the cluster %d: %d data points.", nc, cluster_cloud.width * cluster_cloud.height);
	
	      //extract normals of cluster
	      pcl::PointCloud<pcl::Normal> cluster_normals;
	      nextract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::Normal> > (border_normals));
	      nextract.setIndices(boost::make_shared<pcl::PointIndices> (clusters.back()));
	      nextract.setNegative(false);
	      nextract.filter(cluster_normals);
	
	      // find boundary points of cluster
	      pcl::search::KdTree<PointT>::Ptr tree3 (new pcl::search::KdTree<PointT>());
	      pcl::PointCloud<pcl::Boundary> boundary_cloud;
	      pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> be;
	      be.setSearchMethod(tree3);
	      be.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cluster_cloud));
	      be.setInputNormals(boost::make_shared<pcl::PointCloud<pcl::Normal> > (cluster_normals));
	      be.setRadiusSearch(.8);
	      be.setAngleThreshold(boundary_angle_threshold_);
	      be.compute(boundary_cloud);
	
	      geometry_msgs::Pose nbv_pose;
	      unsigned int nbp = 0;
	      for (unsigned int i = 0; i < boundary_cloud.points.size(); ++i) {
	    if (boundary_cloud.points[i].boundary_point) {
	      nbv_pose.position.x = cluster_cloud.points[i].x;
	      nbv_pose.position.y = cluster_cloud.points[i].y;
	      nbv_pose.position.z = cluster_cloud.points[i].z;
	      tf::Vector3 axis(0, -cluster_normals.points[i].normal[2], cluster_normals.points[i].normal[1]);
	      tf::Quaternion quat(axis, axis.length());
	     
	      if(!isnan(double(quat.w())) && !isnan(double(quat.x())) && !isnan(double(quat.y()))  && !isnan(double(quat.z())))
	      {
		geometry_msgs::Quaternion quat_msg;
		tf::quaternionTFToMsg(quat, quat_msg);
		nbv_pose.orientation = quat_msg;
		nbv_pose_array_.poses.push_back(nbv_pose);
		nbp++;
	      }
	    }
	      }
	      ROS_INFO ("%d boundary points in cluster %d.", nbp, nc);
	
	      //save this cluster pointcloud for visualization
	      cluster_clouds[nc] = cluster_cloud;
	
	      //pop the just used cluster from indices
	      clusters.pop_back();
	    }
	    nbv_pose_array_.header.frame_id = border_cloud.header.frame_id;
	    nbv_pose_array_.header.stamp = ros::Time::now();
	  }
	  else {
	    ROS_INFO ("No clusters found!");
	  }
}


void Nbv::findBorderPoints(pcl::PointCloud<PointT>& border_cloud, std::string frame_id) {
  border_cloud.header.frame_id = frame_id;
  border_cloud.header.stamp = ros::Time::now().toNSec();
 
  for(octomap::OcTree::leaf_iterator it = octree_->begin_leafs(),end=octree_->end_leafs(); it!= end; ++it)
  {
    octomap::point3d centroid = it.getCoordinate();
    octomap::OcTreeNode *octree_node = octree_->search(centroid);
    
    // if free voxel -> check for unknown neighbors
    if (octree_node  && octree_node->getValue() == occupied_label_) //free_label_)
    {
       // std::cout << "free voxel " << std::endl;
      for (int i=0; i<3; i++)
      {
    	  octomap::point3d neighbor_centroid = centroid;
    	  for (int j=-1; j<2; j+=2)
    	  {
    		  neighbor_centroid(i) += j * octree_res_;
    		  octomap::OcTreeNode *neighbor = octree_->search(neighbor_centroid);

		  //if (neighbor) std::cout << "neighbor value: " << neighbor->getValue() << std::endl;
    		  if (neighbor && (int)neighbor->getValue() == unknown_label_)
    		  {
    			  // add to list of border voxels
    			  PointT border_pt; 
			  border_pt.x = centroid.x();
			  border_pt.y = centroid.y();
			  border_pt.z = centroid.z();
			  border_pt.r= 0.0f;;
			  border_pt.g = 0.0f;
			  border_pt.b = 1.0f;
			  border_pt.a = 0.8f;
			// std::cout << "border_pt: " << border_pt << std::endl;
    			  border_cloud.points.push_back(border_pt);
    			  break;
    		  }
	}
      }
    }
  }
  ROS_INFO("%d points in border cloud", (int)border_cloud.points.size());
  
}


/**
* \brief Find the boarder point in the cloud
* \param pointcloud2_msg input point cloud to be processed
*/
void Nbv::findBorderPoints2(const pcl::PointCloud<PointT>::Ptr &input_points, pcl::PointCloud<pcl::Normal>::Ptr &input_normals, pcl::PointCloud<PointT>::Ptr &output)
{
  using namespace pcl::console;

  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ()); 
  pcl::PointCloud<PointT>::Ptr points(new  pcl::PointCloud<PointT>()); ;
  
  // Estimate
  TicToc tt;
  tt.tic ();
  
  print_highlight(stderr, "Estimating Point Cloud Boundaries ");

  pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> ne;
  ne.setInputCloud (input_points);
  ne.setInputNormals (input_normals);
  ne.setSearchMethod (tree);
  ne.setKSearch (20);
  ne.setAngleThreshold (1.57);
 
  //ne.setRadiusSearch (boundary_est_radius_);  
  
  pcl::PointCloud<pcl::Boundary> boundary_cloud;
  ne.compute (boundary_cloud);
  
 

  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", boundary_cloud.width * boundary_cloud.height); print_info (" points]\n");

  
   geometry_msgs::Pose nbv_pose;
      unsigned int nbp = 0;
      for (unsigned int i = 0; i < boundary_cloud.points.size(); ++i) {
	if (boundary_cloud.points[i].boundary_point) {
	  
	  
	/*  nbv_pose.position.x = input_points->points[i].x;
	  nbv_pose.position.y = input_points->points[i].y;
	  nbv_pose.position.z = input_points->points[i].z;
	*/  
	//  points->points[i].x = input_points->points[i].x;
	//  points->points[i].y = input_points->points[i].y;
	//  points->points[i].z = input_points->points[i].z;
	 /*   
	  tf::Vector3 axis(0, -input_normals->points[i].normal[2], input_normals->points[i].normal[1]);
	  tf::Quaternion quat(axis, axis.length());

	  geometry_msgs::Quaternion quat_msg;
	  tf::quaternionTFToMsg(quat, quat_msg);
	  nbv_pose.orientation = quat_msg;
	  nbv_pose_array_.poses.push_back(nbv_pose);
	  */
	  nbp++;
	}
      }
      ROS_INFO ("%d boundary points in cluster", nbp);
  // Convert data back
  pcl::copyPointCloud(boundary_cloud	,*output);
}

/**
* creating an octree from pcl data
*/
void Nbv::createOctree (pcl::PointCloud<PointT>& pointcloud2_pcl, octomath::Pose6D sensor_pose) {

  octomap::point3d octomap_3d_point;
  octomap::Pointcloud octomap_pointcloud;

  //Reading from pcl point cloud and saving it into octomap point cloud
  BOOST_FOREACH (const PointT& pt, pointcloud2_pcl.points) {
    octomap_3d_point(0) = pt.x;
    octomap_3d_point(1) = pt.y;
    octomap_3d_point(2) = pt.z;
    octomap_pointcloud.push_back(octomap_3d_point);
  }
  
  ROS_INFO("Number of points in octomap point cloud: %d", (int)octomap_pointcloud.size());

  // Converting from octomap point cloud to octomap graph
  octomap_pointcloud.transform(sensor_pose.inv());
  octomap::ScanNode* scan_node = octomap_graph_->addNode(&octomap_pointcloud, sensor_pose);

  ROS_INFO("Number of points in scene graph: %d", octomap_graph_->getNumPoints());

  // Converting from octomap graph to octomap tree (octree)
  octree_->insertPointCloud(*scan_node,octree_maxrange_);
  octree_->expand();

  ROS_INFO("Number of nodes in octree: %d", octree_->getNumLeafNodes());
  
  //
  // create nodes that are unknown
  //
  octomap::point3d min, max;
  double min_x, min_y, min_z;
  double max_x, max_y, max_z;
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);
  min (0) = (float) min_x;
  min (1) = (float) min_y;
  min (2) = (float) min_z;
  max (0) = (float) max_x;
  max (1) = (float) max_y;
  max (2) = (float) max_z;

  ROS_INFO("octree min bounds [%f %f %f]", min(0), min(1), min(2));
  ROS_INFO("octree max bounds [%f %f %f]", max(0), max(1), max(2));

  double x,y,z;
  for (x = min(0)+octree_res_/2; x < max(0)-octree_res_/2; x+=octree_res_) {
    for (y = min(1)+octree_res_/2; y < max(1)-octree_res_/2; y+=octree_res_) {
      for (z = min(2)+octree_res_/2; z < max(2)-octree_res_/2; z+=octree_res_) {
    	  octomap::point3d centroid (x, y, z);

    	  if (z > max(2))  ROS_INFO("ahrg node at [%f %f %f]", centroid(0), centroid(1), centroid(2));
	  
    	  	  octomap::OcTreeNode *octree_node = octree_->search(centroid);
    	  	  if (octree_node) {
			  //  ROS_INFO("HEEEEEELP: octree_node != NULL");
    	  		   // octree_node->->setCentroid(centroid);
    	  	  }
    	  	  else {
    	  		  //ROS_INFO("creating node at [%f %f %f]", centroid(0), centroid(1), centroid(2));
    	  		  //corresponding node doesn't exist yet -> create it
    	  		// std::cout <<  octree_->getNumLeafNodes() << " leaf nodes before!" << std::endl;
    	  		  octomap::OcTreeNode *new_node = octree_->updateNode(centroid, false);
			//   std::cout <<  octree_->getNumLeafNodes() << " leaf nodes after!" << std::endl;
	
    	  		  new_node->setValue(unknown_label_);
    	  	  }
      	  }
    }
  }


  if (check_centroids_) {
    int cnt = 0;
    //find Leaf Nodes' centroids, assign controid coordinates to Leaf Node
    for(octomap::OcTree::leaf_iterator it = octree_->begin_leafs(),end=octree_->end_leafs(); it!= end; ++it)
    {
      octomap::point3d centroid = it.getCoordinate();
      octomap::OcTreeNode *octree_node = octree_->search(centroid);
      
      /*  if (octree_node)
       {
    	  octomap::point3d test_centroid;
    	  test_centroid = octree_node-> ->getCentroid();

    	  if (centroid.distance(test_centroid) > octree_res_/4)
    		  ROS_INFO("node at [%f %f %f] has a wrong centroid: [%f %f %f]", centroid(0), centroid(1), centroid(2), test_centroid(0), test_centroid(1), test_centroid(2));
       }
       else
       {
    	  ROS_INFO("node at [%f %f %f] not found", centroid(0), centroid(1), centroid(2));
       }
       */
     }
  }

}

void Nbv::castRayAndLabel(pcl::PointCloud<PointT>& cloud, octomap::pose6d origin) {
  octomap::point3d octomap_point3d;

  BOOST_FOREACH (const PointT& pcl_pt, cloud.points) {
    octomap_point3d(0) = pcl_pt.x;
    octomap_point3d(1) = pcl_pt.y;
    octomap_point3d(2) = pcl_pt.z;
    octomap::OcTreeNode * octree_end_node = octree_->search(octomap_point3d);

    if (octree_end_node) {
      // Get the nodes along the ray and label them as free
      if (octree_->computeRayKeys(origin.trans(), octomap_point3d, ray))
      {
    	  for(octomap::KeyRay::iterator it=ray.begin(); it != ray.end(); it++)
    	  {
    		  octomap::OcTreeNode * free_node = octree_->search(*it);
    		  if (free_node)
    		  {
    			  if (free_node->getValue() != occupied_label_)
    				  free_node->setValue(free_label_);
    		  }
    		  else
    			  ROS_DEBUG("node in ray not found!");
    	  }
    	  
      }
      else
      {
    	  ROS_DEBUG("could not compute ray from [%f %f %f] to [%f %f %f]", origin.x(), origin.y(), origin.z(), pcl_pt.x, pcl_pt.y, pcl_pt.z);
      }

      //octree_end_node->->set3DPointInliers(0);
      octree_end_node->setValue(occupied_label_);
    }
    else {
      ROS_DEBUG("ERROR: node at [%f %f %f] not found", pcl_pt.x, pcl_pt.y, pcl_pt.z);
    }
  }
  
}

void Nbv::visualizeOctree(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg, geometry_msgs::Point viewpoint)
{
  // each array stores all cubes of a different size, one for each depth level:
  octree_marker_array_msg_.markers.resize(4);
  double lowestRes = octree_->getResolution();
  ROS_INFO_STREAM("lowest resolution: " << lowestRes);
  
  for(octomap::OcTree::leaf_iterator it = octree_->begin_leafs(),end=octree_->end_leafs(); it!= end; ++it)
  {
    octomap::OcTreeNode node = *it; 
    octomap::point3d centroid = it.getCoordinate();
    
    geometry_msgs::Point cube_center;
    cube_center.x = centroid.x();
    cube_center.y = centroid.y();
    cube_center.z = centroid.z();
    
    octomap::OcTreeNode *octree_node = octree_->search(centroid);

    if (octree_node) {
      if (occupied_label_ == octree_node->getValue())
	octree_marker_array_msg_.markers[0].points.push_back(cube_center);
      else if (free_label_ == octree_node->getValue())
	octree_marker_array_msg_.markers[1].points.push_back(cube_center);
      else if (unknown_label_ == octree_node->getValue())
	octree_marker_array_msg_.markers[2].points.push_back(cube_center);
    
    }
  }

  octree_marker_array_msg_.markers[3].points.push_back(viewpoint);

  // occupied cells red
  octree_marker_array_msg_.markers[0].ns = "Occupied cells";
  octree_marker_array_msg_.markers[0].color.r = 1.0f;
  octree_marker_array_msg_.markers[0].color.g = 0.0f;
  octree_marker_array_msg_.markers[0].color.b = 0.0f;
  octree_marker_array_msg_.markers[0].color.a = 0.5f;

  if(visualize_free_cells_){
  // free cells green
  octree_marker_array_msg_.markers[1].ns ="Free cells";
  octree_marker_array_msg_.markers[1].color.r = 0.0f;
  octree_marker_array_msg_.markers[1].color.g = 1.0f;
  octree_marker_array_msg_.markers[1].color.b = 0.0f;
  octree_marker_array_msg_.markers[1].color.a = 0.5f;
  }
  // unknown cells blue
  octree_marker_array_msg_.markers[2].ns = "Unknown cells";
  octree_marker_array_msg_.markers[2].color.r = 0.0f;
  octree_marker_array_msg_.markers[2].color.g = 0.0f;
  octree_marker_array_msg_.markers[2].color.b = 1.0f;
  octree_marker_array_msg_.markers[2].color.a = 0.1f;

  // viewpoint
  octree_marker_array_msg_.markers[3].ns = "viewpoint";
  octree_marker_array_msg_.markers[3].color.r = 1.0f;
  octree_marker_array_msg_.markers[3].color.g = 1.0f;
  octree_marker_array_msg_.markers[3].color.b = 0.0f;
  octree_marker_array_msg_.markers[3].color.a = 0.8f;
  octree_marker_array_msg_.markers[3].scale.x = 2.0;
  octree_marker_array_msg_.markers[3].scale.y = 2.0;
  octree_marker_array_msg_.markers[3].scale.z = 2.0;
  ros::Duration d(50.0);
  for (unsigned i = 0; i < octree_marker_array_msg_.markers.size(); ++i)
  {
    octree_marker_array_msg_.markers[i].header.frame_id = pointcloud2_msg->header.frame_id;
    octree_marker_array_msg_.markers[i].header.stamp = ros::Time::now();
    octree_marker_array_msg_.markers[i].id = i;
    octree_marker_array_msg_.markers[i].lifetime = d; //ros::Duration::Duration();
    octree_marker_array_msg_.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    octree_marker_array_msg_.markers[i].scale.x = lowestRes;
    octree_marker_array_msg_.markers[i].scale.y = lowestRes;
    octree_marker_array_msg_.markers[i].scale.z = lowestRes;

    if (octree_marker_array_msg_.markers[i].points.size() > 0)
      octree_marker_array_msg_.markers[i].action = visualization_msgs::Marker::ADD;
    else
      octree_marker_array_msg_.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  octree_marker_array_publisher_.publish(octree_marker_array_msg_);

  for (unsigned int i = 0; i < octree_marker_array_msg_.markers.size(); i++)
  {
    if (!octree_marker_array_msg_.markers[i].points.empty())
    {
      octree_marker_array_msg_.markers[i].points.clear();
    }
  }
  octree_marker_array_msg_.markers.clear();

}


int main (int argc, char* argv[])
{
  ros::init (argc, argv, "next_best_view");
  ros::NodeHandle nh("~");
  Nbv n (nh);
  ROS_INFO("Node up and running...");
  ros::spin ();

  return (0);
}
