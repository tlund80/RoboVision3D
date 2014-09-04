// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/publisher.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>

#include <Eigen/Geometry>

// Types
typedef sensor_msgs::PointCloud2 SensorCloudT;
typedef boost::shared_ptr<SensorCloudT> SensorCloudPtrT;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> CloudT;

// Subscribe to point cloud publishers
ros::Subscriber sub_cloudIn;
pcl_ros::Publisher<PointT> pub_cloudOut;

//ROS params
double _dist_thread;
double _max_depth;
double _radius;
double _minNeighborsInRadius;

// Function declarations
void callback(SensorCloudT::ConstPtr);
bool removePlane(pcl::PointCloud<PointT>::Ptr &src_cloud, pcl::PointCloud<PointT>::Ptr &target_cloud, double dist_thread);
void radiusOutlierRemoval(pcl::PointCloud<PointT>::Ptr &src_cloud, pcl::PointCloud<PointT>::Ptr &target_cloud, double radius);
bool extractClusters(pcl::PointCloud<PointT>::Ptr &src_cloud, std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator_indirection<pcl::PointCloud<PointT>::Ptr> > &clusters);

// Main entry point
int main(int argc, char **argv) {
   // Init
   ros::init(argc, argv, "remove_plane_node");
   
   // Get node
   ros::NodeHandle n("~");
   
    // Get parameters
   if(!n.getParam("dist_thread", _dist_thread)) _dist_thread = 0.005;
   if(!n.getParam("radius", _radius)) _radius = 0.05;
   if(!n.getParam("minNeighborsInRadius", _minNeighborsInRadius)) _minNeighborsInRadius = 50;
   if(!n.getParam("max_depth", _max_depth)) _max_depth = 3.00;
  
   sub_cloudIn  = n.subscribe<SensorCloudT::ConstPtr>("/cloudIn", 10, callback);
   pub_cloudOut = pcl_ros::Publisher<PointT> (n, "/cloudOut", 1);
   
   ROS_INFO("remove_plane_node is running!");
   ros::spin();
   
   return 0;
}

// Callback for cloud data 
void callback(SensorCloudT::ConstPtr cloudros) {
  
  if(cloudros->data.size()> 0){
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloudros, *cloud);
  
    if(removePlane(cloud,cloud,_dist_thread)){
     
      //Remove point more than X meter away
  //    pcl::PassThrough<PointT> pass;
  //    pass.setInputCloud (cloud);
//	pass.setFilterFieldName ("z");
//	pass.setFilterLimits (0, _max_depth);
//	pass.filter (*cloud);
      
     // std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator_indirection<pcl::PointCloud<PointT>::Ptr > > clusters;
     // extractClusters(cloud, clusters);
     // std::cout << "cluster size: " << (int)clusters.size() << std::endl;
    //radiusOutlierRemoval(cloud,cloud,_radius);

      pub_cloudOut.publish(cloud);
    }else
      ROS_ERROR("Remove Plane: Could not remove plane!!");
    
  }else
    ROS_ERROR("Remove Plane: Point Cloud received with zero points!!");
}

bool removePlane(pcl::PointCloud<PointT>::Ptr &src_cloud, pcl::PointCloud<PointT>::Ptr &target_cloud, double dist_thread)
{
   //*********************************************************************//
   //	Plane fitting 
   /**********************************************************************/

   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
   // Create the segmentation object
   pcl::SACSegmentation<PointT> seg;
   // Optional
   seg.setOptimizeCoefficients (true);
   // Mandatory
   seg.setModelType (pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setDistanceThreshold (dist_thread);//0.005
   
 
   seg.setInputCloud (src_cloud);
   seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
   {
     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return false;
   }

   std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                       << coefficients->values[1] << " "
                                       << coefficients->values[2] << " "
                                       << coefficients->values[3] << std::endl;

  
   //*********************************************************************//
   //	Extract Indices
   /**********************************************************************/

   pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
   pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>);
  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
  // Extract the inliers
  extract.setInputCloud (src_cloud);
  extract.setIndices(inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points.");

  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_f);

    ROS_INFO_STREAM("PointCloud representing the model component: " << cloud_f->width * cloud_f->height << " data points.");
  pcl::io::savePCDFile("plane_positive.pcd", *cloud_p);
  pcl::io::savePCDFile("plane_negative.pcd", *cloud_f);
    
  pcl::copyPointCloud(*cloud_f, *target_cloud);
  
  return true;
  
}

void radiusOutlierRemoval(pcl::PointCloud<PointT>::Ptr &src_cloud, pcl::PointCloud<PointT>::Ptr &target_cloud, double radius)
{
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

	if(src_cloud->size() > 0)
	{
	    try{
	      // Create the filtering objects
	      pcl::RadiusOutlierRemoval<PointT> ror;
	      ror.setInputCloud(src_cloud);
	      ror.setRadiusSearch(radius);
	      ror.setMinNeighborsInRadius(_minNeighborsInRadius);
	      ror.filter (*cloud_filtered);
	      ror.setKeepOrganized(true);

	      pcl::copyPointCloud(*cloud_filtered, *target_cloud);
	    }catch(...)
	    {
	      PCL_ERROR("Somthing went wrong in radiusOutlierRemoval()");
	    }
	}
}

bool extractClusters(pcl::PointCloud<PointT>::Ptr &src_cloud, std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator_indirection<pcl::PointCloud<PointT>::Ptr> > &clusters)
{
  
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
   
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
   ec.setClusterTolerance (0.06); // 2cm
    ec.setMinClusterSize (30);
    ec.setMaxClusterSize (600);
    ec.setSearchMethod (tree);
    ec.setInputCloud(src_cloud);
    ec.extract (cluster_indices); 
    
     std::cout<<"Found "  << cluster_indices.size() << " clusters" << std::endl;
     
/*    if ( cluster_indices.size() < 1) return false;
    
     pcl::PointCloud<PointT>::Ptr model (new pcl::PointCloud<PointT>);
     
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
       
	pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
   
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    cloud_cluster->points.push_back (src_cloud->points[*pit]); //*

	    if(cloud_cluster->points.size () > 500)
	     {
	     	 cloud_cluster->width = cloud_cluster->points.size ();
	     	 cloud_cluster->height = 1;
	     	 cloud_cluster->is_dense = true;
		 
		/* Eigen::Vector4f centroid;
		 pcl::compute3DCentroid(*cloud_cluster,centroid);
		 
		 pcl::PCA<PointT> _pca; 
		 PointT projected; 
		 PointT reconstructed;
		 pcl::PointCloud<PointT > cloudi = *cloud_cluster;
		 pcl::PointCloud<PointT> finalCloud;
		 
		 
		 try{
		      //Do PCA for each point to preserve color information
		      //Add point cloud to force PCL to init_compute else a exception is thrown!!!HACK
		      _pca.setInputCloud(cloud_cluster);
		      for(int i = 0; i < (int)cloud_cluster->size(); i++)
		      {
			_pca.project(cloudi[i],projected);
			_pca.reconstruct (projected, reconstructed);

			//assign colors
			projected.r = cloudi[i].r;
			projected.g = cloudi[i].g;
			projected.b = cloudi[i].b;

			//add point to cloud
			finalCloud.push_back(projected);
		}
		 }catch(pcl::InitFailedException &e)
		 {
		    PCL_ERROR(e.what());
		 }
	      */

/*	     	 std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

		 clusters.push_back(cloud_cluster);
	     }
	     
	     
	      pcl::copyPointCloud(*cloud_cluster, *model);
      }
  */  
    return true;
  
}

