// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/StdVector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/norms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// Own
#include <robot_srvs/Segmentation.h>

// Point cloud types
typedef sensor_msgs::PointCloud2 SensorCloudT;
typedef std::vector<SensorCloudT> SensorCloudVecT;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef std::vector<CloudT, Eigen::aligned_allocator<CloudT> > CloudVecT;

// Service type
typedef robot_srvs::Segmentation MsgT;
typedef MsgT::Request ReqT;
typedef MsgT::Response RespT;

// Services
bool serviceNearFar(ReqT&, RespT&);
bool serviceTableRemoval(ReqT&, RespT&);
bool serviceUnderTableRemoval(ReqT&, RespT&);
bool serviceOutlierRemoval(ReqT&, RespT&);
bool serviceClustering(ReqT&, RespT&);
bool serviceAll(ReqT&, RespT&);

// Segmentation functions
void nearFar(const CloudT&, CloudT&);
pcl::ModelCoefficients::ConstPtr tableRemoval(const CloudT&, CloudT&);
void underTableRemoval(const CloudT&, CloudT&, pcl::ModelCoefficients::ConstPtr);
void aboveTableRemoval(const CloudT&, CloudT&, pcl::ModelCoefficients::ConstPtr);
void outlierRemoval(const CloudT&, CloudT&);
void clustering(const CloudT&, CloudVecT&);

// Utility functions
void applyColor(CloudT&, double, double, double);

// Main entry point
int main(int argc, char **argv) {
   // Init
   ros::init(argc, argv, "segmentation");
   
   // Get node
   ros::NodeHandle n("~");
   
   // Segmentation services
   ros::ServiceServer serviceNF= n.advertiseService<ReqT, RespT>("near_far", serviceNearFar);
   ros::ServiceServer serviceTR = n.advertiseService<ReqT, RespT>("table_removal", serviceTableRemoval);
   ros::ServiceServer serviceUTR = n.advertiseService<ReqT, RespT>("under_table_removal", serviceUnderTableRemoval);
   ros::ServiceServer serviceOR = n.advertiseService<ReqT, RespT>("outlier_removal", serviceOutlierRemoval);
   ros::ServiceServer serviceC = n.advertiseService<ReqT, RespT>("clustering", serviceOutlierRemoval);
   ros::ServiceServer serviceA = n.advertiseService<ReqT, RespT>("all", serviceAll);
   
   ROS_INFO("Segmentation node waiting for requests...");
   
   ros::spin();
   
   return 0;
}

/*
 * Callback functions
 */
bool serviceNearFar(ReqT& req, RespT& resp) {
   // Convert request to PCL format
   CloudT::Ptr cloud(new CloudT);
   pcl::fromROSMsg<PointT>(req.input, *cloud);
   
   ROS_DEBUG("Performing near/far clipping...");

   CloudT cloud_segmented;
   nearFar(*cloud, cloud_segmented);
   
   ROS_DEBUG("Near/far clipping done!");
   
   // Color white
   applyColor(cloud_segmented, 1.0, 1.0, 1.0);
   
   // Store into response
   SensorCloudT cloud_segmented_ros;
   pcl::toROSMsg<PointT>(cloud_segmented, cloud_segmented_ros);
   resp.output.push_back(cloud_segmented_ros);
   resp.output.front().header.frame_id = "segmentation_near_far";
   
   return true;
}

bool serviceTableRemoval(ReqT& req, RespT& resp) {
   // Convert request to PCL format
   CloudT::Ptr cloud(new CloudT);
   pcl::fromROSMsg<PointT>(req.input, *cloud);
   
   ROS_DEBUG("Performing table removal...");

   CloudT cloud_segmented;
   tableRemoval(*cloud, cloud_segmented);
   
   ROS_DEBUG("Table removal done!");
   
   // Color white
   applyColor(cloud_segmented, 1.0, 1.0, 1.0);
   
   // Store into response
   SensorCloudT cloud_segmented_ros;
   pcl::toROSMsg<PointT>(cloud_segmented, cloud_segmented_ros);
   resp.output.push_back(cloud_segmented_ros);
   resp.output.front().header.frame_id = "segmentation_table_removal";

   return true;
}

bool serviceUnderTableRemoval(ReqT& req, RespT& resp) {
   // Convert request to PCL format
   CloudT::Ptr cloud(new CloudT);
   pcl::fromROSMsg<PointT>(req.input, *cloud);
   
   ROS_DEBUG("Performing removal of points under table...");

   CloudT tmp;
   pcl::ModelCoefficients::ConstPtr coeff = tableRemoval(*cloud, tmp);
   CloudT cloud_segmented;
   underTableRemoval(*cloud, cloud_segmented, coeff);
   
   ROS_DEBUG("Points under table removal done!");
   
   // Color white
   applyColor(cloud_segmented, 1.0, 1.0, 1.0);
   
   // Store into response
   SensorCloudT cloud_segmented_ros;
   pcl::toROSMsg<PointT>(cloud_segmented, cloud_segmented_ros);
   resp.output.push_back(cloud_segmented_ros);
   resp.output.front().header.frame_id = "segmentation_under_table_removal";

   return true;   
}

bool serviceOutlierRemoval(ReqT& req, RespT& resp) {
   // Convert request to PCL format
   CloudT::Ptr cloud(new CloudT);
   pcl::fromROSMsg<PointT>(req.input, *cloud);
   
   ROS_DEBUG("Performing outlier removal...");

   CloudT cloud_segmented;
   outlierRemoval(*cloud, cloud_segmented);
   
   ROS_DEBUG("Outlier removal done!");
   
   // Color white
   applyColor(cloud_segmented, 1.0, 1.0, 1.0);
   
   // Store into response
   SensorCloudT cloud_segmented_ros;
   pcl::toROSMsg<PointT>(cloud_segmented, cloud_segmented_ros);
   resp.output.push_back(cloud_segmented_ros);
   resp.output.front().header.frame_id = "segmentation_outlier_removal";

   return true;
}

bool serviceClustering(ReqT& req, RespT& resp) {
   // Convert request to PCL format
   CloudT::Ptr cloud(new CloudT);
   pcl::fromROSMsg<PointT>(req.input, *cloud);
   
   ROS_DEBUG("Performing clustering...");

   CloudVecT cloud_segmented_vec;
   clustering(*cloud, cloud_segmented_vec);
   
   ROS_DEBUG("Clustering done!");
   
   // Store into response
   const int size = cloud_segmented_vec.size();
   SensorCloudVecT cloud_segmented_vec_ros(size);
   for(int i = 0; i < size; ++i) {
      // Color random
      double r,g,b;
      pcl::visualization::getRandomColors(r, g, b);
      applyColor(cloud_segmented_vec[i], r, g, b);
      // Store
      pcl::toROSMsg<PointT>(cloud_segmented_vec[i], cloud_segmented_vec_ros[i]);
      std::ostringstream oss;
      oss << "segmentation_cluster_" << i;
      cloud_segmented_vec_ros[i].header.frame_id = oss.str();
   }
   
   resp.output = cloud_segmented_vec_ros;

   return true;
}

bool serviceAll(ReqT& req, RespT& resp) {
   ROS_INFO("Performing segmentation...");
   const ros::Time begin = ros::Time::now();
   
   // Convert request to PCL format
   CloudT cloud_segmented;
   pcl::fromROSMsg(req.input, cloud_segmented);
   
   ROS_INFO("\tPerforming near/far clipping...");
   int size = cloud_segmented.size();
   nearFar(cloud_segmented, cloud_segmented);
   ROS_INFO_STREAM("\t" << size << " --> " << cloud_segmented.size());
   
   ROS_INFO("\tPerforming table removal...");
   size = cloud_segmented.size();
   pcl::ModelCoefficients::ConstPtr coeff = tableRemoval(cloud_segmented, cloud_segmented);
   underTableRemoval(cloud_segmented, cloud_segmented, coeff);
   aboveTableRemoval(cloud_segmented, cloud_segmented, coeff);
   ROS_INFO_STREAM("\t" << size << " --> " << cloud_segmented.size());
   
   ROS_INFO("\tPerforming outlier removal...");
   size = cloud_segmented.size();   
   outlierRemoval(cloud_segmented, cloud_segmented);
   ROS_INFO_STREAM("\t" << size << " --> " << cloud_segmented.size());
   
   ROS_INFO("\tPerforming clustering...");
   size = cloud_segmented.size();   
   CloudVecT cloud_segmented_vec;
   clustering(cloud_segmented, cloud_segmented_vec);
   ROS_INFO_STREAM("\t" << size << " --> " << cloud_segmented.size());
   
   // Store into response
   size = cloud_segmented_vec.size();
   SensorCloudVecT cloud_segmented_vec_ros(size);
   for(int i = 0; i < size; ++i) {
      // Color random
      double r,g,b;
      pcl::visualization::getRandomColors(r, g, b);
      applyColor(cloud_segmented_vec[i], r, g, b);
      // Stor
      pcl::toROSMsg<PointT>(cloud_segmented_vec[i], cloud_segmented_vec_ros[i]);
      std::ostringstream oss;
      oss << "segmentation_cluster_" << i;
      cloud_segmented_vec_ros[i].header.frame_id = oss.str();
   }
   
   resp.output = cloud_segmented_vec_ros;
   
   ROS_INFO("All done!");
   ROS_INFO_STREAM("\tElapsed time: " << (ros::Time::now()-begin).toSec() << " s");
   ROS_INFO_STREAM("\tSegments: " << size);
   
   return true;   
}

/*
 * Segmentation functions
 */
void nearFar(const CloudT& cloud, CloudT& cloud_segmented) {
   pcl::PassThrough<PointT> filter;
   filter.setInputCloud(cloud.makeShared());
   filter.setFilterFieldName("z");
   filter.setFilterLimits(0.4, 2.0); // TODO: Hard-coded
   filter.setFilterLimitsNegative(false);
   CloudT result;
   filter.filter(result);
   
   cloud_segmented = result;
}

pcl::ModelCoefficients::ConstPtr tableRemoval(const CloudT& cloud, CloudT& cloud_segmented) {
   // Initialize stuff
   pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
   // Create the segmentation object
   pcl::SACSegmentation<PointT> seg;
   seg.setInputCloud(cloud.makeShared());
   // Optional
   seg.setMaxIterations(500); // TODO: Hard-coded
   seg.setOptimizeCoefficients(true);
   // Mandatory
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setMethodType(pcl::SAC_RANSAC);
   seg.setDistanceThreshold (0.01); // TODO: Hard-coded
   // Segment out inlier indices
   seg.segment(*inliers, *coeff);
   pcl::ExtractIndices<PointT> extract;
   extract.setInputCloud(cloud.makeShared());
   extract.setIndices(inliers);
   extract.setNegative(true);
   CloudT result;
   extract.filter(result);
   cloud_segmented = result;
   
   return coeff;
}

void underTableRemoval(const CloudT& cloud, CloudT& cloud_segmented, pcl::ModelCoefficients::ConstPtr coeff) {
   // Project all points to the table plane
   pcl::ProjectInliers<PointT> proj;
   proj.setModelType(pcl::SACMODEL_PLANE);
   proj.setInputCloud(cloud.makeShared());
   proj.setModelCoefficients(coeff);
   CloudT projected;
   proj.filter(projected);
   // Store only points which are closer to view than their projected counterparts
   CloudT result;
   const int size = cloud.size();
   for(int i = 0; i < size; ++i) {
      const PointT& p = cloud[i];
      const PointT& pc = projected[i];
      if( (p.x*p.x+p.y*p.y+p.z*p.z) < (pc.x*pc.x+pc.y*pc.y+pc.z*pc.z) )
         result.push_back(p);
   }
   cloud_segmented = result;
}

void aboveTableRemoval(const CloudT& cloud, CloudT& cloud_segmented, pcl::ModelCoefficients::ConstPtr coeff) {
   // Project all points to the table plane
   pcl::ProjectInliers<PointT> proj;
   proj.setModelType(pcl::SACMODEL_PLANE);
   proj.setInputCloud(cloud.makeShared());
   proj.setModelCoefficients(coeff);
   CloudT projected;
   proj.filter(projected);
   // TODO: Store only points which are closer to table than hard-coded value
   const float disttolSquared = 0.5f*0.5f; // 50 cm
   const int size = cloud.size();
   CloudT result;
   for(int i = 0; i < size; ++i) {
      const PointT& p = cloud[i];
      const PointT& pc = projected[i];
      const float dx = pc.x - p.x;
      const float dy = pc.y - p.y;
      const float dz = pc.z - p.z;
      if( (dx*dx + dy*dy + dz*dz) < disttolSquared ) // Distance between original and projected point
         result.push_back(p);
   }
   cloud_segmented = result;
}

void outlierRemoval(const CloudT& cloud, CloudT& cloud_segmented) {
   pcl::StatisticalOutlierRemoval<PointT> filter;
   filter.setInputCloud(cloud.makeShared());
   filter.setMeanK(50); // TODO: Hard-coded
   filter.setStddevMulThresh(1.0); // TODO: Hard-coded
   CloudT result;
   filter.filter(result);
   cloud_segmented = result;
}

void clustering(const CloudT& cloud, CloudVecT& cloud_segmented_vec) {
   // Result
   std::vector<pcl::PointIndices> clusters;
   // Search
   pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
   tree->setInputCloud(cloud.makeShared());

   // Clustering
   pcl::EuclideanClusterExtraction<PointT> clustering;
   clustering.setInputCloud(cloud.makeShared());
   clustering.setSearchMethod(tree);
   clustering.setClusterTolerance(0.02); // TODO: Hard-coded
   clustering.setMinClusterSize(100); // TODO: Hard-coded
   clustering.setMaxClusterSize(25000); // TODO: Hard-coded
   clustering.extract(clusters);


   // Extract clusters as point clouds
   pcl::ExtractIndices<PointT> filter;
   filter.setInputCloud(cloud.makeShared());
   const int size = clusters.size();
   cloud_segmented_vec.resize(size);
   for(int i = 0; i < size; ++i) { // Loop  over all clusters

      const std::vector<int>& indicesi = clusters[i].indices;
      CloudT& cloudsegi = cloud_segmented_vec[i];
      cloudsegi.reserve(indicesi.size());
      for(std::vector<int>::const_iterator it = indicesi.begin(); it != indicesi.end(); ++it) // Loop over all point indices of cluster i
         cloudsegi.push_back(cloud[*it]);
   }
}

/*
 * Utility functions
 */
void applyColor(CloudT& cloud, double r, double g, double b) {
   const uint8_t rr(255*r+0.5);
   const uint8_t gg(255*g+0.5);
   const uint8_t bb(255*b+0.5);
   for(CloudT::iterator it = cloud.begin(); it != cloud.end(); ++it) {
      it->r = rr;
      it->g = gg;
      it->b = bb;
   }
}
