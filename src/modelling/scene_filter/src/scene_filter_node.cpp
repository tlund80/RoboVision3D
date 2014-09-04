// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/publisher.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>

# define PI       3.14159265358979323846  /* pi */
// Types
typedef sensor_msgs::PointCloud2 SensorCloudT;
typedef boost::shared_ptr<SensorCloudT> SensorCloudPtrT;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> CloudT;

// Subscribe to point cloud publishers
ros::Subscriber sub_cloudIn;
pcl_ros::Publisher<PointT> pub_cloudOut;
pcl_ros::Publisher<PointT> pub_cylinderOut;
ros::Publisher _marker_array_publisher;
visualization_msgs::MarkerArray _marker_array_msg_;

std::string _robot_base_frame_name;
std::string _joint1_name;
std::string _joint2_name;
std::string _joint3_name;
std::string _joint4_name;
std::string _joint5_name;
std::string _sensor_frame;

// Function declarations
void cloudCallback(SensorCloudT::ConstPtr);
void publish_marker_array(std::vector<tf::Transform> &T);
void createCylinderConxexHull(pcl::PointCloud<PointT>::Ptr &cloud_in, pcl::PointCloud<PointT>::Ptr &cloud_out, std::vector< pcl::Vertices > &polygons);
void cropHull(pcl::PointCloud<PointT>::Ptr &hull_cloud, std::vector< pcl::Vertices > &polygons, pcl::PointCloud<PointT>::Ptr &cloud_in, pcl::PointCloud<PointT>::Ptr &cloud_out);

// Main entry point
int main(int argc, char **argv) {
   // Init
   ros::init(argc, argv, "scene_filter_node");
   
   // Get node
   ros::NodeHandle n("~");
   
   if(!n.getParam("/robot_base_frame_name",_robot_base_frame_name)){
    ROS_ERROR("Scene filter: Could not get 'robot_base_frame_name' parameter from parameter server");
    return 0;
   }
   if(!n.getParam("/joint1_name",_joint1_name)){
    ROS_ERROR("Scene filter: Could not get 'joint1_name' parameter from parameter server");
    return 0;
   }
   if(!n.getParam("/joint2_name",_joint2_name)){
    ROS_ERROR("Scene filter: Could not get 'joint2_name' parameter from parameter server");
    return 0;
   }
   if(!n.getParam("/joint3_name",_joint3_name)){
    ROS_ERROR("Scene filter: Could not get 'joint3_name' parameter from parameter server");
    return 0;
   }
   if(!n.getParam("/joint4_name",_joint4_name)){
    ROS_ERROR("Scene filter: Could not get 'joint4_name' parameter from parameter server");
    return 0;
   }
   if(!n.getParam("/joint5_name",_joint5_name)){
    ROS_ERROR("Scene filter: Could not get 'joint5_name' parameter from parameter server");
    return 0;
   }
   if(!n.getParam("/sensor_frame",_sensor_frame)){
    ROS_ERROR("Scene filter: Could not get 'sensor_frame' parameter from parameter server");
    return 0;
   }
   
      std::cout << _joint1_name << std::endl;
   sub_cloudIn  = n.subscribe<SensorCloudT::ConstPtr>("/cloudIn", 10, cloudCallback);
   pub_cloudOut = pcl_ros::Publisher<PointT> (n, "/cloudOut", 1);
   pub_cylinderOut = pcl_ros::Publisher<PointT> (n, "/cylinder", 1);
   _marker_array_publisher = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
    
   ROS_INFO("scene_filter_node is running!");
   ros::spin();
   
   return 0;
}

// Callback for cloud data 
void cloudCallback(SensorCloudT::ConstPtr cloudros) {
   ROS_INFO("Cloud callback!");
  if(cloudros->data.size()> 0){
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloudros, *cloud);
  
    std::vector<tf::Transform> T; 
    tf::TransformListener listener;
    tf::StampedTransform T_base_joint3; 
    tf::StampedTransform T_base_joint2; 
    tf::StampedTransform T_base_joint1;
    tf::StampedTransform T_world_base;
    tf::StampedTransform T_base_sensorframe;

    try{
      if(listener.waitForTransform(_robot_base_frame_name, _joint3_name,ros::Time(0), ros::Duration(0.3))){
	    listener.lookupTransform(_robot_base_frame_name, _joint3_name,ros::Time(0), T_base_joint3);
	    T.push_back(T_base_joint3);
      }else
	 ROS_ERROR("tf timeout!!");
      
      if(listener.waitForTransform("/Frame", _robot_base_frame_name,ros::Time(0), ros::Duration(0.3))){
	    listener.lookupTransform("/Frame", _robot_base_frame_name,ros::Time(0), T_world_base);
	
      }else
	 ROS_ERROR("tf timeout!!");
       if(listener.waitForTransform(_robot_base_frame_name, _joint2_name,ros::Time(0), ros::Duration(0.3))){
	    listener.lookupTransform(_robot_base_frame_name, _joint2_name,ros::Time(0), T_base_joint2);
      }else
	 ROS_ERROR("tf timeout!!");
      if(listener.waitForTransform(_robot_base_frame_name, _joint1_name,ros::Time(0), ros::Duration(0.3))){
	    listener.lookupTransform(_robot_base_frame_name, _joint1_name,ros::Time(0), T_base_joint1); 
      }else
	 ROS_ERROR("tf timeout!!");
      
       if(listener.waitForTransform(_sensor_frame, _robot_base_frame_name ,ros::Time(0), ros::Duration(1.5))){
	    listener.lookupTransform(_sensor_frame, _robot_base_frame_name ,ros::Time(0), T_base_sensorframe); 
      }else
	 ROS_ERROR("tf timeout!!");
  /*    if(listener.waitForTransform(_joint4_name, _joint3_name,ros::Time(0), ros::Duration(0.3))){
	    listener.lookupTransform(_joint4_name, _joint3_name,ros::Time(0), T_joint3_joint4);
	    T.push_back(T_joint3_joint4);
      }else
	 ROS_ERROR("tf timeout!!");
      */
    }catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    
/*     std::cout << "T_world_base: " << T_world_base.getOrigin().getX() << " "
				    << T_world_base.getOrigin().getY() << " "
				    << T_world_base.getOrigin().getZ() << " "
				    << std::endl;
    
     std::cout << "T_base_joint3: " << T_base_joint3.getOrigin().getX() << " "
				    << T_base_joint3.getOrigin().getY() << " "
				    << T_base_joint3.getOrigin().getZ() << " "
				    << std::endl;
				    
     std::cout << "T_base_sensorframe: " << T_base_sensorframe.getOrigin().getX() << " "
					 << T_base_sensorframe.getOrigin().getY() << " "
					 << T_base_sensorframe.getOrigin().getZ() << " "
					 << std::endl;
  */				   				    
  //  publish_marker_array(T);
  //  pub_cloudOut.publish(cloud);
  float sample_rate = PI/16;
  float radius = 0.23;
  double hight = T_base_joint3.getOrigin().getZ() * 1.1;
   
  Eigen::Vector3f P(0.0,0.0,0.0);
  Eigen::Vector3f P2; 
  P2[0] = T_base_joint3.getOrigin().getX();
  P2[1] = T_base_joint3.getOrigin().getY();
  P2[2] = T_base_joint3.getOrigin().getZ();
  
  Eigen::Vector3f P1; 
  P1[0] = T_base_joint1.getOrigin().getX();
  P1[1] = T_base_joint1.getOrigin().getY();
  P1[2] = T_base_joint1.getOrigin().getZ()-0.1;
  
  Eigen::Vector3f dP1 = P-P1;
  Eigen::Vector3f dP2 = P2-P1;
  
  Eigen::Vector3f R = dP1.cross(dP2).normalized();
  Eigen::Vector3f S = R.cross(dP2).normalized();

  pcl::PointCloud<PointT>::Ptr cy(new pcl::PointCloud<PointT>());
  
  for(int j = 0; j<=1; j = j + 1 ){
   for(float i = 0; i<= 2*PI; i = sample_rate + i ){
      
    float x = P1[0]+(dP2[0])*j + radius * cos(i) * R[0] + radius * sin(i) * S[0];
    float y = P1[1]+(dP2[1])*j + radius * cos(i) * R[1] + radius * sin(i) * S[1];
    float z = P1[2]+(dP2[2])*j + radius * cos(i) * R[2] + radius * sin(i) * S[2];
    
//      float dx = T_base_joint2.getOrigin().getX()/2;
//	float dy = T_base_joint2.getOrigin().getY()/2;
//	float x = radius * cos(i) + dx;
//	float y = radius * sin(i) + dy;
//	float z = j;
	
	pcl::PointXYZRGB p; p.x = x; p.y = y; p.z =z; p.r = 255.0; p.g = 0.0; p.b = 0.0; p.a = 255.0;
	cy->push_back(p);
      }
    }
    
    ROS_INFO("Publish cylinder!");
    cy->header.frame_id = _robot_base_frame_name;
    pub_cylinderOut.publish(cy);
    
    tf::Quaternion Q = T_base_sensorframe.getRotation();
    tf::Matrix3x3 M(Q);  Eigen::Matrix3d ET; Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    tf::matrixTFToEigen(M,ET);
    
    transform(0,0) = float(ET(0,0)); transform(0,1) = float(ET(0,1)); transform(0,2) = float(ET(0,2)); transform(0,3) = T_base_sensorframe.getOrigin().getX();
    transform(1,0) = float(ET(1,0)); transform(1,1) = float(ET(1,1)); transform(1,2) = float(ET(1,2)); transform(1,3) = T_base_sensorframe.getOrigin().getY();
    transform(2,0) = float(ET(2,0)); transform(2,1) = float(ET(2,1)); transform(2,2) = float(ET(2,2)); transform(2,3) = T_base_sensorframe.getOrigin().getZ();
    transform(3,0) = 0;		  transform(3,1) = 0;		transform(3,2) = 0;	      transform(3,3) = 1;
   // std::cout << "transform:\n " << transform << std::endl;	
  //  std::cout << "ET:\n " << ET << std::endl;

    //Align cylinder with input cloud
    pcl::transformPointCloud(*cy,*cy,transform);
   // pcl::io::savePCDFile("cylinder.pcd",*cy);
   // pcl::io::savePCDFile("cloud_input.pcd",*cloud);
    
    pcl::PointCloud<PointT>::Ptr chull(new pcl::PointCloud<PointT>());
    std::vector< pcl::Vertices > polygons;
    createCylinderConxexHull(cy,chull,polygons);
    pcl::io::savePCDFile("chull.pcd",*chull);
    cropHull(chull,polygons,cloud,cloud);
    ROS_INFO("Publish scene cloud!");
    pub_cloudOut.publish(cloud);
    
  }else
    ROS_ERROR("Scene_filter: Point Cloud received with zero points!!");
}


void publish_marker_array(std::vector<tf::Transform> &T)
{
  _marker_array_msg_.markers.resize(T.size());
  std::cout << "T size: " << T.size() << std::endl;
  for(int i = 0; i<= T.size()-1; i++ )
  {
    double _x = double(T[i].getOrigin().getX());
    double _y = double(T[i].getOrigin().getY());
    double _z = double(T[i].getOrigin().getZ());
    
    double _xn = double(T[i+1].getOrigin().getX());
    double _yn = double(T[i+1].getOrigin().getY());
    double _zn = double(T[i+1].getOrigin().getZ());
    
     geometry_msgs::Point cylinder_center;
    cylinder_center.x = _x;
    cylinder_center.y = _y;
    cylinder_center.z = _z;
    
    pcl::PointXYZ p1(_x,_y,_z); pcl::PointXYZ p2(_xn,_yn,_zn);
    float ed = pcl::euclideanDistance(p1,p2);
   
    std::cout << ed << std::endl;
      _marker_array_msg_.markers[1].header.frame_id = "base_link";
      _marker_array_msg_.markers[1].header.stamp = ros::Time::now();
      _marker_array_msg_.markers[1].ns = "~";
      _marker_array_msg_.markers[1].id = 0;
      _marker_array_msg_.markers[1].type = visualization_msgs::Marker::CUBE_LIST;
      _marker_array_msg_.markers[1].action = visualization_msgs::Marker::ADD;
      _marker_array_msg_.markers[1].points.push_back(cylinder_center);
      _marker_array_msg_.markers[1].pose.orientation.x = 0.0;
      _marker_array_msg_.markers[1].pose.orientation.y = 0.0;
      _marker_array_msg_.markers[1].pose.orientation.z = 0.0;
      _marker_array_msg_.markers[1].pose.orientation.w = 1.0;
      _marker_array_msg_.markers[1].scale.x = 0.15; //15 cm
      _marker_array_msg_.markers[1].scale.y = 0.15;
      _marker_array_msg_.markers[1].scale.z = ed;
      _marker_array_msg_.markers[1].color.a = 1.0;
      _marker_array_msg_.markers[1].color.r = 1.0;
      _marker_array_msg_.markers[1].color.g = 0.0;
      _marker_array_msg_.markers[1].color.b = 0.0;
      
        std::cout << "i" << i << std::endl;
  }
  
  _marker_array_publisher.publish(_marker_array_msg_);
  _marker_array_msg_.markers.clear();
  
}

void createCylinderConxexHull(pcl::PointCloud<PointT>::Ptr &cloud_in, pcl::PointCloud<PointT>::Ptr &cloud_out, std::vector< pcl::Vertices > &polygons)
{
  pcl::PointCloud<PointT>::Ptr cloudOut (new pcl::PointCloud<PointT>());
  
  pcl::ConvexHull<PointT> chull;
  chull.setInputCloud (cloud_in);
  chull.setDimension(3);
  chull.reconstruct (*cloudOut, polygons);

  std::cerr << "Concave hull has: " << cloudOut->points.size ()
            << " data points." << std::endl;
  std::cerr << "Concave hull has: " << polygons.size()
            << " polygons." << std::endl;
  pcl::copyPointCloud(*cloudOut,*cloud_out);  
}

void cropHull(pcl::PointCloud<PointT>::Ptr &hull_cloud, std::vector< pcl::Vertices > &polygons, pcl::PointCloud<PointT>::Ptr &cloud_in, pcl::PointCloud<PointT>::Ptr &cloud_out)
{
  pcl::PointCloud<PointT>::Ptr cloudOut (new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloudResult (new pcl::PointCloud<PointT>());
  cloudResult->resize(0);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  
  pcl::CropHull<PointT> cropHull; 
  cropHull.setHullCloud(hull_cloud); 
  cropHull.setHullIndices(polygons);
  cropHull.setInputCloud(cloud_in);
  cropHull.setDim(3);
  cropHull.setCropOutside(true);
  cropHull.filter(*cloudOut);
  std::cout << "cloud_in size: " << cloud_in->size() << std::endl;
  std::cout << "cloudOut size: " << cloudOut->size() << std::endl;
 // pcl::io::savePCDFile("crop_hull.pcd",*cloudOut);

  if(cloudOut->size() > 0){
    std::vector<int> k_indices;
    std::vector<float> k_squared_dist;
    pcl::search::KdTree<PointT>::Ptr search(new pcl::search::KdTree<PointT>());
    search->setInputCloud(cloud_in);
 
    for(int i = 0; i<= cloudOut->size()-1;i++){
    PointT p; 
    p.x = cloudOut->points[i].x;
    p.y = cloudOut->points[i].y;
    p.z = cloudOut->points[i].z;
    search->radiusSearch(p,0.0001,k_indices,k_squared_dist);
    //std::cout << "k_indices.size(): " << k_indices.size() << std::endl;
    if(k_indices.size() > 0){
       for(int j = 0; j<= k_indices.size()-1; j++){
	  indices->indices.push_back(k_indices[j]);
       }
    }
  }
 
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_in);
  extract.setNegative(true);
  extract.setIndices(indices);
  extract.filter(*cloudResult);
          
    // pcl::io::savePCDFile("cloud_result.pcd",*cloudResult);
    pcl::copyPointCloud(*cloudResult,*cloud_out);  
  }
}