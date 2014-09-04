// Boost
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>

// STL
#include <map>

// ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
tf::TransformListener* tl;

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/point_picking_event.h>

// Types
typedef sensor_msgs::PointCloud2 SensorCloudT;
typedef boost::shared_ptr<SensorCloudT> SensorCloudPtrT;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef pcl::visualization::PCLVisualizer VisuT;
typedef boost::shared_ptr<VisuT> VisuPtr;
typedef pcl::visualization::PointCloudColorHandlerGenericField<PointT> HandlerXYZT;
typedef pcl::visualization::PointCloudColorHandlerRGBField<PointT> HandlerRGBT;

// Visualizer
VisuPtr visu;
boost::mutex mvisu;

// Set to true when first data set is received
bool data = false;
boost::mutex mdata;

// Map of <label,cloud> that are shown in view
std::map<std::string,CloudT::ConstPtr> cloudmap;
boost::mutex mcloudmap;

// Working directory
std::string working_dir;

// Function declarations
void callback(SensorCloudT::ConstPtr);
void updateVisu(const std::string&, bool);
bool callbackClearVisu(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
void callbackKey(const pcl::visualization::KeyboardEvent&);
void clearParams(ros::NodeHandle& n);

// Main entry point
int main(int argc, char **argv) {
   // Init
   ros::init(argc, argv, "point_cloud_visualization");
   
   // Get node
   ros::NodeHandle n("~");
   
   tl = new tf::TransformListener;
   
   // Get parameters
   std::string source, topic, pcdfile;
   n.getParam("source", source);
   n.getParam("topic", topic);
   n.getParam("pcdfile", pcdfile);
   
   if(!pcdfile.empty()) {
      ROS_WARN("PCD file specified in command line, acting only as a visualizer!");
      
      // Load
      SensorCloudPtrT cloud(new SensorCloudT);
      pcl::io::loadPCDFile(pcdfile, *cloud);
      
      // Check for RGB
      bool hasrgb = false, hasrgba = false;
      for(int i = 0; i < cloud->fields.size(); ++i) {
         if(cloud->fields[i].name == "rgb")
            hasrgb = true;
         if(cloud->fields[i].name == "rgba")
            hasrgba = true;
      }
      pcl::visualization::PCLVisualizer visu;
      
      if(hasrgb) {
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
         pcl::fromROSMsg(*cloud, *cloudrgb);
         visu.addPointCloud<pcl::PointXYZRGB>(cloudrgb);
      } else if(hasrgba) {
         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudrgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
         pcl::fromROSMsg(*cloud, *cloudrgba);
         visu.addPointCloud<pcl::PointXYZRGBA>(cloudrgba);
      } else {
         pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>);
         pcl::fromROSMsg(*cloud, *cloudxyz);
         visu.addPointCloud<pcl::PointXYZ>(cloudxyz, pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>(cloudxyz, "z"));
      }
      
      visu.setBackgroundColor(0.75, 0.75, 0.75);
      
      visu.spin();
      
      ROS_WARN("Shutting down...");
      ros::shutdown();
   }
   
   // Check source parameter
   if(source.empty()) {
      ROS_WARN("No source argument specified!");
      ROS_WARN("Usage:\n\
\trosrun kinect_visualization kinect_visualization _source:=SOURCE_NAME [_topic:=TOPIC_NAME] [_pcdfile:=PCD_FILE]\n\
\tSOURCE_NAME can either specify a single source, e.g. \"kinect_left\", or multiple sources separated by comma, e.g. \"kinect_left,object_detection\"\n\
\tTOPIC_NAME is optional and defaults to \"depth_registered/points\" and currently applies to all sources\n\
\tPCD_FILE is optional. NOTE: if specified, disables all topics and just acts as a visualizer of a PCD file!");
      clearParams(n);
      ros::shutdown();
   }
   
   // Get node working dir
   working_dir = ros::package::getPath("kinect_visualization");
   
   // Get topic
   if(topic.empty())
      topic = "depth_registered/points";
   
   // Generate source list and check
   std::vector<std::string> sources;
   boost::split(sources, source, boost::is_any_of(","));
   const int size = sources.size();
   if(size==0) {
      ROS_WARN("Failed to parse source argument!");
      clearParams(n);
      ros::shutdown();
   }
   
   // Subscribe to point cloud publishers
   ros::Subscriber subs[size];
   for(int i = 0; i < size; ++i) {
      ROS_INFO_STREAM("Subscribing to source topic \"/" << sources[i] << "/" << topic << "\"...");
      subs[i] = n.subscribe<SensorCloudT::ConstPtr>("/"+sources[i]+"/"+topic, 10, callback);
   }
   
   // Create service for clearing view
   ros::ServiceServer servclear = n.advertiseService<std_srvs::Empty::Request,std_srvs::Empty::Response>("clear", callbackClearVisu);
   
   ROS_INFO_STREAM("Kinect visualization node waiting for data from " << size << (size==1 ? " source" : " sources" ) << "...");
   
   // TODO: Let's try to run this at 30 Hz which is the maximum possible for the Kinect
   ros::Rate r(30);
   
   while(ros::ok()) { // Returns false at SIGINT (CTRL+C) or ros::shutdown()
      // Get data flag
      bool dataa;
      {
         boost::mutex::scoped_lock lock(mdata);
         dataa = data;
      }
      
      if(dataa) {
         // Lock visu
         boost::mutex::scoped_lock lock(mvisu);
         
         // Update the view
         visu->spinOnce();
            
         // If user closes window, terminate app
         if(visu->wasStopped()) {
            ROS_INFO("Kinect visualization closing down.");
            ros::shutdown();
         }
      }
      
      r.sleep();
      ros::spinOnce();
   }
   
   clearParams(n);
   
   return 0;
}

// Utility function for adding a frame
namespace {
   inline void addFrame(pcl::visualization::PCLVisualizer& view, const tf::Transform& T, const std::string& label) {
      const double length = 0.1;
      // Origo
      pcl::PointXYZ p;
      p.x = T.getOrigin()[0];
      p.y = T.getOrigin()[1];
      p.z = T.getOrigin()[2];
      
      // Axes
      const tf::Matrix3x3& R = T.getBasis();
      pcl::PointXYZ px = p, py = p, pz = p;
      px.x += R[0][0] * length;
      px.y += R[1][0] * length;
      px.z += R[2][0] * length;
      py.x += R[0][1] * length;
      py.y += R[1][1] * length;
      py.z += R[2][1] * length;
      pz.x += R[0][2] * length;
      pz.y += R[1][2] * length;
      pz.z += R[2][2] * length;
      
      // Add
      view.removeShape(label+"x");
      view.removeShape(label+"y");
      view.removeShape(label+"z");
      view.addLine(p, px, 1.0, 0.0, 0.0, label+"x");
      view.addLine(p, py, 0.0, 1.0, 0.0, label+"y");
      view.addLine(p, pz, 0.0, 0.0, 1.0, label+"z");
   }
}

// Callback for cloud data - label is extracted from the PointCloud2::header::frame_id entry 
void callback(SensorCloudT::ConstPtr cloudros) {
   // Get label
   const std::string& label = cloudros->header.frame_id;
   
   // See whether we have RGB information
   bool rgb = false;
   bool rgba = false;
   for(std::vector<sensor_msgs::PointField>::const_iterator it = cloudros->fields.begin(); it != cloudros->fields.end(); ++it) {
      if(it->name=="rgb")
         rgb = true;
      if(it->name=="rgba")
         rgba = true;
   }
   
   // Convert to PCL point cloud
   CloudT::Ptr cloud(new CloudT);
   if(rgb) { // RGB, do direct conversion
      pcl::fromROSMsg<PointT>(*cloudros, *cloud);
   } else if(rgba) { // RGBA, intermediate RGBA --> RGB conversion necessary
      pcl::PointCloud<pcl::PointXYZRGBA> tmp;
      pcl::fromROSMsg<pcl::PointXYZRGBA>(*cloudros, tmp);
      // Copy XYZ
      pcl::copyPointCloud(tmp, *cloud);
      // Copy RGB
      pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator ittmp = tmp.begin();
      pcl::PointCloud<pcl::PointXYZRGB>::iterator itcloud = cloud->begin();
      for(; ittmp != tmp.end(); ++ittmp, ++itcloud) {
         // Unpack RGBA
         uint32_t rgb = reinterpret_cast<const uint32_t&>(ittmp->rgba);
         const uint8_t r = (rgb >> 16) & 0x0000ff;
         const uint8_t g = (rgb >> 8)  & 0x0000ff;
         const uint8_t b = (rgb)       & 0x0000ff;
         // Pack RGB
         rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
         itcloud->rgb = reinterpret_cast<const float&>(rgb);
      }
      
      rgb=true; // Update RGB flag
   } else { // No RGB information, get XYZ data only, but store in RGB cloud
      pcl::PointCloud<pcl::PointXYZ> cloudxyz;
      pcl::fromROSMsg<pcl::PointXYZ>(*cloudros, cloudxyz);
      pcl::copyPointCloud(cloudxyz, *cloud);
   }
   
   // Voxelize point cloud if large
   if (cloud->size() > 307200) {
      pcl::VoxelGrid<PointT> vg;
      vg.setLeafSize(0.005, 0.005, 0.005);
      vg.setInputCloud(cloud);
   
      CloudT::Ptr cloudv(new CloudT);
      vg.filter(*cloudv);
      cloud = cloudv;
   }
   
   // Store into map
   {
      // Lock map
      boost::mutex::scoped_lock lock(mcloudmap);
      cloudmap[label] = cloud;
   }
   
   // Update the view
   updateVisu(label, rgb);
   
   // TODO: Find frames for the node object_detection
   const int maxnum = 25;
   for(int i = 0; i < maxnum; ++i) {
      std::ostringstream oss;
      oss << "/object_detection_" << i;
      if( tl->canTransform("/world", oss.str(), ros::Time(0)) ) {
         tf::StampedTransform T;
         tl->lookupTransform("/world", oss.str(), ros::Time(0), T);
         // Add the frame axes
         boost::mutex::scoped_lock lock(mvisu);
         ::addFrame(*visu, T, oss.str());
      }
   }
   
   // Set data flag
   {
      boost::mutex::scoped_lock lock(mdata);
      data = true;
   }
}

// Visualization function, updates view with the map entry of the given label
void updateVisu(const std::string& label, bool rgb) {
   // Lock visualizer
   boost::mutex::scoped_lock lock(mvisu);
   
   // First time initialize the view
   if(!visu) {
      visu.reset(new VisuT("Kinect visualization"));
      visu->setBackgroundColor(0.25, 0.25, 0.25);
      visu->addCoordinateSystem(0.25);
      visu->addText("Press 'a' to save all point clouds to PCD files", 0, 60, 1, 1, 1);
      visu->addText("Press 'p' to list active point clouds in the terminal", 0, 40, 1, 1, 1);
      visu->addText("Press 'h' to list interaction commands in the terminal", 0, 20, 1, 1, 1);
      
      visu->registerKeyboardCallback(callbackKey);
     // visu->setCameraPose(0,0,-0.5, 0,0,1, 0,-1,0);
     visu->setCameraPosition(0,0,-0.5, 0,0,1, 0,-1,0);
   }
   
   // Get point cloud
   CloudT::ConstPtr cloud;
   {
      // Lock map
      boost::mutex::scoped_lock lock(mcloudmap);
      cloud = cloudmap[label];
   }
   
   // Add point cloud
   if(cloud) {
      // First try to update the labeled point cloud, otherwise add
      if(rgb) {
         if( !visu->updatePointCloud<PointT>(cloud, HandlerRGBT(cloud), label) )
            visu->addPointCloud<PointT>(cloud, HandlerRGBT(cloud), label);
      } else {
         if( !visu->updatePointCloud<PointT>(cloud, HandlerXYZT(cloud, "z"), label) )
            visu->addPointCloud<PointT>(cloud, HandlerXYZT(cloud, "z"), label);
      }
   }
}

// Clear view, called either from service or from within visu
bool callbackClearVisu(std_srvs::Empty::Request& ereq, std_srvs::Empty::Response& eresp) {
   ROS_INFO("Clearing visualization");
   boost::mutex::scoped_lock lockvisu(mvisu);
   boost::mutex::scoped_lock lockdata(mdata);
   boost::mutex::scoped_lock lockcloudmap(mcloudmap);
   visu->removeAllPointClouds();
   cloudmap.clear();
   data=false;
   
   return true;
}

// Callback from within visu
void callbackKey(const pcl::visualization::KeyboardEvent& e) {
   if(e.keyUp()) {
      const unsigned char key = e.getKeyCode();
      switch(key) {
         case 'p': {
            ROS_INFO("Active point clouds (size):");
            boost::mutex::scoped_lock lockcloudmap(mcloudmap);
            for(std::map<std::string,CloudT::ConstPtr>::const_iterator it = cloudmap.begin(); it != cloudmap.end(); ++it)
               ROS_INFO_STREAM("\t\"" << it->first << "\" (" << it->second->size() << ")");
            break;
         }
         case 'a': {
            ROS_INFO_STREAM("Saving " << cloudmap.size() << " PCD file(s) inside working directory \"" << working_dir << "\"...");
            const std::string extension = ".pcd";
            for(std::map<std::string,CloudT::ConstPtr>::const_iterator it = cloudmap.begin(); it != cloudmap.end(); ++it) {
               const std::string basename = working_dir+"/"+it->first;
               std::string filename = basename+extension;
               int i = 1;
               while(boost::filesystem::exists(boost::filesystem::path(filename))) {
                  std::ostringstream oss;
                  oss << i;
                  filename = basename+"_"+oss.str()+extension;
                  ++i;
               }
               ROS_INFO_STREAM("\t" << filename);
               pcl::io::savePCDFile(filename, *it->second);
            }
            ROS_INFO("Done!");
            break;
         }
         default:
            break;
      }
   }
}

// Remove private parameters from the parameter server
void clearParams(ros::NodeHandle& n) {
   n.deleteParam("source");
   n.deleteParam("topic");
   n.deleteParam("pcdfile");
   delete tl;
}
