/********************************************************************************************************************
 *
 * @file		 Recognition_node.cpp
 * @author		Thomas Sølund (thso@teknologisk.dk)
 * @date		2013-04-23
 * @version		1.0
 * @brief		ROS Node implementing Halcon surface based matching
 *
*********************************************************************************************************************/
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>
#include <ros/callback_queue.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
//Downsampling using the voxel grid approach
#include <pcl/filters/voxel_grid.h>

//Boost uuid
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <boost/lexical_cast.hpp>

#include <pose_estimation_halcon/ModelCreationParameters.hpp>
#include <pose_estimation_halcon/SurfaceModelCreator.h>
#include <pose_estimation_halcon/SurfaceModelDetector.h>
#include <pose_estimation_halcon/ModelCreators.hpp>
#include <pose_estimation_halcon/ModelDetectors.hpp>
#include <pose_estimation_halcon/Create3DObjectModel.h>
#include <pose_estimation_halcon/PCDFileHandler.h>

#include <pose_estimation_halcon/prepareEstimation.h>
#include <pose_estimation_halcon/estimate.h>
#include <robot_msgs/ResultPose.h>



using namespace HalconCpp;


namespace perception {

class RecognitionCtrl {
public:
	RecognitionCtrl(ros::NodeHandle nh);
	virtual ~RecognitionCtrl();

	void startServices();
	void InitParameters();
	void setTopicSubscriber();
	void setTopicPublisher();
	void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
	void createMarker(int id, double x, double y, double z, double roll, double pitch, double yaw, double lengthX, double lengthY, double lengthZ );


	/*ROS services */
	bool prepareEstimation(pose_estimation_halcon::prepareEstimation::Request &req, pose_estimation_halcon::prepareEstimation::Response &res);
	bool estimate(pose_estimation_halcon::estimate::Request &req, pose_estimation_halcon::estimate::Response &res);


private:
	ros::NodeHandle nh;

	// Create a map for name resolving
	std::map<std::string,boost::uuids::uuid> indexmap;

	// Create a map storing halcon model by uuid
	std::map<boost::uuids::uuid,HTuple> modelmap;

	//Primilary model container
	pcl::PointCloud<pcl::PointXYZ> pcl_model;

	std::string point_cloud_topic;
	std::string scene_cloud_path;
	std::string model_cloud_path;

	ros::ServiceServer sPrepareRecognition;
	ros::ServiceServer sFind3DModel;
	ros::Publisher visPub;
	ros::Publisher segResult;
	pcl::PointCloud<pcl::PointXYZRGB> *cloud;
	bool cloud_ready;

	HTuple halcon_model;

	bool removeTable(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr tar);
};

}


namespace perception {


RecognitionCtrl::RecognitionCtrl(ros::NodeHandle _nh) {
	// TODO Auto-generated constructor stub
	nh = _nh;
	cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
	cloud_ready = false;
}

RecognitionCtrl::~RecognitionCtrl() {
	// TODO Auto-generated destructor stub

}

void RecognitionCtrl::startServices()
{

	sPrepareRecognition = nh.advertiseService("prepare", &perception::RecognitionCtrl::prepareEstimation,this);
	ROS_INFO("pose_estimation_halcon: Started service /pose_estimation_halcon/prepare.");
	sFind3DModel = nh.advertiseService("estimate", &perception::RecognitionCtrl::estimate,this);
	ROS_INFO("pose_estimation_halcon: Started service /pose_estimation_halcon/estimate.");

}

void RecognitionCtrl::InitParameters()
{
	ROS_INFO("pose_estimation_halcon: Loading Node parameters");

	// Load parameters from launch file
	nh.param<std::string>("point_cloud_topic", point_cloud_topic, "");
	nh.param<std::string>("scene_cloud_path", scene_cloud_path, "");
	nh.param<std::string>("model_cloud_path", model_cloud_path, "");

}

void RecognitionCtrl::setTopicSubscriber()
{
	ROS_INFO("pose_estimation_halcon: Subcribe to topics");


}

void RecognitionCtrl::setTopicPublisher()
{
	ROS_INFO("pose_estimation_halcon: Starting publishing service");

//	visPub = nh.advertise<visualization_msgs::Marker>("objectMarker",0);
	std::string result = "result";
	ROS_INFO_STREAM("Creating publisher \"" << nh.getNamespace() << "/" << result << "\"...");
	visPub = nh.advertise<sensor_msgs::PointCloud2>(result,50);
	// Visualization publisher
	std::string topic = "segmented";
	ROS_INFO_STREAM("Creating publisher \"" << nh.getNamespace() << "/" << topic << "\"...");
	segResult = nh.advertise<sensor_msgs::PointCloud2>(topic, 50);


}

bool RecognitionCtrl::removeTable(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr tar)
{
	//*********************************************************************//
	//	Plane fitting
	/**********************************************************************/

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (src);
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

	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

	//*********************************************************************//
	//	Extract Indices
	/**********************************************************************/

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_n (new pcl::PointCloud<pcl::PointXYZ>);
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	// Extract the inliers
	extract.setInputCloud (src);
	extract.setIndices(inliers);
	extract.setNegative (true);
	extract.filter (*cloud_n);
	std::cerr << "PointCloud representing the planar component: " << cloud_n->width * cloud_n->height << " data points." << std::endl;

	sensor_msgs::PointCloud2 segsvisuros;
	pcl::toROSMsg<pcl::PointXYZ>(*cloud_n, segsvisuros);
	segResult.publish(segsvisuros);

	pcl::copyPointCloud(*cloud_n,*tar);

	return true;
}

void RecognitionCtrl::createMarker(int id, double x, double y, double z, double roll, double pitch, double yaw, double lengthX, double lengthY, double lengthZ )
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "objectMarker";
	marker.header.stamp = ros::Time();
	marker.ns = nh.getNamespace();
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 1;
	marker.pose.position.y = 1;
	marker.pose.position.z = 1;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = lengthX;
	marker.scale.y = lengthY;
	marker.scale.z = lengthZ;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	//only if using a MESH_RESOURCE marker type:
	//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	visPub.publish(marker);

}

void RecognitionCtrl::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	ROS_INFO("DTI_3D_Recognition: Pointcloud_callback");
	// Convert input message to pointcloud
	cloud_ready = false;
	pcl::fromROSMsg(*input, *cloud);
	cloud_ready = true;

}

bool RecognitionCtrl::prepareEstimation(pose_estimation_halcon::prepareEstimation::Request &req, pose_estimation_halcon::prepareEstimation::Response &res)
{
	ROS_INFO("pose_estimation_halcon: Prepare pose estimation service called!!");

	if(!req.model_name.empty())
	{
		//Check if model already exist in map
		if(indexmap.find(req.model_name) == indexmap.end())
		{
			// Create unique id
			boost::uuids::uuid uuid = boost::uuids::random_generator()();
			std::cout << uuid << std::endl;

			// create a map for model name --> uuid
			indexmap.insert( std::make_pair(req.model_name,uuid));

			//Create the object model from a point cloud
			Create3DObjectModel cadModelCreator;


			sensor_msgs::PointCloud2 m = req.model;
			pcl::PointCloud<pcl::PointXYZ> model;
			pcl::fromROSMsg(m, model);
			pcl_model = model;

			HTuple x,y,z;
			for (size_t i = 0; i < model.points.size (); ++i)
			{
				x.Append(HTuple(model.points[i].x));
				y.Append(HTuple(model.points[i].y));
				z.Append(HTuple(model.points[i].z));
			}

			cadModelCreator.createModelFromPoints(x,y,z);

			//	cadModelCreator.readModel(req.pcd_model_path);
			//	ROS_INFO("pose_estimation_halcon: CAD model loaded from %s", req.pcd_model_path.c_str());

			//HTuple Pose;
			//HTuple radius = 10;
			//HTuple minExtend = 20;
			//HTuple maxExtend = 20;
			//CreatePose(10,10,10,0,0,0,"Rp+T","gba","point",&Pose);
			//cadModelCreator.createBoxModel(Pose,HTuple(10),HTuple(10),HTuple(10));
			//ROS_INFO("DTI_3D_Recognition: Synthetic cylinder model created!");
			cadModelCreator.computeSurfaceNormals();

			cadModelCreator.writeModel(req.surface_model_path, "obj");
			HTuple objectModel3D = cadModelCreator.get3DObjectModel();

			ROS_INFO("=============CAD Models contains: =================");
			if(cadModelCreator.hasPoints() ){
					ROS_INFO("\t%d Points", cadModelCreator.getNumberOfPoints());
			}
			if(cadModelCreator.hasFaces()){
					ROS_INFO("\t%d Faces", cadModelCreator.getNumberOfFaces());
			}
			if(cadModelCreator.hasLines()){
					ROS_INFO("\t%d Lines", cadModelCreator.getNumberOfLines());
			}
			if(cadModelCreator.hasPointNormals()){
					ROS_INFO("\tPoint normals");
			}
			if(cadModelCreator.hasTriangles()){
					ROS_INFO("\t%d Triangles", cadModelCreator.getNumberOfTriangles());
			}
			ROS_INFO("===================================================");

			//double min_x,min_y,min_z,max_x,max_y,max_z;
			//cadModelCreator.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
			//std::cout << min_x <<" "<< min_y <<" "<< min_z <<" "<< max_x <<" "<< max_y <<" "<< max_z << std::endl;

			//Create the surface model using the objectModel
			SurfaceModelCreationParameters smcParams;
			smcParams.RelSamplingDistance = HTuple(req.RelSamplingDistance);
			//smcParams.feat_angle_resolution =
			smcParams.feat_step_size_rel = HTuple(req.RelSamplingDistance);
			smcParams.ObjectModel3D = objectModel3D;

			ROS_INFO("pose_estimation_halcon: Creating surface model");
			SurfaceModelCreator smc;
			smc.setParameters(smcParams);
			smc.createModel();
			ROS_INFO("pose_estimation_halcon: Saving surface model to %s", req.surface_model_path.c_str());
			smc.saveModel(req.surface_model_path);


			halcon_model =  smc.getSurfaceModel();
			// Add Halcon model to map
			modelmap.insert(std::make_pair(uuid,smc.getSurfaceModel().Clone()));

			res.model_id = boost::lexical_cast<std::string>(uuid);
			res.isReady = true;

		}else{
			ROS_WARN("%s model already exist as surface model!",req.model_name.c_str());
			res.isReady = true;
		}

	}else
	{
		ROS_WARN("No Model name associated to the model. Please provide a name!");
		res.isReady = false;
	}


	return true;
}

bool RecognitionCtrl::estimate(pose_estimation_halcon::estimate::Request &req, pose_estimation_halcon::estimate::Response &res)
{
	ROS_INFO("pose_estimation_halcon: Pose estimation service called!!");

	// Get a point cloud
	if(!point_cloud_topic.empty()){
	   sensor_msgs::PointCloud2::ConstPtr cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic, ros::Duration(5.0));
	if(!cloud) {
	   ROS_WARN("Retrieval of point cloud failed!");
	   return false;
	}

		pcl::PointCloud<pcl::PointXYZ> scene;
		pcl::PointCloud<pcl::PointXYZRGB> scene_rgb;
		pcl::fromROSMsg(*cloud, scene);
		pcl::fromROSMsg(*cloud, scene_rgb);
		ROS_INFO("pose_estimation_halcon: grabbing scene cloud with %d points!", (int)scene.points.size ());

		//Downsample point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		// Create the filtering object
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud (scene.makeShared());
		sor.setLeafSize (req.leaf_size,req.leaf_size,req.leaf_size);
		sor.filter (*cloud_filtered);

		std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
			       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;


		pcl::PointCloud<pcl::PointXYZ>::Ptr scene_without_plane (new pcl::PointCloud<pcl::PointXYZ>);
		if(req.table_top){
		//Application is a table top application remove
			if(!removeTable(cloud_filtered,scene_without_plane)) return false;
		}else{
			pcl::copyPointCloud(*cloud_filtered, *scene_without_plane);
		}

		HTuple x,y,z;
		for (size_t i = 0; i < scene_without_plane->points.size (); ++i)
		{
			//std::cout << pc->points[i].x << " " << std::endl;
			 x.Append(HTuple(scene_without_plane->points[i].x));
			 y.Append(HTuple(scene_without_plane->points[i].y));
			 z.Append(HTuple(scene_without_plane->points[i].z));
		}

		//Create the object model from the point cloud
		ROS_INFO("Creating Model of the scene!!");
		Create3DObjectModel SearchModelCreator;
		SearchModelCreator.createModelFromPoints(x,y,z);
		SearchModelCreator.computeSurfaceNormals();
		ROS_INFO("Number of points in scene: %d", SearchModelCreator.getNumberOfPoints());
		HTuple scene_data = SearchModelCreator.get3DObjectModel();

		ObjectModelFileFormat file;
		SearchModelCreator.writeModel("scene", file.obj);

		SurfaceModelDetectionParameters detectParams;
		detectParams.RelSamplingDistance = HTuple(req.RelSamplingDistance);
		detectParams.num_matches = 10;
		detectParams.KeyPointFraction = 0.5;
		detectParams.MinScore = 0.1;

		boost::uuids::uuid uuid = boost::lexical_cast<boost::uuids::uuid>(req.model_id);
		std::map<boost::uuids::uuid, HTuple>::iterator it = modelmap.find(uuid);

	//	HTuple surface_model = halcon_model;//it->second;
	//	std::cout << surface_model << std::endl;

		HTuple pose, score;
		SurfaceModelDetector detector;
		detector.setParameters(detectParams);
		ROS_INFO("model path: %s", req.surface_model_path.c_str());
		//detector.setSurfaceModel(halcon_model);
		detector.loadModel(req.surface_model_path);
		ROS_INFO("Detecting 3D model!");
		int instances = detector.detectModel(scene_data);

		if(instances > 0)
		{
		ROS_INFO("Found %d instances of the model in the scene!", instances);

		if(detector.getBestMatch(pose, score) == 1)
		{
			ROS_INFO("Apply transformation to model!");

			HTuple HomMat;
			PoseToHomMat3d(pose,&HomMat);

			Eigen::Matrix4f t;
			t(0,0) = (double)HomMat[0]; t(0,1) = (double)HomMat[1]; t(0,2) = (double)HomMat[2]; t(0,3) = (double)HomMat[3];
			t(1,0) = (double)HomMat[4]; t(1,1) = (double)HomMat[5]; t(1,2) = (double)HomMat[6]; t(1,3) = (double)HomMat[7];
			t(2,0) = (double)HomMat[8]; t(2,1) = (double)HomMat[9]; t(2,2) = (double)HomMat[10]; t(2,3) = (double)HomMat[11];
			t(3,0) = 0; t(3,1) = 0; t(3,2) = 0; t(3,3) = 1;

			pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_model (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::transformPointCloud(pcl_model,*aligned_model, t);

			//*result_cloud += *aligned_model;
			pcl::copyPointCloud(*aligned_model,*result_cloud);

			//color model red
			for (size_t i = 0; i < result_cloud->points.size (); ++i)
			{
				 result_cloud->points[i].r = 255;
				 result_cloud->points[i].g = 0;
				 result_cloud->points[i].b = 0;
			}

			//color scene blue
			for (size_t i = 0; i < scene_rgb.points.size (); ++i)
			{
				scene_rgb.points[i].r = 0;
				scene_rgb.points[i].g = 0;
				scene_rgb.points[i].b = 255;
			}

			*result_cloud += scene_rgb;

			sensor_msgs::PointCloud2 res_msg;
			pcl::toROSMsg<pcl::PointXYZRGB>(*result_cloud, res_msg);
			visPub.publish(res_msg);

			robot_msgs::ResultPose result;
			result.pose.x = pose[0];
			result.pose.y = pose[1];
			result.pose.z = pose[2];
			result.pose.roll = pose[3];
			result.pose.pitch = pose[4];
			result.pose.yaw = pose[5];
			result.status = true;
			result.confirmed = true;

			res.pose = result;
			res.isReady = true;

			}
		}
	}else{
		ROS_ERROR("No scene topic name avaliable!! Please add in the parameter list!!");
		res.isReady = false;
	}


	return true;
}
} /* namespace perception */


int main(int argc, char **argv)
{
	ROS_INFO("*************************************************");
	ROS_INFO("This node implements 3D pose estimation using    ");
	ROS_INFO("Halcon surface based matching                    ");
	ROS_INFO("*************************************************");

	ros::init(argc, argv, "pose_estimation_halcon");
	ros::NodeHandle nh("~");

	perception::RecognitionCtrl rc(nh);
	rc.InitParameters();
	rc.setTopicSubscriber();
	rc.setTopicPublisher();
	rc.startServices();

	ROS_INFO("pose_estimation_halcon node is running");

	ros::spin();

	return 0;
}

