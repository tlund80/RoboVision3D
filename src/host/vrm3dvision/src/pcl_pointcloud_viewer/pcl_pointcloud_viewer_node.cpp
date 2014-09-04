/********************************************************************************************************************
 *
 * \file                pcl_pointcloud_viewer_node.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-22
 * \version             0.1
 * \brief
 *
*********************************************************************************************************************/

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <pcl/common/common_headers.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h> // load polygon file (.obj)
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/point_cloud.h>

#include <robot_msgs/Pose.h>


pcl::visualization::PCLVisualizer* viewer;
pcl::PointCloud<pcl::PointXYZ> point_cloud;
pcl::PolygonMesh model_mesh;

tf::TransformListener* listener;

double opacity = 0.001;

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& point_cloud)
{
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud);
	viewer->removePolygonMesh("Model");
    if(!viewer->updatePointCloud<pcl::PointXYZRGB>(point_cloud,rgb))
    {
    	viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud,rgb);
    	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    	// Find center of pointcloud
    	double x(0),y(0),z(0);
    	for (size_t i = 0; i < point_cloud->points.size(); i++)
    	{
    		x += point_cloud->points[i].x;
    		y += point_cloud->points[i].y;
    		z += point_cloud->points[i].z;
    	}
    	x /= point_cloud->points.size();
    	y /= point_cloud->points.size();
    	z /= point_cloud->points.size();
    	// Set camera to pivot around center of point cloud
    	viewer->setCameraPosition(x,y,z/4,x,y,z,0,1,0);

    }
}

void keyboardCallBack(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	if (event.getKeyCode() == 'c' && event.keyUp())
	{
		viewer->removeAllPointClouds();
	}

	if (event.getKeyCode() == 'm' && event.keyUp())
	{
		opacity *= 1.5;
    	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "Model");

	}
	if (event.getKeyCode() == 'n' && event.keyUp())
	{
		opacity *= 0.75;
    	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "Model");

	}

	if (event.getKeyCode() == 's' && event.keyUp())
	{
//		pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_no_nan(new pcl::PointCloud<pcl::PointXYZ>);
//		std::vector<int> tmp;
//		pcl::removeNaNFromPointCloud(point_cloud, *point_cloud_no_nan,tmp);
//		std::string path = ros::package::getPath("vrm3dvision") + std::string("/") + std::string("pc_sgbm.pcd");
//		ROS_INFO_STREAM("Saving Point Cloud as PCD file to: " << path);
//		pcl::io::savePCDFileASCII(path, *point_cloud_no_nan);
//
//		// save trimmed point cloud
//		pcl::PassThrough<pcl::PointXYZ> pass;
//		pass.setInputCloud(point_cloud_no_nan);
//		pass.setFilterFieldName ("z");
//		pass.setFilterLimits (0.0, 0.38);
//		//pass.setFilterLimitsNegative (true);
//		pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//		pass.filter (*point_cloud_filtered);
//
//		std::string path_trimmed = ros::package::getPath("vrm3dvision") + std::string("/") + std::string("pc_sgbm_trimmed.pcd");
//		pcl::io::savePCDFileASCII(path_trimmed, *point_cloud_filtered);
//		ROS_INFO_STREAM("Saving Trimmed Point Cloud as PCD file to: " << path_trimmed);
//
//		double leaf_size = 0.001;
//		// Subsample point cloud
//		 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//		// Create the filtering object
//		pcl::VoxelGrid<pcl::PointXYZ> sor;
//		sor.setInputCloud (point_cloud_filtered);
//		sor.setLeafSize (leaf_size,leaf_size,leaf_size);
//		sor.filter (*cloud_filtered);
//
//		std::string path_trimmed_downsampled = ros::package::getPath("vrm3dvision") + std::string("/") + std::string("pc_sgbm_trimmed_downsampled.pcd");
//		pcl::io::savePCDFileASCII(path_trimmed_downsampled, *cloud_filtered);
//		ROS_INFO_STREAM("Saving Trimmed Point Cloud as PCD file to: " << path_trimmed_downsampled);


	}
}

void modelMeshCallback(const pcl_msgs::PolygonMesh::ConstPtr& msg)
{
	viewer->removePolygonMesh("Model");

    tf::StampedTransform transform;
    try{
    	listener->lookupTransform("/vrm_stereo_camera", msg->header.frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
    	ROS_ERROR("PCL Viewer Node: %s",ex.what());
    	return;
    }
    Eigen::Matrix4f transformation_matrix;
    pcl_ros::transformAsMatrix(transform, transformation_matrix);

//	pcl::console::print_info("Transformation Matrix:\n");
//	pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_matrix (0,0), transformation_matrix (0,1), transformation_matrix (0,2));
//	pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation_matrix (1,0), transformation_matrix (1,1), transformation_matrix (1,2));
//	pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_matrix (2,0), transformation_matrix (2,1), transformation_matrix (2,2));
//	pcl::console::print_info ("\n");
//	pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation_matrix (0,3), transformation_matrix (1,3), transformation_matrix (2,3));

    pcl_conversions::toPCL(*msg, model_mesh);
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::fromPCLPointCloud2(model_mesh.cloud, tmp);

    pcl::transformPointCloud(tmp, tmp, transformation_matrix);
    pcl::toPCLPointCloud2(tmp,model_mesh.cloud);

    // Add coordinate system in reference point
    Eigen::Affine3f affineTranform(transformation_matrix);
    viewer->removeCoordinateSystem();
    viewer->addCoordinateSystem(0.02, affineTranform);

    if(!viewer->updatePolygonMesh(model_mesh,"Model"))
    {
    	viewer->addPolygonMesh(model_mesh,"Model");
    	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 100, 0, "Model");
    	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "Model");

    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_pointcloud_viewer");
	ros::NodeHandle nh("/");

	if (argc == 7)
	{
		double x, y, z, roll, pitch, yaw;
		x = atof(argv[1]);
		y = atof(argv[2]);
		z = atof(argv[3]);
		roll = atof(argv[4]) / 180.0;
		pitch = atof(argv[5]) / 180.0;
		yaw = atof(argv[6]) / 180.0;


		tf::Matrix3x3 tmp;
		tmp.setEulerYPR(roll, pitch, yaw);

		std::cout << "Transform:" << std::endl;
		std::cout << tmp.getRow(0).x() << " " << tmp.getRow(0).y() << " " << tmp.getRow(0).z() << " " << x << std::endl;
		std::cout << tmp.getRow(1).x() << " " << tmp.getRow(2).y() << " " << tmp.getRow(1).z() << " " << y << std::endl;
		std::cout << tmp.getRow(2).x() << " " << tmp.getRow(1).y() << " " << tmp.getRow(2).z() << " " << z << std::endl;
		std::cout << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
	}
	else
	{

		// Read parameter from launch file
		ros::NodeHandle local_nh("~");
		std::string point_cloud_topic;
		std::string model_mesh_topic;
		local_nh.param<std::string>("point_cloud_subscriber_topic", point_cloud_topic, "point_cloud");
		local_nh.param<std::string>("model_mesh_subscriber_topic", model_mesh_topic, "model_mesh");

		ros::Subscriber pc_subscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >(point_cloud_topic, 1, pointCloudCallback);
		ros::Subscriber model_subscriber = nh.subscribe<pcl_msgs::PolygonMesh>(model_mesh_topic, 1, modelMeshCallback);

	//	ros::Publisher robot_pose_publisher = nh.advertise<robot_msgs::Pose>("/ur_pose", 1);
//	    Eigen::Matrix4f transformation_matrix;

//	    transformation_matrix << 9.9995790275346397e-01, -8.2484103060559387e-03,
//	    	       4.0195084670494275e-03, -1.3332150707812527e-02,
//	    	       8.2856288665240918e-03, 9.9992212323888374e-01,
//	    	       -9.3325136875710372e-03, 1.9789897186697807e-02,
//	    	       -3.9422170386667647e-03, 9.3654249698253544e-03,
//	    	       9.9994837253727975e-01, 1.5721199616674042e-03, 0., 0., 0., 1.;

//	    tf::Matrix3x3 rot(0.9999586505661346, -0.007854406140264676, 0.004583171624447244,
//	    					0.007871038702313402, 0.9999624618919009, -0.003622368953552197,
//	    					-0.004554548023904282, 0.003658293491882085, 0.9999829363449286);
//
//	    double r1, r2, p1, p2, y1, y2;
//
//	    rot.getEulerYPR(y1, p1, r1);
//	    rot.getEulerZYX(y2, p2, r2);
//
//	    ROS_WARN_STREAM("YPR: R: " << r1 << " P: " << p1 << " Y: " << y1);
//	    ROS_WARN_STREAM("ZYX: R: " << r2 << " P: " << p2 << " Y: " << y2);

		listener = new tf::TransformListener();

		//Create visualizer
		viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addCoordinateSystem(0.1);
		viewer->initCameraParameters();

		viewer->registerKeyboardCallback(keyboardCallBack, (void*)&viewer);

		//Main loop
		while (!viewer->wasStopped() && ros::ok())
		{
			ros::spinOnce();
	//	    tf::StampedTransform transform;
	//	    try{
	//	    	listener->lookupTransform("/base_link", "/vrm_sensor", ros::Time(0), transform);
	//		    Eigen::Matrix4f transformation_matrix;
	//		    pcl_ros::transformAsMatrix(transform, transformation_matrix);
	//
	//		    Eigen::Affine3f affineT(transformation_matrix);
	//		    tf::Matrix3x3 m(transform.getRotation());
	//		    robot_msgs::Pose pose;
	//		    m.getEulerYPR(pose.yaw, pose.pitch, pose.roll);
	//		    pose.x = transform.getOrigin().getX();
	//		    pose.y = transform.getOrigin().getY();
	//		    pose.z = transform.getOrigin().getZ();
	//
	//		    robot_pose_publisher.publish(pose);
	//	    }
	//	    catch (tf::TransformException ex){
	//	    	//ROS_ERROR("PCL Viewer Node: %s",ex.what());
	//	    }

			viewer->spinOnce(25);
		}

		delete(viewer);

	}
	return 0;
}
