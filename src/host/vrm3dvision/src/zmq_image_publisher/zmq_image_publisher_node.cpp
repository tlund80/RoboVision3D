/********************************************************************************************************************
 *
 * \file                zmq_image_publisher_node.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-22
 * \version             0.1
 * \brief              	ROS node for publishing images via ZMQ to simulate the D3 board
 *
*********************************************************************************************************************/

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

#include <vrm3dvision/triggerCamera.h>


// ZeroMQ
#include <zmq.h>

// OpenCV
#include <opencv2/opencv.hpp>

// VRM communication protocol
#include <vrm_protocol/pubsub.hpp>
#include <vrm_protocol/image_group_msg.hpp>
#include <vrm_protocol/vrm_cmd_msg.hpp>
#include <vrm_protocol/vrm_log_msg.hpp>
#include <vrm_global.h>

// Standard libraries
#include <string>
#include <vector>
#include <fstream>
#include <vector>
#include <sstream>

bool publish_images;

bool triggerCamera(vrm3dvision::triggerCamera::Request &req, vrm3dvision::triggerCamera::Response &res)
{
	res.success = true;

	publish_images = true;

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "zmq_image_publisher_node");
	ros::NodeHandle nh("/");

	// Load parameters
	ros::NodeHandle local_nh("~");
	int layers;
	std::string exposures_string;
	std::vector<int> exposures;
	bool cameras[3];
	std::string image_folder;
	std::string image_server_address;
	bool use_small_images;

	local_nh.param<bool>("has_left_camera", cameras[0], true);
	local_nh.param<bool>("has_color_camera", cameras[1], true);
	local_nh.param<bool>("has_right_camera", cameras[2], true);
	local_nh.param<bool>("small_images", use_small_images, true);


	local_nh.param<std::string>("image_folder", image_folder, "");

	local_nh.param<std::string>("image_server_address", image_server_address, "tcp://127.0.0.1:6001");
	local_nh.param<std::string>("exposures", exposures_string, std::string("-1"));

	local_nh.param<bool>("publish_on_startup", publish_images, false);

	local_nh.param<int>("no_of_layers_structured_light", layers, 8);

	// Parse parameters
	if (!image_folder.empty())
	{
		if (image_folder.at(image_folder.size()-1) != '/')
		{
			image_folder.push_back('/');
		}
		if (image_folder.at(0) != '/') // Use relative path
		{
			image_folder = ros::package::getPath("vrm3dvision") + std::string("/") + image_folder;
		}
	}
	else
	{
		ROS_ERROR_STREAM("No image folder specified!");
		return -1;
	}

	std::istringstream iss(exposures_string);
	ROS_INFO_STREAM("exposure_String: " << exposures_string);
	int temp;
	while(iss.peek() != EOF)
	{
		iss >> temp;
		exposures.push_back(temp);
	}

	for (size_t i = 0; i < exposures.size(); i++)
	{
		ROS_INFO_STREAM("EXP: " << i << ": " << exposures[i]);
	}

	// Load data
	std::vector<vrm_protocol::image_group> ig_vec;

	// Read first IG header
	std::string s;
	vrm_protocol::image_group ig_header;
	std::string pb_header_path = image_folder + "header_0_1.txt";
	std::ifstream pb_header (pb_header_path, std::ios::in | std::ios::binary);
	if(ig_header.header.ParseFromIstream(&pb_header))
	{
		ROS_INFO_STREAM("Image: " << (ig_header.header.image_id()+1) << " out of " << ig_header.header.num_images() << " images.");
	}
	else
	{
		ROS_ERROR_STREAM("Failed to parse PB header from IStream");
		return -1;
	}
	pb_header.close();

	if (exposures[0] < 0) // Use all exposures
	{
		ROS_INFO_STREAM("Using all exposures");
		exposures.clear();
		for (size_t i = 0; i < ig_header.header.num_exposures(); i++)
		{
			exposures.push_back(i+1);
		}
	}
	else if (ig_header.header.num_exposures() < exposures.size())
	{
		ROS_ERROR_STREAM("Trying to run with more exposures than data has.. - using all exposures");
		exposures.clear();
		for (size_t i = 0; i < ig_header.header.num_exposures(); i++)
		{
			exposures.push_back(i+1);
		}
	}

//	for (size_t i = 0; i < exposures.size(); i++)
//	{
//		if (exposures[i] > (int)ig_header.header.num_exposures())
//		{
//			ROS_ERROR_STREAM("Trying to use invalid exposure - invalid exposure is removed");
//			exposures.erase(exposures.begin()+i);
//		}
//	}

	if (cameras[0] && !ig_header.header.has_left_img())
	{
		ROS_ERROR_STREAM("Tried to run with left camera, but data was missing.. - running without left camera");
		cameras[0] = false;
	}
	if (cameras[1] && !ig_header.header.has_color_img())
	{
		ROS_ERROR_STREAM("Tried to run with center camera, but data was missing.. - running without center camera");
		cameras[1] = false;
	}
	if (cameras[2] && !ig_header.header.has_right_img())
	{
		ROS_ERROR_STREAM("Tried to run with right camera, but data was missing.. - running without right camera");
		cameras[2] = false;
	}

	if (layers < 0) // Use all layers
	{
		layers = (ig_header.header.num_images()) / 2;
	}
	else if (layers > ( (ig_header.header.num_images()) / 2) )
	{
		ROS_ERROR_STREAM("Trying to use " << layers << " layers, but data only supports " << ((ig_header.header.num_images()) / 2) << " layers - running with " << ((ig_header.header.num_images()) / 2) << " layers..");
		layers = (ig_header.header.num_images()) / 2;
	}
	int images_per_exp = layers*2;

	for (size_t exp = 0; exp < exposures.size(); exp++)
	{
		int exp_idx = exposures[exp];
		vrm_protocol::image_group ig;
		ig.header.set_cam_mode(ig_header.header.cam_mode());
		ig.header.set_num_images(images_per_exp);
		ig.header.set_exposure_id(exp+1);
		ig.header.set_num_exposures(exposures.size());
		ig.header.set_is_full_view(ig_header.header.is_full_view());
		if (ig_header.header.has_pattern_type())
		{
			ig.header.set_pattern_type(ig_header.header.pattern_type());
		}
		else
		{
			ROS_ERROR("Data doesn't have information about pattern type - pattern type is set to LGGC");
			ig.header.set_pattern_type(vrm_protocol::PATTERN_LARGE_GAP_GC);
		}

		// Test if global illumination image is present
		bool add_global_illumination = false;
		if (cameras[0])
		{
			ig.header.set_has_left_img(true);
			std::stringstream image_path;
			image_path << image_folder << "left_" << 60 << "_" << exp_idx << ".jpg";
			ig.left_image = cv::imread(image_path.str(),CV_LOAD_IMAGE_GRAYSCALE);

			if (!ig.left_image.empty())
			{
				if (use_small_images)
				{
					cv::resize(ig.left_image, ig.left_image, cv::Size(), 0.5, 0.5, CV_INTER_LINEAR);
				}
				add_global_illumination = true;
			}
		}
		else
		{
			ig.header.set_has_left_img(false);
		}
		if (cameras[1])
		{
			ig.header.set_has_color_img(true);
			std::stringstream image_path;
			image_path << image_folder << "color_" << 60 << "_" << exp_idx << ".jpg";
			ig.color_image = cv::imread(image_path.str(),CV_LOAD_IMAGE_GRAYSCALE);

			if (!ig.color_image.empty())
			{
				add_global_illumination = true;
				if (use_small_images)
				{
					cv::resize(ig.color_image, ig.color_image, cv::Size(), 0.5, 0.5, CV_INTER_LINEAR);
				}
			}
		}
		else
		{
			ig.header.set_has_color_img(false);
		}
		if (cameras[2])
		{
			ig.header.set_has_right_img(true);
			std::stringstream image_path;
			image_path << image_folder << "right_" << 60 << "_" << exp_idx << ".jpg";
			ig.right_image = cv::imread(image_path.str(),CV_LOAD_IMAGE_GRAYSCALE);

			if (!ig.right_image.empty())
			{
				if (use_small_images)
				{
					cv::resize(ig.right_image, ig.right_image, cv::Size(), 0.5, 0.5, CV_INTER_LINEAR);
				}
				add_global_illumination = true;
			}
		}
		else
		{
			ig.header.set_has_right_img(false);
		}
		if (add_global_illumination)
		{
			ig.header.set_has_ambient_img(true);
			ig.header.set_image_id(60);
			ig_vec.push_back(ig);
			ROS_INFO("Data has ambient light image - this is used");
		}
		else
		{
			ROS_WARN("Data doesn't have ambient light image - old occlusion mask will be used");

		}


		for (int img = 0; img < images_per_exp; img++)
		{
			ig.header.set_image_id(img);
			if (cameras[0])
			{
				ig.header.set_has_left_img(true);
				std::stringstream image_path;
				image_path << image_folder << "left_" << img << "_" << exp_idx << ".jpg";
				ig.left_image = cv::imread(image_path.str(),CV_LOAD_IMAGE_GRAYSCALE);
				if (use_small_images)
				{
					cv::resize(ig.left_image, ig.left_image, cv::Size(), 0.5, 0.5, CV_INTER_LINEAR);
				}
			}
			if (cameras[1])
			{
				ig.header.set_has_color_img(true);
				std::stringstream image_path;
				image_path << image_folder << "color_" << img << "_" << exp_idx << ".jpg";
				ig.color_image = cv::imread(image_path.str(),CV_LOAD_IMAGE_GRAYSCALE);
				if (use_small_images)
				{
					cv::resize(ig.color_image, ig.color_image, cv::Size(), 0.5, 0.5, CV_INTER_LINEAR);
				}
			}
			if (cameras[2])
			{
				ig.header.set_has_right_img(true);
				std::stringstream image_path;
				image_path << image_folder << "right_" << img << "_" << exp_idx << ".jpg";
				ig.right_image = cv::imread(image_path.str(),CV_LOAD_IMAGE_GRAYSCALE);
				if (use_small_images)
				{
					cv::resize(ig.right_image, ig.right_image, cv::Size(), 0.5, 0.5, CV_INTER_LINEAR);
				}
			}
			ig_vec.push_back(ig);
		}
	}


	vrm_protocol::pubsub_server<vrm_protocol::image_group> image_server;
	image_server.startup(image_server_address);

	ros::ServiceServer triggerCameraSrv = nh.advertiseService("triggerCamera", triggerCamera);

	ROS_INFO_STREAM("Streaming from path: " << image_folder << " using " << layers << " layers and " << exposures.size() << " exposures");

	// Sleep 0.25 seconds to make sure ZMQ is initialized
	ros::Duration(0.25).sleep();

	// Main loop
	bool exit = false;;
	while(!exit)
	{
		if (publish_images)
		{
			publish_images = false;
			ROS_INFO_STREAM("Starting image sequence!");
			for (size_t i = 0; i < ig_vec.size(); i++)
			{
				image_server.publish(ig_vec[i], 100);
				if (use_small_images)
				{
					ros::Duration(1.0/75.0).sleep();
				}
				else
				{
					ros::Duration(1.0/15.0).sleep();
				}
			}
		}
		if (!ros::ok())
		{
			exit = true;
		}
		if (!exit)
		{
			ros::spinOnce();
			ros::Duration(0.05).sleep();
		}
	}
	return 0;
}
