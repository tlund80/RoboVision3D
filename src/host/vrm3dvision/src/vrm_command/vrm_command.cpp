/********************************************************************************************************************
 *
 * \file                vrm3dvision.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-22
 * \version             0.1
 * \brief               ROS handler for VRmagic D3 camera module
 *
*********************************************************************************************************************/
#include "vrm_command.hpp"

namespace vrm3dvision {

	VrmCommand::VrmCommand(const ros::NodeHandle& nodeHandle)
	 : nh_(nodeHandle)
	{
		// Read parameter from launch file
		ros::NodeHandle local_nh("~");
		local_nh.param<std::string>("vrm_board_ip", vrm_board_ip_, "192.168.50.10");
		local_nh.param<bool>("has_left_camera", has_left_camera_, true);
		local_nh.param<bool>("has_right_camera", has_right_camera_, true);
		local_nh.param<bool>("has_color_camera", has_color_camera_, true);
		local_nh.param<int>("initial_mode", initial_mode_, vrm3dvision::setMode::Request::STREAMING);
		local_nh.param<int>("initial_active_cams", initial_active_cams_, vrm3dvision::setActiveCameras::Request::LEFT_CENTER_RIGHT);
		local_nh.param<std::string>("initial_expo_grey", initial_exposure_, std::string("3.0"));
		local_nh.param<bool>("initial_pattern_partial_view", initial_pattern_partial_view_, false);
		local_nh.param<int>("initial_pattern_resolution", initial_pattern_resolution_, vrm3dvision::setPattern::Request::LARGE_GAP_GC);
		local_nh.param<int>("initial_pattern_type", initial_pattern_type_, 0);
		local_nh.param<double>("initial_frame_rate", initial_frame_rate_, 5.0);

		is_initialized_ = false;
		vrm_command_client_ = new vrm_protocol::reqrep_client<vrm_protocol::vrm_cmd>();
	}


	void VrmCommand::initialize()
	{
		std::string vrm_command_server_address = "tcp://" + vrm_board_ip_ + ":" + std::string(VRM_COMMAND_SERVER_PORT);
		ROS_INFO("Initializing command client with address: %s", vrm_command_server_address.c_str());
		vrm_command_client_->startup(vrm_command_server_address);

		setModeSrv_ = nh_.advertiseService("setMode", &VrmCommand::setMode, this);
		setActiveCamerasSrv_= nh_.advertiseService("setActiveCameras", &VrmCommand::setActiveCameras, this);
		setExposureSrv_ = nh_.advertiseService("setExposure", &VrmCommand::setExposure, this);
		setGainSrv_ = nh_.advertiseService("setGain", &VrmCommand::setGain, this);
		setFrameRateSrv_ = nh_.advertiseService("setFrameRate", &VrmCommand::setFrameRate, this);
		setPatternSrv_ = nh_.advertiseService("setPattern", &VrmCommand::setPattern, this);
		setTriggerModeSrv_ = nh_.advertiseService("setTriggerMode", &VrmCommand::setTriggerMode, this);
		triggerCameraSrv_ = nh_.advertiseService("triggerCamera", &VrmCommand::triggerCamera, this);
	}



	bool VrmCommand::setMode(vrm3dvision::setMode::Request &req, vrm3dvision::setMode::Response &res)
	{
		vrm_protocol::vrm_cmd cmd_msg;
		res.success = parseCameraMode(req.type, cmd_msg);
		if(res.success)
			vrm_command_client_->publish(cmd_msg,0);
		return true;
	}



	bool VrmCommand::setActiveCameras(vrm3dvision::setActiveCameras::Request &req, vrm3dvision::setActiveCameras::Response &res)
	{
		vrm_protocol::vrm_cmd cmd_msg;
		res.success = parseActiveCameras(req.cameras, cmd_msg);
		if(res.success)
			vrm_command_client_->publish(cmd_msg,0);
		return true;
	}



	bool VrmCommand::setExposure(vrm3dvision::setExposure::Request &req, vrm3dvision::setExposure::Response &res)
	{
		vrm_protocol::vrm_cmd cmd_msg;
		res.success = parseExposure(req.exposure, cmd_msg);
		if(res.success)
			vrm_command_client_->publish(cmd_msg,0);
		return true;
	}



	bool VrmCommand::setGain(vrm3dvision::setGain::Request &req, vrm3dvision::setGain::Response &res)
	{
		if (req.gain >= VRM_EXPOSURE_MIN && req.gain <= VRM_EXPOSURE_MAX)
		{
			vrm_protocol::vrm_cmd cmd_msg;
			cmd_msg.header.set_command(vrm_protocol::CMD_SET_PARAMETERS);
//			cmd_msg.command = vrm_protocol::vrm_command::SET_GAIN;
//			if (req.camera == vrm3dvision::setExposure::Request::LEFT_CAMERA)
//				cmd_msg.left_camera_settings.gain = req.gain;
//			else if (req.camera == vrm3dvision::setExposure::Request::RIGHT_CAMERA)
//				cmd_msg.right_camera_settings.gain = req.gain;
//			else if (req.camera == vrm3dvision::setExposure::Request::COLOR_CAMERA)
//				cmd_msg.color_camera_settings.gain = req.gain;

			vrm_command_client_->publish(cmd_msg,0);
			res.success = true;
		}
		else
		{
			res.success = false;
			ROS_ERROR_STREAM("Failed to set gain - Gain has to be between " << VRM_GAIN_MIN << " and " << VRM_GAIN_MAX << ".");
		}
		return true;
	}



	bool VrmCommand::setFrameRate(vrm3dvision::setFrameRate::Request &req, vrm3dvision::setFrameRate::Response &res)
	{
		vrm_protocol::vrm_cmd cmd_msg;
		res.success = parseFrameRate(req.frame_rate, cmd_msg);
		if(res.success)
			vrm_command_client_->publish(cmd_msg,0);
		return true;
	}



	bool VrmCommand::setPattern(vrm3dvision::setPattern::Request &req, vrm3dvision::setPattern::Response &res)
	{
		vrm_protocol::vrm_cmd cmd_msg;
		res.success = parsePattern(req.partial_view, req.resolution, req.type,
				req.use_simple_occlusion, req.add_point_colors, cmd_msg);
		if(res.success)
			vrm_command_client_->publish(cmd_msg,0);

		return true;
	}



	bool VrmCommand::setTriggerMode(vrm3dvision::setTriggerMode::Request &req, vrm3dvision::setTriggerMode::Response &res)
	{
		vrm_protocol::vrm_cmd cmd_msg;
		res.success = parseTriggerMode(req.type, cmd_msg);
		if(res.success)
			vrm_command_client_->publish(cmd_msg,0);

		return true;
	}



	bool VrmCommand::triggerCamera(vrm3dvision::triggerCamera::Request &req, vrm3dvision::triggerCamera::Response &res)
	{
		vrm_protocol::vrm_cmd cmd_msg;
		cmd_msg.header.set_command(vrm_protocol::CMD_TRIGGER_CAM);
		vrm_command_client_->publish(cmd_msg,0);
		res.success = true;

		return true;
	}



	bool VrmCommand::parseActiveCameras(int value, vrm_protocol::vrm_cmd& cmd_msg)
	{
		bool ret = true;
		cmd_msg.header.set_command(vrm_protocol::CMD_SET_PARAMETERS);

		switch(value)
		{
			case vrm3dvision::setActiveCameras::Request::LEFT_CENTER_RIGHT:
				cmd_msg.header.set_active_cams(vrm_protocol::CAMS_LEFT_CENTER_RIGHT);
				break;
			case vrm3dvision::setActiveCameras::Request::LEFT_RIGHT:
				cmd_msg.header.set_active_cams(vrm_protocol::CAMS_LEFT_RIGHT);
				break;
			case vrm3dvision::setActiveCameras::Request::LEFT_CENTER:
				cmd_msg.header.set_active_cams(vrm_protocol::CAMS_LEFT_CENTER);
				break;
			case vrm3dvision::setActiveCameras::Request::CENTER_RIGHT:
				cmd_msg.header.set_active_cams(vrm_protocol::CAMS_CENTER_RIGHT);
				break;
			case vrm3dvision::setActiveCameras::Request::NONE:
				cmd_msg.header.set_active_cams(vrm_protocol::CAMS_NONE);
				break;
			default:
				ret = false;
				ROS_ERROR_STREAM("Unknown set of active cameras");
				break;
		}
		return ret;
	}



	bool VrmCommand::parseExposure(const std::string& exposure, vrm_protocol::vrm_cmd& cmd_msg)
	{
		cmd_msg.header.set_command(vrm_protocol::CMD_SET_PARAMETERS);

		std::istringstream iss(exposure);
		double temp;
		while(iss.peek() != EOF)
		{
			iss >> temp;
			if (temp >= VRM_EXPOSURE_MIN && temp <= VRM_EXPOSURE_MAX)
			{
				cmd_msg.header.add_exposure_grey(temp);
			}
			else
			{
				ROS_ERROR_STREAM("Failed to set exposure. Exposure has to be between " << VRM_EXPOSURE_MIN << " and " << VRM_EXPOSURE_MAX << ".");
				cmd_msg.header.clear_exposure_grey();
				return false;
			}
		}
		return true;
	}



	bool VrmCommand::parseFrameRate(double value, vrm_protocol::vrm_cmd& cmd_msg)
	{
		bool ret = false;
		if (value >= VRM_FRAMERATE_MIN && value <= VRM_FRAMERATE_MAX)
		{
			cmd_msg.header.set_command(vrm_protocol::CMD_SET_PARAMETERS);
			cmd_msg.header.set_frame_rate(value);
			ret = true;
		}
		else
		{
			ROS_ERROR_STREAM("Failed to set framerate - Framerate has to be between " << VRM_FRAMERATE_MIN << " and " << VRM_FRAMERATE_MAX << ".");
		}
		return ret;
	}



	bool VrmCommand::parsePattern(bool partial_view, int resolution, int type,
			bool use_simple_occlusion, bool add_point_colors, vrm_protocol::vrm_cmd& cmd_msg)
	{
		bool ret = false;
		cmd_msg.header.set_command(vrm_protocol::CMD_SET_PARAMETERS);
		if(resolution >= 0 && resolution < 3 &&
				(type == vrm3dvision::setPattern::Request::LARGE_GAP_GC || type == vrm3dvision::setPattern::Request::CONVENTIONAL_GC))
		{
			if(type == vrm3dvision::setPattern::Request::LARGE_GAP_GC)
			{
				cmd_msg.header.set_pattern_type(vrm_protocol::PATTERN_LARGE_GAP_GC);
			}
			else
			{
				cmd_msg.header.set_pattern_type(vrm_protocol::PATTERN_CONV_GC);

			}
			cmd_msg.header.set_pattern_partial_view(partial_view);
			cmd_msg.header.set_pattern_resolution(resolution);
			cmd_msg.header.set_simple_occlusion(use_simple_occlusion);
			cmd_msg.header.set_point_colors(add_point_colors);
			ret = true;
		}
		else
		{
			ROS_ERROR_STREAM("Failed to parse pattern. Partial view: " << partial_view << " Resolution: " << resolution << " Type: " << type);
		}

		return ret;
	}



	bool VrmCommand::parseTriggerMode(int value, vrm_protocol::vrm_cmd& cmd_msg)
	{
		bool ret = true;
		cmd_msg.header.set_command(vrm_protocol::CMD_SET_PARAMETERS);

		switch(value)
		{
			case vrm3dvision::setTriggerMode::Request::INTERNAL_TRIGGER:
				cmd_msg.header.set_trig_mode(vrm_protocol::TRIG_INTERNAL);
				break;
			case vrm3dvision::setTriggerMode::Request::EXTERNAL_TRIGGER:
				cmd_msg.header.set_trig_mode(vrm_protocol::TRIG_EXTERNAL);
				break;
			case vrm3dvision::setTriggerMode::Request::SOFT_TRIGGER:
				cmd_msg.header.set_trig_mode(vrm_protocol::TRIG_SOFTWARE);
				break;
			default:
				ret = false;
				ROS_ERROR_STREAM("Unknown trigger mode");
				break;
		}
		return ret;
	}



	bool VrmCommand::parseCameraMode(int value, vrm_protocol::vrm_cmd& cmd_msg)
	{
		bool ret = true;
		cmd_msg.header.set_command(vrm_protocol::CMD_SET_PARAMETERS);

		switch(value)
		{
			case vrm3dvision::setMode::Request::STREAMING:
				cmd_msg.header.set_cam_mode(vrm_protocol::MODE_STREAMING);
				break;
			case vrm3dvision::setMode::Request::HDR:
				cmd_msg.header.set_cam_mode(vrm_protocol::MODE_HDR);
				break;
			case vrm3dvision::setMode::Request::RANDOM_DOT_PATTERN:
				cmd_msg.header.set_cam_mode(vrm_protocol::MODE_RANDOM_DOT_PATTERN);
				break;
			case vrm3dvision::setMode::Request::PATTERN:
				cmd_msg.header.set_cam_mode(vrm_protocol::MODE_STRIPE_PATTERN);
				break;
			default:
				ROS_ERROR_STREAM("Unknown camera mode");
				ret = false;
				break;
		}
		return ret;
	}



	void VrmCommand::mainLoop()
	{
		initialize();
		while (ros::ok())
		{
			vrm_protocol::vrm_cmd cmd_msg;

			if(vrm_command_client_->isConnected() && is_initialized_)
			{
				if(vrm_command_client_->receive(cmd_msg))
				{
					ROS_INFO_STREAM("Received message from D3");
				}
				// Spin for the specified duration
				ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
			}
			else if(vrm_command_client_->isConnected() && !is_initialized_)
			{
				cmd_msg.header.set_timestamp(ros::Time::now().toSec());
				parseActiveCameras(initial_active_cams_, cmd_msg);
				parseCameraMode(initial_mode_, cmd_msg);
				parseExposure(initial_exposure_, cmd_msg);
				parseFrameRate(initial_frame_rate_, cmd_msg);
				parsePattern(initial_pattern_partial_view_, initial_pattern_resolution_, initial_pattern_type_, false, false, cmd_msg);
				ROS_INFO_STREAM("Sending initial command");
				vrm_command_client_->publish(cmd_msg,0);
				is_initialized_ = true;
			}
			else
			{
				is_initialized_= false;
				if(vrm_command_client_->connect(5000))
				{
					ROS_INFO_STREAM("Successfully connected to D3 board");
				}
				else
				{
					ROS_WARN_STREAM("Failed to connected to D3 board");
				}
			}
		}
		vrm_command_client_->shutdown();
	}
} /* namespace vrm3dvision */
