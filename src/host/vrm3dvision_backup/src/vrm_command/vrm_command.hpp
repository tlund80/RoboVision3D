/********************************************************************************************************************
 *
 * \file                vrm3dvision.hpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-22
 * \version             0.1
 * \brief               ROS handler for VRmagic D3 camera module
 *
*********************************************************************************************************************/
#ifndef _VRM_COMMAND_HPP_
#define _VRM_COMMAND_HPP_

// ROS includes
#include <ros/ros.h>
#include <ros/callback_queue.h>

// ZeroMQ
#include <zmq.h>

// VRM communication protocol
#include <vrm_protocol/reqrep.hpp>
#include <vrm_protocol/vrm_cmd_msg.hpp>
#include <vrm_global.h>

// Local includes

// ROS services
#include <vrm3dvision/setActiveCameras.h>
#include <vrm3dvision/setExposure.h>
#include <vrm3dvision/setFrameRate.h>
#include <vrm3dvision/setGain.h>
#include <vrm3dvision/setMode.h>
#include <vrm3dvision/setPattern.h>
#include <vrm3dvision/triggerCamera.h>
#include <vrm3dvision/setTriggerMode.h>

// Dynamic reconfigure

// Standard libraries
#include <string>
#include <vector>

namespace vrm3dvision {

	/** @class vrm3dvision Node
	  * VRM D3 communication/handler class for ROS
	  */
	class VrmCommand
	{
		public:
			/// Constructor
			VrmCommand(const ros::NodeHandle& node_handle);

			void mainLoop();
			void initialize();


		private:
			// ROS services
			bool setMode(vrm3dvision::setMode::Request &req,
					vrm3dvision::setMode::Response &res);

			bool setActiveCameras(vrm3dvision::setActiveCameras::Request &req,
					vrm3dvision::setActiveCameras::Response &res);


			bool setExposure(vrm3dvision::setExposure::Request &req,
					vrm3dvision::setExposure::Response &res);

			bool setGain(vrm3dvision::setGain::Request &req,
					vrm3dvision::setGain::Response &res);

			bool setFrameRate(vrm3dvision::setFrameRate::Request &req,
					vrm3dvision::setFrameRate::Response &res);

			bool setPattern(vrm3dvision::setPattern::Request &req,
					vrm3dvision::setPattern::Response &res);


			bool setTriggerMode(vrm3dvision::setTriggerMode::Request &req,
					vrm3dvision::setTriggerMode::Response &res);

			bool triggerCamera(vrm3dvision::triggerCamera::Request &req,
					vrm3dvision::triggerCamera::Response &res);

			bool parseActiveCameras(int value, vrm_protocol::vrm_cmd& cmd_msg);
			bool parseCameraMode(int value, vrm_protocol::vrm_cmd& cmd_msg);
			bool parseExposure(const std::string& exposure, vrm_protocol::vrm_cmd& cmd_msg);
			bool parseFrameRate(double value, vrm_protocol::vrm_cmd& cmd_msg);
			bool parsePattern(bool partial_view, int resolution, int type,
					bool use_simple_occlusion, bool add_point_colors, vrm_protocol::vrm_cmd& cmd_msg);
			bool parseTriggerMode(int value, vrm_protocol::vrm_cmd& cmd_msg);


			// ROS variables
			ros::NodeHandle nh_;
			ros::ServiceServer setModeSrv_;
			ros::ServiceServer setActiveCamerasSrv_;
			ros::ServiceServer setExposureSrv_;
			ros::ServiceServer setGainSrv_;
			ros::ServiceServer setFrameRateSrv_;
			ros::ServiceServer setPatternSrv_;
			ros::ServiceServer setTriggerModeSrv_;
			ros::ServiceServer triggerCameraSrv_;

			// ZeroMQ variables
			vrm_protocol::reqrep_client<vrm_protocol::vrm_cmd>* vrm_command_client_;

			std::string vrm_board_ip_;

			// Variables for initialization
			bool is_initialized_;
			int initial_mode_;
			int initial_active_cams_;
			std::string initial_exposure_;
			bool initial_pattern_partial_view_;
			int initial_pattern_resolution_;
			int initial_pattern_type_;
			double initial_frame_rate_;

			// Camera variables
			bool has_left_camera_;
			bool has_right_camera_;
			bool has_color_camera_;

			// Functions
	};

} /* namespace vrm3dvision */

#endif /* _VRM_COMMAND_HPP_ */
