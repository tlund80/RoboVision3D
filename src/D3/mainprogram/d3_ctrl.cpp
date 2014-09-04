#include "d3_ctrl.hpp"

#define COLOR_EXP_GAIN 2.35
#define COLOR_EXP_OFFSET 2.5
#define COLOR_EXP(x) ((double)x*COLOR_EXP_GAIN + COLOR_EXP_OFFSET)

namespace vrm3dvision {

	bool D3Ctrl::shutdown_ = false;

	D3Ctrl::D3Ctrl()
	{
		streaming_state_ = StreamingState::INIT;
		random_dot_pattern_state_ = RandomDotPatternState::INIT;
		stripe_pattern_state_ = StripePatternState::INIT;
		hdr_state_ = HdrState::INIT;
		camera_trigger_ = false;
		sequence_id_ = -1;
		image_id_ = 0;
		exposure_id_ = 0;
		shutdown_ = false;
		add_ambient_image_ = false;
	}

	bool D3Ctrl::initialize()
	{
		// Initialize communication
		std::string command_server_address = "tcp://*:" + std::string(VRM_COMMAND_SERVER_PORT);
		std::string image_server_address = "tcp://*:" + std::string(VRM_IMAGE_SERVER_PORT);
		command_server_.startup(command_server_address);
		image_server_.startup(image_server_address);

		// Initialize camera
		ch_.initialize();

		// Initialize projector
		pc_.initialize();

		settings_.set_cam_mode(vrm_protocol::MODE_UNKNOWN);
		settings_.set_trig_mode(vrm_protocol::TRIG_UNKNOWN);

		return true;
	}

	bool D3Ctrl::setParameters(const vrm_protocol::CmdHeader& header)
	{
		bool ret = true;
		if(header.exposure_grey_size() > 0)
		{
			if (ch_.setExposure(PORT_LEFT_CAM,header.exposure_grey(0)) &&
					ch_.setExposure(PORT_RIGHT_CAM, header.exposure_grey(0))
					&& ch_.setExposure(PORT_COLOR_CAM, COLOR_EXP(header.exposure_grey(0))) )
			{
				settings_.clear_exposure_grey();
				for(size_t i = 0; i < header.exposure_grey_size(); i++)
				{
					settings_.add_exposure_grey(header.exposure_grey(i));
				}
				pc_.setExposure(int(COLOR_EXP(settings_.exposure_grey(0))*1000.0) + 1000);
				LOG_INFO("Updated exposure for all cameras. Number of exposures: " <<
						header.exposure_grey_size() << " with initial value: " << header.exposure_grey(0));

			// TODO Fix for projector exposures
			stripe_pattern_state_ = vrm3dvision::StripePatternState::INIT;

			}
			else
			{
				LOG_ERROR("Failed to set grey camera exposure");
				ret = false;
			}
		}

		if(header.gain_size() > 0)
		{
			LOG_WARNING("Gain adjustments not implemented yet");
		}

		if(header.has_frame_rate())
		{
			if (ch_.setFramerate(header.frame_rate()))
			{
				settings_.set_frame_rate(header.frame_rate());
				LOG_INFO("Frame rate updated");
			}
			else
			{
				LOG_ERROR("Failed to set framerate");
				ret = false;
			}
		}

		if(header.has_simple_occlusion())
		{
			settings_.set_simple_occlusion(header.simple_occlusion());
		}

		if(header.has_point_colors())
		{
			settings_.set_point_colors(header.point_colors());
		}

		if(header.has_pattern_partial_view() || header.has_pattern_resolution() || header.has_pattern_type())
		{
			settings_.set_pattern_partial_view(header.pattern_partial_view());
			settings_.set_pattern_resolution(header.pattern_resolution());
			settings_.set_pattern_type(header.pattern_type());
			parseCurrentPattern();
            pc_.startPattern(ProjectorPattern::GREY_CODE_INVERTED, pattern_splash_index_, 0, !settings_.simple_occlusion());
			std::string pattern_name = (settings_.pattern_type() == vrm_protocol::PATTERN_LARGE_GAP_GC ? "Large-Gap Gray Code" : "Conventional Gray Code");
			std::string view_name = (settings_.pattern_partial_view() ? "partial view" : "full view");

			LOG_INFO("Pattern changed to " << pattern_name << " with resolution " << settings_.pattern_resolution() << " in " << view_name << ".");
		}

		if(header.has_trig_mode())
		{
			if (header.trig_mode() != vrm_protocol::TRIG_UNKNOWN)
			{
				if(ch_.setTriggerMode(header.trig_mode()))
				{
					settings_.set_trig_mode(header.trig_mode());
					LOG_INFO("Trigger mode updated");
				}
				else
				{
					LOG_ERROR("Failed to change trigger mode");
					ret = false;
				}
			}
			else
			{
				LOG_WARNING("Unknown trigger mode");
				ret = false;
			}
		}


		if(header.has_cam_mode())
		{
			if (header.cam_mode() != vrm_protocol::MODE_UNKNOWN)
			{
				settings_.set_cam_mode(header.cam_mode());
				resetStateMachine();
				LOG_INFO("Camera mode updated");
			}
			else
			{
				LOG_WARNING("Unknown camera mode");
				ret = false;
			}
		}

		if(header.has_active_cams())
		{
			if (settings_.active_cams() != header.active_cams())
			{
				settings_.set_active_cams(header.active_cams());
				resetStateMachine();
				switch(header.active_cams())
				{
					case vrm_protocol::ActiveCams::CAMS_LEFT_CENTER_RIGHT:
						ch_.enableCameraPort(PORT_LEFT_CAM);
						ch_.enableCameraPort(PORT_COLOR_CAM);
						ch_.enableCameraPort(PORT_RIGHT_CAM);
						LOG_INFO("Enabled left/center/right cameras");
						break;
					case vrm_protocol::ActiveCams::CAMS_LEFT_RIGHT:
						ch_.enableCameraPort(PORT_LEFT_CAM);
						ch_.disableCameraPort(PORT_COLOR_CAM);
						ch_.enableCameraPort(PORT_RIGHT_CAM);
						LOG_INFO("Enabled left/right cameras and disabled center camera");
						break;
					case vrm_protocol::ActiveCams::CAMS_LEFT_CENTER:
						ch_.enableCameraPort(PORT_LEFT_CAM);
						ch_.enableCameraPort(PORT_COLOR_CAM);
						ch_.disableCameraPort(PORT_RIGHT_CAM);
						LOG_INFO("Enabled left/center cameras and disabled right camera");
						break;
					case vrm_protocol::ActiveCams::CAMS_CENTER_RIGHT:
						ch_.disableCameraPort(PORT_LEFT_CAM);
						ch_.enableCameraPort(PORT_COLOR_CAM);
						ch_.enableCameraPort(PORT_RIGHT_CAM);
						LOG_INFO("Enabled center/right cameras and disabled left camera");
						break;
					case vrm_protocol::ActiveCams::CAMS_NONE:
						ch_.disableCameraPort(PORT_LEFT_CAM);
						ch_.disableCameraPort(PORT_COLOR_CAM);
						ch_.disableCameraPort(PORT_RIGHT_CAM);
						LOG_INFO("Disabled left/center/right cameras");
						break;
				}
			}
		}

		return ret;
	}

	void D3Ctrl::resetStateMachine()
	{
		ch_.stopCamera();
		streaming_state_ = vrm3dvision::StreamingState::INIT;
		random_dot_pattern_state_ = vrm3dvision::RandomDotPatternState::INIT;
		hdr_state_ = vrm3dvision::HdrState::INIT;
		stripe_pattern_state_ = vrm3dvision::StripePatternState::INIT;
	}

	void D3Ctrl::parseCurrentPattern()
	{
		int temp_num = 10-settings_.pattern_resolution();
		pattern_num_levels_ = (settings_.pattern_partial_view() ? temp_num-1 : temp_num);

		if(settings_.pattern_type() == vrm_protocol::PATTERN_LARGE_GAP_GC)
		{
			int temp_idx = 17 + settings_.pattern_resolution();
			pattern_splash_index_ = (settings_.pattern_partial_view() ? temp_idx+3 : temp_idx);
		}
		else // Conventional Gray Code
		{
			if(settings_.pattern_partial_view())
			{
				pattern_splash_index_ = 16;
			}
			else
			{
				pattern_splash_index_ = 15;
			}
		}
	}



	void D3Ctrl::streamingState()
	{
		switch (streaming_state_)
		{
			case vrm3dvision::StreamingState::INIT:
				LOG_INFO("Initializing camera streaming");
				ch_.setTriggerMode(vrm_protocol::TRIG_INTERNAL);
				settings_.set_trig_mode(vrm_protocol::TRIG_INTERNAL);
				pc_.stopPattern();
				ch_.setExposure(PORT_LEFT_CAM, 35.0);
				ch_.setExposure(PORT_RIGHT_CAM, 35.0);
				ch_.setExposure(PORT_COLOR_CAM, COLOR_EXP(35.0));
				ch_.startCamera();
				streaming_state_ = vrm3dvision::StreamingState::WAIT_FOR_TRIGGER;
				break;
			case vrm3dvision::StreamingState::WAIT_FOR_TRIGGER:
				if((settings_.trig_mode() == vrm_protocol::TRIG_INTERNAL) || (camera_trigger_ == true))
				{
					camera_trigger_ = false;
					if(!ch_.triggerCamera())
					{
						LOG_ERROR("Failed to trigger camera");
					}
					streaming_state_ = vrm3dvision::StreamingState::TAKE_IMAGES;
				}
				else
				{
					usleep(10000);
				}
				break;
			case vrm3dvision::StreamingState::TAKE_IMAGES:
			if (ch_.readCamera(ig_,100) > 0)
				{
					ig_.header.set_cam_mode(vrm_protocol::MODE_STREAMING);
					image_server_.publish(ig_,1000);
				}
				if(settings_.trig_mode() != vrm_protocol::TRIG_INTERNAL)
				{
					streaming_state_ = vrm3dvision::StreamingState::WAIT_FOR_TRIGGER;
				}
				break;
			default:
				break;
		}
	}

	void D3Ctrl::hdrState()
	{

	}

	void D3Ctrl::randomDotPatternState()
	{
		switch (random_dot_pattern_state_)
		{
			case vrm3dvision::RandomDotPatternState::INIT:
				LOG_INFO("Initializing random dot pattern");
				ch_.setTriggerMode(vrm_protocol::TRIG_INTERNAL);
				settings_.set_trig_mode(vrm_protocol::TRIG_INTERNAL);
				ch_.setExposure(PORT_LEFT_CAM, 30.0);
				ch_.setExposure(PORT_RIGHT_CAM, 30.0);
				ch_.setExposure(PORT_COLOR_CAM, COLOR_EXP(30.0));
				pc_.setExposure(int(COLOR_EXP(30.0)*1000.0) + 1000);

				pc_.startPattern(ProjectorPattern::RANDOM_NOISE, 18, 0, false);
				ch_.startCamera();
				random_dot_pattern_state_ = vrm3dvision::RandomDotPatternState::WAIT_FOR_TRIGGER;
				break;
			case vrm3dvision::RandomDotPatternState::WAIT_FOR_TRIGGER:
				if((settings_.trig_mode() == vrm_protocol::TRIG_INTERNAL) || (camera_trigger_ == true))
				{
					camera_trigger_ = false;
					if(!ch_.triggerCamera())
					{
						LOG_ERROR("Failed to trigger camera");
					}
					random_dot_pattern_state_ = vrm3dvision::RandomDotPatternState::TAKE_IMAGES;
				}
				else
				{
					usleep(10000);
				}
				break;
			case vrm3dvision::RandomDotPatternState::TAKE_IMAGES:
				if (ch_.readCamera(ig_,100) > 0)
				{
					ig_.header.set_cam_mode(vrm_protocol::MODE_RANDOM_DOT_PATTERN);
					image_server_.publish(ig_,1000);
				}
				if(settings_.trig_mode() != vrm_protocol::TRIG_INTERNAL)
				{
					random_dot_pattern_state_ = vrm3dvision::RandomDotPatternState::WAIT_FOR_TRIGGER;
				}
				break;
			default:
				break;
		}
	}



	void D3Ctrl::stripePatternState()
	{
		switch (stripe_pattern_state_) {
			case vrm3dvision::StripePatternState::INIT:
				sequence_id_ = 0;
				exposure_id_ = 0;
				camera_trigger_ = false;
				settings_.set_trig_mode(vrm_protocol::TRIG_INTERNAL);
				ch_.setTriggerMode(settings_.trig_mode());

				/*if(!settings_.has_pattern_levels()) //TODO DEFAULT PATTERN
				{
					settings_.set_pattern_levels(8);
				}*/
				if(settings_.exposure_grey_size() == 0)
				{
					settings_.add_exposure_grey(3.0); // Add default exposure
				}

				// Avoid frame dropping at max frame rate
				//ch_.setMaxSupportedFramerate();
				settings_.set_frame_rate(7.0);
				ch_.setFramerate(settings_.frame_rate());
				stripe_pattern_state_ = vrm3dvision::StripePatternState::START_NEW_SEQUENCE;
				break;
			case vrm3dvision::StripePatternState::START_NEW_SEQUENCE:
				if(++exposure_id_ > settings_.exposure_grey_size())
				{
					sequence_id_++;
					exposure_id_ = 1;
				}
				image_id_ = 0;
				if(!settings_.simple_occlusion())
				{
					add_ambient_image_ = true;
				}
				else
				{
					add_ambient_image_ = false;
				}
				ch_.stopCamera();
				ch_.setExposure(PORT_LEFT_CAM, settings_.exposure_grey(exposure_id_-1));
				ch_.setExposure(PORT_RIGHT_CAM, settings_.exposure_grey(exposure_id_-1));
				ch_.setExposure(PORT_COLOR_CAM, COLOR_EXP(settings_.exposure_grey(exposure_id_-1)) );
				pc_.setExposure(int(COLOR_EXP(settings_.exposure_grey(exposure_id_-1))*1000.0) + 1000);
	            pc_.startPattern(ProjectorPattern::GREY_CODE_INVERTED, pattern_splash_index_, 0, add_ambient_image_);
	            stripe_pattern_state_ = vrm3dvision::StripePatternState::WAIT_FOR_TRIGGER;
				break;
			case vrm3dvision::StripePatternState::WAIT_FOR_TRIGGER:
				if (camera_trigger_ == true || exposure_id_ != 1)
				{
					stripe_pattern_state_ = vrm3dvision::StripePatternState::TAKE_IMAGES;
					ch_.startCamera();
					camera_trigger_ = false;
				}
				else
				{
					usleep(10000);
				}
				break;

			case vrm3dvision::StripePatternState::TAKE_IMAGES:
				if (image_id_ < (2*pattern_num_levels_  /*+ 2*/))
				{
					int val = ch_.readCamera(ig_,100);
					if (val == vrm3dvision::CamHandler::CAM_IMAGES_ACQUIRED)
					{
						if((image_id_ == 0) && (add_ambient_image_ == true) && (!settings_.simple_occlusion()))
						{
							add_ambient_image_ = false;
							ig_.header.set_image_id(60);
						}
						else
						{
							ig_.header.set_image_id(image_id_++);
						}
						ig_.header.set_cam_mode(vrm_protocol::MODE_STRIPE_PATTERN);
						ig_.header.set_sequence_id(sequence_id_);
						ig_.header.set_num_images(2*pattern_num_levels_ /*+ 2*/);
						ig_.header.set_exposure_id(exposure_id_);
						ig_.header.set_num_exposures(settings_.exposure_grey_size());
						ig_.header.set_is_full_view(!settings_.pattern_partial_view());
						ig_.header.set_pattern_type(settings_.pattern_type());
						ig_.header.set_has_ambient_img(!settings_.simple_occlusion());

						image_server_.publish(ig_,1000);
					}
					else if(val == vrm3dvision::CamHandler::CAM_IMAGES_DROPPED)
					{
						LOG_WARNING("Frames dropped. Trying to recover pattern from image " << image_id_);
						ch_.stopCamera();
						ch_.flushCameraBuffer();
						pc_.recoverPattern(ProjectorPattern::GREY_CODE_INVERTED, image_id_);
						ch_.startCamera();
					}
				}
				else
				{
					stripe_pattern_state_ = vrm3dvision::StripePatternState::START_NEW_SEQUENCE;
				}
				break;

			case vrm3dvision::StripePatternState::UPDATE_EXPOSURE:
				break;
			default:
				break;
		}
	}



	void D3Ctrl::sigHandler(int s)
	{
		LOG_INFO("D3 application shutdown..");
		shutdown_ = true;
		//exit(0);
	}



	void D3Ctrl::mainLoop()
	{
		// Setup sighandler
		struct sigaction sigIntHandler;
		sigIntHandler.sa_handler = sigHandler;
		sigemptyset(&sigIntHandler.sa_mask);
		sigIntHandler.sa_flags = 0;
		sigaction(SIGINT, &sigIntHandler, NULL);

		LOG_INFO("VRM D3 mainprogram started...");
		initialize();
		vrm_protocol::vrm_cmd cmd_msg;
		while(!shutdown_)
		{
			if (!command_server_.isClientConnected())
			{
				LOG_WARNING("No client connected - waiting for client...");
				resetStateMachine();
				command_server_.waitForClient(10000);
			}
			else
			{
				// Check for new commands
				if(command_server_.receive(cmd_msg))
				{
					switch(cmd_msg.header.command())
					{
						case vrm_protocol::CMD_SET_PARAMETERS:
							setParameters(cmd_msg.header);
							break;
						case vrm_protocol::CMD_TRIGGER_CAM:
							camera_trigger_ = true;
							break;
						case vrm_protocol::CMD_SEND_CALIB:
							LOG_INFO("Sending calibration data");
							break;
						case vrm_protocol::CMD_UPDATE_CALIB:
							LOG_INFO("Updating calibration data");
							break;
						case vrm_protocol::CMD_UNKNOWN:
							LOG_INFO("Unknown command");
							break;
					}
				}

				// Proceed from state of current camera mode
				ig_.header.Clear();
				switch(settings_.cam_mode())
				{
					case vrm_protocol::MODE_STREAMING:
						streamingState();
						break;
					case vrm_protocol::MODE_HDR:
						hdrState();
						break;
					case vrm_protocol::MODE_RANDOM_DOT_PATTERN:
						randomDotPatternState();
						break;
					case vrm_protocol::MODE_STRIPE_PATTERN:
						stripePatternState();
						break;
					case vrm_protocol::MODE_UNKNOWN:
					default:
						usleep(10000);
						//LOG_ERROR("Unknown camera mode");
						break;
				}
			}
		}
		command_server_.shutdown();
	}
}
