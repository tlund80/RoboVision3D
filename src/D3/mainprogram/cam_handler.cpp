#include "cam_handler.hpp"

namespace vrm3dvision {
	// VRM helper function
	void LogExit()
	{
		LOG_ERROR("VRmUsbCam Error: " <<  VRmUsbCamGetLastError() << "\nApplication exit");
		exit(-1);
	}

	CamHandler::CamHandler()
	{
		cam_device_ = 0;
		for(VRmDWORD i = 0; i < MAX_PORTS; i++)
		{
			port_active_[i] = CAM_DISABLED;
		}
	}

	void CamHandler::initialize()
	{
		initDevice();
		listAvailableCameras();
	}

	void CamHandler::initDevice()
	{
		// at first, be sure to call VRmUsbCamCleanup() at exit, even in case
		// of an error
		atexit(VRmUsbCamCleanup);

		// check for connected devices
		VRmDWORD size=0;
		if(!VRmUsbCamGetDeviceKeyListSize(&size))
			LogExit();

		// open first usable device
		VRmDeviceKey* p_device_key=0;
		for(VRmDWORD i=0; i<size && !cam_device_; ++i)
		{
			if(!VRmUsbCamGetDeviceKeyListEntry(i,&p_device_key))
				LogExit();
			if(!p_device_key->m_busy) {
				if(!VRmUsbCamOpenDevice(p_device_key,&cam_device_))
					LogExit();
			}
			if(!VRmUsbCamFreeDeviceKey(&p_device_key))
				LogExit();
		}

		// display error when no camera has been found
		if(!cam_device_)
		{
			std::cerr << "No suitable VRmagic device found!" << std::endl;
			exit(-1);
		}

		// NOTE:
		// from now on, the "device" handle can be used to access the camera board.
		// use VRmUsbCamCloseDevice to end the usage

		// init camera, change some settings...
		// we get the format of the images in return, and which ports have cameras attached

	}



	void CamHandler::updateCameraPort(VRmDWORD camera_port, VRmBOOL enable_port)
	{
		VRmBOOL supported;
		VRmPropId sensor_enable = (VRmPropId)(VRM_PROPID_GRAB_SENSOR_ENABLE_1_B-1+camera_port);
		if(!VRmUsbCamGetPropertySupported(cam_device_,sensor_enable,&supported))
			LogExit();

		if(supported)
		{
			if(!VRmUsbCamSetPropertyValueB(cam_device_, sensor_enable, &enable_port))
				LogExit();

			port_active_[camera_port-1] = enable_port;

			VRmImageFormat source_format;
			if (!VRmUsbCamGetSourceFormatEx(cam_device_, camera_port,&source_format))
				LogExit();

			image_formats_[camera_port-1] = source_format;

			if(!VRmUsbCamResetFrameCounter(cam_device_))
				LogExit();

			if(!VRmUsbCamRestartTimer())
				LogExit();
		}
	}



	void CamHandler::enableCameraPort(VRmDWORD camera_port)
	{
		updateCameraPort(camera_port, VRmBOOL(true));
	}

	void CamHandler::disableCameraPort(VRmDWORD camera_port)
	{
		updateCameraPort(camera_port, VRmBOOL(false));
	}



	void CamHandler::listAvailableCameras()
	{
		VRmDWORD num_sensor_ports = 0;
		if(!VRmUsbCamGetSensorPortListSize(cam_device_,&num_sensor_ports))
			LogExit();

		LOG_INFO("Number of available sensor ports: " << num_sensor_ports);

		// List available cameras
		for(VRmDWORD i=0; i<num_sensor_ports; i++)
		{
			VRmDWORD tmp_port = 0;
			if(!VRmUsbCamGetSensorPortListEntry(cam_device_, i, &tmp_port))
				LogExit();

			switch(tmp_port)
			{
				case PORT_LEFT_CAM:
					LOG_INFO("Left camera available on port " << tmp_port);
					break;
				case PORT_RIGHT_CAM:
					LOG_INFO("Right camera available on port " << tmp_port);
					break;
				case PORT_COLOR_CAM:
					LOG_INFO("Color camera available on port " << tmp_port);
					break;
				default:
					LOG_INFO("Unknown camera available on port " << tmp_port);
					break;
			}
		}
	}



	void CamHandler::startCamera()
	{
		// start grabber
		VRmUsbCamStart(cam_device_);
		//if(!VRmUsbCamStart(cam_device_))
		//	LogExit();
	}

	void CamHandler::stopCamera()
	{
		// stop grabber
		VRmUsbCamStop(cam_device_);
		//if(!VRmUsbCamStop(cam_device_))
		//	LogExit();
		flushCameraBuffer();
	}

	void CamHandler::flushCameraBuffer()
	{
		VRmImage* p_source_img = 0;
		VRmDWORD allPorts = 0;
		VRmDWORD frames_dropped = 0;
		while(VRmUsbCamLockNextImageEx2(cam_device_,allPorts,&p_source_img,&frames_dropped,10));
	}

	int CamHandler::readCamera(vrm_protocol::image_group& ig, int timeout_ms)
	{
		int return_value = CAM_NO_IMAGES_AVAILABLE;
		bool imagesAvailable = true;
		bool all_images_captured = false;
		int current_frame_counter = 0;

		while(!all_images_captured && imagesAvailable)
		{
			// lock next (raw) image for read access, convert it to the desired
			// format and unlock it again, so that grabbing can
			// go on
			VRmImage* p_source_img = 0;
			VRmDWORD allPorts = 0;
			VRmDWORD frames_dropped = 0;
			imagesAvailable = VRmUsbCamLockNextImageEx2(cam_device_,allPorts,&p_source_img,&frames_dropped,timeout_ms);
			if(imagesAvailable)
			{

				// VRmUsbCamLockNextImageEx2() successfully returned an image
				// ----------------------------------------------------------
				VRmDWORD img_sensorport;
				VRmUsbCamGetImageSensorPort(p_source_img,&img_sensorport);

				VRmImageFormat target_format = image_formats_[img_sensorport-1];

				VRmImage* p_target_img = 0;

				// Copy image
				if(!VRmUsbCamCopyImage(&p_target_img,p_source_img))
					LogExit();

				VRmDWORD frame_counter;
				if(!VRmUsbCamGetFrameCounter(p_source_img,&frame_counter))
					LogExit();

				if(!VRmUsbCamUnlockNextImage(cam_device_,&p_source_img))
					LogExit();

				// see, if we had to drop some frames due to data transfer stalls. if so,
				// output a message
				if (frames_dropped)
				{
					LOG_WARNING("- " << frames_dropped <<  " frame(s) dropped -");
					return_value = CAM_IMAGES_DROPPED;
					//TODO Drop all frames in current set
				}
				else
				{
					// Make cv::Mat header for the recieved image (This does not copy data, hence the use of Clone() later)
					cv::Mat img = cv::Mat(cv::Size(p_target_img->m_image_format.m_width,p_target_img->m_image_format.m_height),CV_8UC1,p_target_img->mp_buffer, cv::Mat::AUTO_STEP);
					if (img_sensorport == PORT_LEFT_CAM)
					{
						ig.left_image = img.clone();
						ig.header.set_has_left_img(true);
						if ((port_active_[PORT_RIGHT_CAM-1] == CAM_DISABLED || ig.header.has_right_img()) && (port_active_[PORT_COLOR_CAM-1] == CAM_DISABLED || ig.header.has_color_img()))
						{
							all_images_captured = true;
						}
					}
					else if (img_sensorport == PORT_RIGHT_CAM)
					{
						ig.right_image = img.clone();
						ig.header.set_has_right_img(true);
						if ((port_active_[PORT_LEFT_CAM-1] == CAM_DISABLED || ig.header.has_left_img()) && (port_active_[PORT_COLOR_CAM-1] == CAM_DISABLED || ig.header.has_color_img()))
						{
							all_images_captured = true;
						}

					}
					else if (img_sensorport == PORT_COLOR_CAM)
					{
						ig.color_image = img.clone();
						ig.header.set_has_color_img(true);
						if ((port_active_[PORT_RIGHT_CAM-1] == CAM_DISABLED || ig.header.has_right_img()) && (port_active_[PORT_LEFT_CAM-1] == CAM_DISABLED || ig.header.has_left_img()))
						{
							all_images_captured = true;
						}
					}
				}
				// free the resources of the target image
				if(!VRmUsbCamFreeImage(&p_target_img))
					LogExit();
			}else{

				// VRmUsbCamLockNextImageEx2() did not return an image. why?
				// ----------------------------------------------------------

				int error_code = VRmUsbCamGetLastErrorCode();
				switch(VRmUsbCamGetLastErrorCode())
				{
					case VRM_ERROR_CODE_FUNCTION_CALL_TIMEOUT:
					case VRM_ERROR_CODE_TRIGGER_TIMEOUT:
					case VRM_ERROR_CODE_TRIGGER_STALL:
						return_value = CAM_NO_IMAGES_AVAILABLE;
						//LOG_ERROR("VRmUsbCamLockNextImageEx2() failed with " << VRmUsbCamGetLastError());
						break;

					case VRM_ERROR_CODE_GENERIC_ERROR:
					default:
						LogExit();
						break;
				}
			}
		}
		if(all_images_captured)
			return_value = CAM_IMAGES_ACQUIRED;
		return return_value;
	}

	bool CamHandler::setPropertyPort(VRmUsbCamDevice device, unsigned int port)
	{
		unsigned int r;

		if(!VRmUsbCamGetPropertySupported(device,VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E,&r))
				LogExit();
		if(r)
		{
			VRmPropId tmp = VRmPropId(VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_1-1+port);
			if(!VRmUsbCamSetPropertyValueE(device,VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E,&tmp))
				LogExit();
		}

		return (bool)r;
	}

	bool CamHandler::getExposure(unsigned int camera, float& exposure)
	{
		bool ret = false;
		VRmBOOL supported = setPropertyPort(cam_device_, camera);
		if (!supported)
			LogExit();

		if(!VRmUsbCamGetPropertySupported(cam_device_,VRM_PROPID_CAM_EXPOSURE_TIME_F,&supported))
			LogExit();
		if(supported)
		{
			if(!VRmUsbCamGetPropertyValueF(cam_device_,VRM_PROPID_CAM_EXPOSURE_TIME_F,&exposure))
				LogExit();
			ret = true;
		}

		return ret;
	}

	bool CamHandler::triggerCamera()
	{
		bool ret = VRmUsbCamSoftTrigger(cam_device_);
		return ret;
	}

	bool CamHandler::setExposure(unsigned int camera, float exposure)
	{
		bool ret = false;
		VRmBOOL supported = setPropertyPort(cam_device_, camera);
		if (!supported)
			LogExit();

		if(!VRmUsbCamGetPropertySupported(cam_device_,VRM_PROPID_CAM_EXPOSURE_TIME_F,&supported))
			LogExit();
		if(supported)
		{
			if(!VRmUsbCamSetPropertyValueF(cam_device_,VRM_PROPID_CAM_EXPOSURE_TIME_F,&exposure))
				LogExit();
			ret = true;
		}
		// Scale framerate if too high
		verifyFrameRate();
		return ret;
	}

	bool CamHandler::setGain(unsigned int camera, int gain)
	{
		bool ret = false;
		VRmBOOL supported = setPropertyPort(cam_device_, camera);
		if (!supported)
			LogExit();

		if(!VRmUsbCamGetPropertySupported(cam_device_,VRM_PROPID_CAM_GAIN_MONOCHROME_I,&supported))
			LogExit();
		if(supported)
		{
			// uncomment the following lines to change exposure time to 25ms
			// when camera supports this feature
			if(!VRmUsbCamSetPropertyValueI(cam_device_,VRM_PROPID_CAM_GAIN_MONOCHROME_I,&gain))
				LogExit();
			ret = true;
		}
		return ret;
	}

	bool CamHandler::setFramerate(double framerate)
	{

		bool ret = false;
		float new_frame_rate = framerate;
		float max_frame_rate = getMaxSupportedFramerate();

		if (max_frame_rate < new_frame_rate)
		{
			LOG_WARNING("Framerate too high - framerate will be set to maximum allowed! - which is: " << max_frame_rate);
			new_frame_rate = max_frame_rate;
		}
		VRmBOOL supported;
		if(!VRmUsbCamGetPropertySupported(cam_device_,VRM_PROPID_CAM_INTERNAL_TRIGGER_RATE_F,&supported))
			LogExit();
		if(supported)
		{
			// uncomment the following lines to change exposure time to 25ms
			// when camera supports this feature
			if(!VRmUsbCamSetPropertyValueF(cam_device_,VRM_PROPID_CAM_INTERNAL_TRIGGER_RATE_F,&new_frame_rate))
				LogExit();
			ret = true;
		}
		return ret;
	}

	bool CamHandler::setMaxSupportedFramerate()
	{
		return setFramerate(getMaxSupportedFramerate());
	}

	bool CamHandler::verifyFrameRate()
	{
		bool ret = true;
		float max_frame_rate = getMaxSupportedFramerate();
		float frame_rate = getFramerate();
		if (frame_rate > max_frame_rate)
		{
			LOG_WARNING("Framerate changed to: " << max_frame_rate << " from: " << frame_rate << " due to changed exposure");
			setFramerate(max_frame_rate);
			ret = false;
		}
		return ret;
	}

	float CamHandler::getMaxSupportedFramerate()
	{

		float max_frame_rate;
		if(!VRmUsbCamGetPropertyValueF(cam_device_,VRM_PROPID_CAM_ACQUISITION_RATE_MAX_F,&max_frame_rate))
			LogExit();

		return max_frame_rate;
	}

	float CamHandler::getFramerate()
	{

		float frame_rate;
		if(!VRmUsbCamGetPropertyValueF(cam_device_,VRM_PROPID_CAM_INTERNAL_TRIGGER_RATE_F,&frame_rate))
			LogExit();

		return frame_rate;
	}

	bool CamHandler::setTriggerMode(enum vrm_protocol::TrigMode triggermode)
	{
		VRmPropId tmp;
		if (triggermode == vrm_protocol::TRIG_INTERNAL)
		{
			tmp = VRM_PROPID_GRAB_MODE_TRIGGERED_INTERNAL;
		}
		else if (triggermode == vrm_protocol::TRIG_EXTERNAL)
		{
			tmp = VRM_PROPID_GRAB_MODE_TRIGGERED_EXT;
		}
		else if (triggermode == vrm_protocol::TRIG_SOFTWARE)
		{
			tmp = VRM_PROPID_GRAB_MODE_TRIGGERED_SOFT;
		}
		else
		{
			LOG_ERROR("CamHandler: Unknown triggermode");
			return false;
		}

		if(!VRmUsbCamSetPropertyValueE(cam_device_,VRM_PROPID_GRAB_MODE_E,&tmp))
			LogExit();

		return true;
	}

	VRmPropId CamHandler::getTriggerMode()
	{
		VRmPropId ret;
		if(!VRmUsbCamGetPropertyValueE(cam_device_,VRM_PROPID_GRAB_MODE_E,&ret))
			LogExit();

		return ret;
	}

	void CamHandler::buildHDRf3mono(const cv::Mat& img1,
						const cv::Mat& img2,
						const cv::Mat& img3,
						cv::Mat& hdrImage,
						float HDRFACTOR,
						float gamma)
	{
		int w = img1.cols;
		int h = img1.rows;
		int p1;
		int bright[3];
		int weight[3];
		float b;

		hdrImage = img1.clone();

		for (int y =0;y < h;y++) for (int x =0;x < w;x++)
		{
			p1=(y*w+x);
			bright[0] = (int) img1.at<uchar>(p1);
			bright[1] = (int) img2.at<uchar>(p1);
			bright[2] = (int) img3.at<uchar>(p1);

			// Compute weights
			for (int j = 0; j < 3; j++)
			{
				if (bright[j] < 128)
				{
					weight[j] = bright[j];
					if (weight[j] < 5)
						weight[j] = 0;
				}
				else
				{
					weight[j] = 255 - bright[j];
				}
			}

			// BUILD HDR FLOAT IMAGE
			if (weight[0] + weight[1] + weight[2]==0)
			{
				if (bright[1] < 128)
					weight[2]=1;
				else
					weight[0]=1;
			}

			b =(((float)weight[0] * ((float)bright[0]) + (float)weight[1] * ((float)bright[1]) / HDRFACTOR +
					(float)weight[2] * ((float)bright[2]) / (HDRFACTOR * HDRFACTOR) ) /
					((float)weight[2] + (float)weight[1] + (float)weight[0])) / 256.0;

			// SHOW HDR IMAGES IN UCHAR IMG
			if (b)
			{
				hdrImage.at<uchar>(p1) = (uint8_t)(std::min(1.0,pow(b,gamma))*255.0);
			}
			else
			{
				hdrImage.at<uchar>(p1) = 0;
			}
		}
	}

} // end of namespace vrm3dvision
