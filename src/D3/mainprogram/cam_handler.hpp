#ifndef __CAM_HANDLER_INCLUDED_HPP__
#define __CAM_HANDLER_INCLUDED_HPP__

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <vector>

#include "vrmusbcam2.h"
#include "vrm_protocol/pubsub.hpp"
#include "vrm_protocol/image_group_msg.hpp"
#include "logger.hpp"
#include "timer.hpp"

namespace vrm3dvision {

#define MAX_PORTS 4
#define PORT_LEFT_CAM 1
#define PORT_COLOR_CAM 2
#define PORT_RIGHT_CAM 3
#define PORT_NO_CAM 4

#define CAM_DISABLED 0

	class CamHandler {
	public:
		/** Default constructor. */
		CamHandler();

		/** Destructor. */
		virtual ~CamHandler() { stopCamera(); }

		void initialize();
		void startCamera();
		void stopCamera();
		int readCamera(vrm_protocol::image_group& ig, int timeout_ms = 100);
		bool setExposure(unsigned int camera, float exposure);
		bool setGain(unsigned int camera, int gain);
		bool setFramerate(double framerate);
		bool setMaxSupportedFramerate();
		bool setTriggerMode(enum vrm_protocol::TrigMode triggermode);

		float getMaxSupportedFramerate();
		float getFramerate();

		bool triggerCamera();
		void flushCameraBuffer();
		VRmPropId getTriggerMode();

		void buildHDRf3mono(	const cv::Mat& img1,
								const cv::Mat& img2,
								const cv::Mat& img3,
								cv::Mat& hdrImage,
								float HDRFACTOR,
								float gamma);

		bool getExposure(unsigned int camera, float& exposure);

		void enableCameraPort(VRmDWORD camera_port);
		void disableCameraPort(VRmDWORD camera_port);

		enum ReturnType {
			CAM_NO_IMAGES_AVAILABLE = 0,
			CAM_IMAGES_ACQUIRED = 1,
			CAM_IMAGES_DROPPED = -1
		};

	private:
		void initDevice();
		void listAvailableCameras();

		void updateCameraPort(VRmDWORD camera_port, VRmBOOL enable_port);
		bool setPropertyPort(VRmUsbCamDevice device, unsigned int port);

		bool verifyFrameRate();

		VRmUsbCamDevice cam_device_;
		VRmBOOL port_active_[MAX_PORTS];
		VRmImageFormat image_formats_[MAX_PORTS];

	};

}

#endif // end of CAM_HANDLER_HPP
