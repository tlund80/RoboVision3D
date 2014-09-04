/********************************************************************************************************************
 *
 * \file                d3_ctrl.hpp
 * \author              Kent Hansen (kenh@teknologisk.dk)
 * \date                2014-02-18
 * \version             1.0
 * \brief               Main class for controlling features on D3 board
 *
*********************************************************************************************************************/

#ifndef __D3_CTRL_HPP__
#define __D3_CTRL_HPP__

#include <signal.h>

#include <vrm_protocol/pubsub.hpp>
#include <vrm_protocol/reqrep.hpp>
#include <vrm_protocol/vrm_cmd_msg.hpp>
#include <vrm_protocol/image_group_msg.hpp>
#include "ProjectorCtrl/ProjectorCtrl.hpp"
#include "cam_handler.hpp"
#include "logger.hpp"


namespace vrm3dvision {

enum class HdrState {
	INIT = 1,
	TAKE_IMAGES,
	PROCESS_AND_PUBLISH_IMAGES
};

enum class StreamingState {
	INIT = 1,
	WAIT_FOR_TRIGGER,
	TAKE_IMAGES
};

enum class RandomDotPatternState {
	INIT = 1,
	WAIT_FOR_TRIGGER,
	TAKE_IMAGES
};

enum class StripePatternState {
	INIT = 1,
	WAIT_FOR_TRIGGER,
	START_NEW_SEQUENCE,
	TAKE_IMAGES,
	UPDATE_EXPOSURE
};

class D3Ctrl {
public:
	/** Default constructor. */
	D3Ctrl();

	/** Destructor. */
	virtual ~D3Ctrl() {}

	bool initialize();
	void mainLoop();

private:
	bool setParameters(const vrm_protocol::CmdHeader& cmd);
	void streamingState();
	void hdrState();
	void randomDotPatternState();
	void stripePatternState();
	void resetStateMachine();
	void parseCurrentPattern();
	static void sigHandler(int s);


	vrm_protocol::pubsub_server<vrm_protocol::image_group> image_server_;
	vrm_protocol::reqrep_server<vrm_protocol::vrm_cmd> command_server_;

	vrm_protocol::CmdHeader settings_;

	vrm3dvision::CamHandler ch_;
	vrm3dvision::ProjectorCtrl pc_;

	// Camera mode states
	StreamingState streaming_state_;
	RandomDotPatternState random_dot_pattern_state_;
	StripePatternState stripe_pattern_state_;
	HdrState hdr_state_;

	vrm_protocol::image_group ig_;
	bool camera_trigger_;

	// Variables for stripe pattern
	int sequence_id_;
	int image_id_;
	int exposure_id_;
	bool add_ambient_image_;

	int pattern_splash_index_;
	int pattern_num_levels_;

	static bool shutdown_;


};

} // End of namespace vrm3dvision

#endif //__D3_CTRL_HPP__
