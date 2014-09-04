#ifndef __COMMAND_HANDLER_INCLUDED_HPP__
#define __COMMAND_HANDLER_INCLUDED_HPP__

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <vector>

#include "vrmusbcam2.h"
#include "vrm_protocol/pubsub.hpp"
#include "vrm_protocol/vrm_cmd_msg.hpp"
#include "cam_handler.hpp"
#include "vrm_global.h"

namespace vrm3dvision {

	class CommandHandler {
	public:
		/** Default constructor. */
		CommandHandler();

		/** Destructor. */
		virtual ~CommandHandler() {}

		void readMessage(const vrm_protocol::vrm_cmd& cmd_msg, CamHandler& ch);

	private:


	};

}

#endif // end of __COMMAND_HANDLER_INCLUDED_HPP__
