#ifndef __LOGGER_INCLUDED_HPP__
#define __LOGGER_INCLUDED_HPP__

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <vector>

#include "vrmusbcam2.h"
#include "vrm_protocol/pubsub.hpp"
#include "vrm_protocol/vrm_cmd_msg.hpp"
#include "vrm_global.h"
#include "vrm_protocol/vrm_log_msg.hpp"

namespace vrm3dvision {

#if LOG_LEVEL >= LOG_ERROR_LEVEL
	#define LOG(level, x) Logger::writeLog(level, \
		static_cast<std::ostringstream&>(       \
				std::ostringstream().flush() << x  \
		).str())
	#define LOG_ERROR(x) LOG(LOG_ERROR_LEVEL, x)
#else
	#define LOG_ERROR(...) do {} while(0)
#endif

#if LOG_LEVEL >= LOG_WARNING_LEVEL
	#define LOG_WARNING(x) LOG(LOG_WARNING_LEVEL, x)
#else
	#define LOG_WARNING(...) do {} while(0)
#endif

#if LOG_LEVEL >= LOG_INFO_LEVEL
	#define LOG_INFO(x) LOG(LOG_INFO_LEVEL, x)
#else
	#define LOG_INFO(...) do {} while(0)
#endif

	class Logger {
	public:
		/** Destructor. */
		virtual ~Logger();

		static void writeLog(int level, const std::string& message);

	private:
		/** Default constructor. */
		Logger();
		static vrm_protocol::pubsub_server<vrm_protocol::vrm_log>& logServer();
		static std::string timeToString();
		static std::ofstream logFile;
	};

}

#endif // end of __LOGGER_INCLUDED_HPP__
