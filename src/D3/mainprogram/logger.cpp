#include "logger.hpp"

namespace vrm3dvision {

// Static member declaration
std::ofstream Logger::logFile;



Logger::Logger()
{
#ifdef LOG_TO_HOST
	std::string address = "tcp://*:" + std::string(VRM_LOG_SERVER_PORT);
	logServer().startup(address);
	usleep(150000);	//TODO Move delay for construction to board initialization instead of first log message
#endif
#ifdef LOG_TO_FILE
	std::stringstream ss;
	ss << "/root/log/log_" << timeToString() << ".txt";
	logFile.open(ss.str().c_str(), std::ofstream::out);
#endif
}



Logger::~Logger()
{
#ifdef LOG_TO_HOST
	logServer().shutdown();
#endif
#ifdef LOG_TO_FILE
	logFile.close();
#endif
}



vrm_protocol::pubsub_server<vrm_protocol::vrm_log>& Logger::logServer()
{
	static vrm_protocol::pubsub_server<vrm_protocol::vrm_log>* ret = new vrm_protocol::pubsub_server<vrm_protocol::vrm_log>();
	return *ret;
}



std::string Logger::timeToString()
{
	time_t rawtime = time(0);
	struct tm * timeinfo = localtime ( &rawtime );;
	char buffer[20];
	strftime (buffer,sizeof(buffer),"%Y-%m-%d %H:%M:%S",timeinfo);
	std::string logTime(buffer);
	return logTime;
}



void Logger::writeLog(int level, const std::string& message)
{
	static Logger log;

	// Convert level to string
	std::string level_str;
	if (level == LOG_ERROR_LEVEL)
		level_str = "[ERROR]  ";
	else if (level == LOG_WARNING_LEVEL)
		level_str = "[WARNING]";
	else if (level == LOG_INFO_LEVEL)
		level_str = "[INFO]   ";
	else
		level_str = "[UNKNOWN]";

	// Final log message
	std::string log_msg_str = "" + level_str + " " + timeToString() + ": " + message;

#ifdef LOG_TO_CONSOLE
	if (level == LOG_ERROR_LEVEL)
		std::cout << "\033[31m";
	else if (level == LOG_WARNING_LEVEL)
		std::cout << "\033[33m";

	std::cout << log_msg_str << "\033[0m" << std::endl;
#endif
#ifdef LOG_TO_HOST
	vrm_protocol::vrm_log log_msg;
	log_msg.message = message;
	log_msg.level = level;
	log_msg.timestamp = 0; // TODO find good solution for timestamps

	logServer().publish(log_msg,0);
#endif
#ifdef LOG_TO_FILE
	logFile << log_msg_str << std::endl;
#endif
}

} // end of namespace vrm3dvision
