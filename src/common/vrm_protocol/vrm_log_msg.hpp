/********************************************************************************************************************
 *
 * \file                vrm_log_msg.hpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-14
 * \version             0.1
 * \brief               Struct and passing functions for sending log messages from D3 board to host through ZMQ
 *
*********************************************************************************************************************/

#ifndef __VRM_MSG_MSG_HPP__
#define __VRM_MSG_MSG_HPP__

#include "vrm_protocol.hpp"

namespace vrm_protocol {

	struct vrm_log {
		int message_id;
		double timestamp;
		int level;
		int type;
		std::string message;

		vrm_log() :		message_id(-1),
						timestamp(-1),
						level(-1),
						type(-1),
						message("")
		{}

	};
	namespace io {
		/** Send vrm_log struct. */
		template<>
		inline bool send(zmq::socket_t &s, const vrm_log &v, int flags)
		{
			std::ostringstream ostr;
			ostr 	<< v.message_id << " "
					<< v.timestamp << " "
					<< v.level << " "
					<< v.type << " "
					<< v.message;

			if (ostr.good())
			{
				const std::string &str = ostr.str();

				zmq::message_t msgHeader(str.size());
				memcpy(msgHeader.data(), str.c_str(), str.size());
				s.send(msgHeader,flags);

				return true;
			}
			else
			{
				return false;
			}

		}

		/** Receive an vrm_log struct. */
		template<>
		inline bool recv(zmq::socket_t &s, vrm_log &v, int flags)
		{
			zmq::message_t msg;

			s.recv(&msg, flags); // TODO Add error check

			std::string str;
			str.assign(
					static_cast<char*>(msg.data()),
					static_cast<char*>(msg.data()) + msg.size());
			std::istringstream is(str.c_str());

			is  >> v.message_id
				>> v.timestamp
				>> v.level
				>> v.type;
			std::getline(is,v.message);

			return true;

		}
	} // End of namespace io
} // End of namespace vrm_protocol

#endif //__VRM_MSG_MSG_HPP__
