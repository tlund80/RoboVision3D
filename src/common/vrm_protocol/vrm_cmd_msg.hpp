/********************************************************************************************************************
 *
 * \file                vrm_cmd_msg.hpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-14
 * \version             0.1
 * \brief               Struct and passing functions for sending commands between Host and D3 board sets through ZMQ
 *
*********************************************************************************************************************/

#ifndef __VRM_CMD_MSG_HPP__
#define __VRM_CMD_MSG_HPP__

#include "vrm_protocol.hpp"
#include "MsgHeaders.pb.h"
#include "zhelpers.hpp"

namespace vrm_protocol {


	struct vrm_cmd {
		CmdHeader header;

		vrm_cmd() : header()
		{}

	};
	namespace io {
		/** Send command struct. */
		template<>
		inline bool send(zmq::socket_t &s, const vrm_cmd &v, int flags)
		{
			std::string temp;
			v.header.SerializeToString(&temp);
			s_send(s, temp);
			return true;
		}

		/** Receive command struct. */
		template<>
		inline bool recv(zmq::socket_t &s, vrm_cmd &v, int flags)
		{
			std::string temp = s_recv(s);
			v.header.ParseFromString(temp);
			return true;
		}
	} // End of namespace io
} // End of namespace vrm_protocol

#endif //_VRM_CMD_MSG_HPP__
