/********************************************************************************************************************
 *
 * \file                image_group_msg.hpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-14
 * \version             0.1
 * \brief               Struct and passing functions for sending image sets through ZMQ
 *
*********************************************************************************************************************/

#ifndef __VRM_IMAGE_GROUP_MSG_HPP__
#define __VRM_IMAGE_GROUP_MSG_HPP__

#include "vrm_protocol.hpp"
#include "MsgHeaders.pb.h"
#include "zhelpers.hpp"

namespace vrm_protocol {

	struct image_group {
		ImgGroupHeader header;
		cv::Mat left_image;
		cv::Mat right_image;
		cv::Mat color_image;
	};

	namespace io {
		/** Send image_group struct. */
		template<>
		inline bool send(zmq::socket_t &s, const image_group &v, int flags)
		{
			std::string temp;
			v.header.SerializeToString(&temp);
			if(v.header.has_left_img() || v.header.has_right_img() || v.header.has_color_img())
				s_sendmore(s, temp);
			else
				s_send(s, temp);

			if (v.header.has_left_img())
			{
				if (v.header.has_right_img() || v.header.has_color_img())
					io::send(s,v.left_image,flags | ZMQ_SNDMORE);
				else
					io::send(s,v.left_image,flags);
			}
			if (v.header.has_right_img())
			{
				if (v.header.has_color_img())
					io::send(s,v.right_image,flags | ZMQ_SNDMORE);
				else
					io::send(s,v.right_image,flags);
			}
			if (v.header.has_color_img())
			{
				io::send(s,v.color_image,flags);
			}

			return true;
		}


		/** Receive an image_group struct. */
		template<>
		inline bool recv(zmq::socket_t &s, image_group &v, int flags)
		{
			std::string temp = s_recv(s);
			v.header.ParseFromString(temp);

			if (v.header.has_left_img())
				io::recv(s,v.left_image,flags);
			if (v.header.has_right_img())
				io::recv(s,v.right_image,flags);
			if (v.header.has_color_img())
				io::recv(s,v.color_image,flags);

			return true;
		}
	} // End of namespace io
} // End of namespace vrm_protocol

#endif //__VRM_IMAGE_GROUP_MSG_HPP__
