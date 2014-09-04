#ifndef __VRM_PROTOCOL_HPP__
#define __VRM_PROTOCOL_HPP__

#include "zmq.hpp"
#include <zmq.h>
#include <zmq_utils.h>
#include <memory>
#include <streambuf>
#include <sstream>
#include <string>
#include <limits>
#include <vector>
#include <opencv2/opencv.hpp>

/** A lightweight C++ library to control and exchange data between VRM-D3 and PC **/
namespace vrm_protocol {

	enum class camera_mode {
		UNKNOWN = -1,
		STREAMING = 10,
		PATTERN = 11,
		HDR = 12,
		RANDOM_DOT_PATTERN = 13
	};

	/** Reference counted pointer to a ZMQ socket. */
	typedef std::shared_ptr<zmq::socket_t> socket_ptr;
	/** Reference counted pointer to a ZMQ context. */
	typedef std::shared_ptr<zmq::context_t> context_ptr;

	class network_entity {
		public:

		/** Construct from context. */
		network_entity(const context_ptr &c)
		  :_ctx_ptr(c)
		{}

		/** Destructor. */
		virtual ~network_entity()
		{
		  shutdown();
		}

		/** Get context */
		inline context_ptr get_context() const {
		  return _ctx_ptr;
		}

		/** Get socket */
		inline socket_ptr get_socket() const {
		  return _s;
		}

		/** Startup a new service or connection to service. Calling this method multiple
		  * times shall result in creating additional services or connections. */
		virtual void startup(const std::string &addr) = 0;

		/** Shutdown all services or connections*/
		virtual void shutdown()
		{
		  if (_s && _s->connected()) {
			_s->close(); // does not throw.
			_s.reset();
		  }
		}

		protected:
		context_ptr _ctx_ptr; ///< ZMQ context
		socket_ptr _s;    ///< ZMQ socket
	};

	template<typename T>
	class server : public network_entity {
	public:
		server(const context_ptr& c) : network_entity(c)
		{}

		virtual ~server() {};

		virtual bool publish(const T &t, int timeout_ms) = 0;

	};

	template<typename T>
	class client : public network_entity {
	public:
		client(const context_ptr& c) : network_entity(c)
		{}

		virtual ~client() {};

		virtual bool receive(T &t, int timeout_ms) = 0;

	};

	//helper functions
	namespace io {

		/** Test if data to be read is pending on the socket. Returns true
		  * when at least one byte readable within the given timeout in milli
		  * seconds. */
		inline bool is_data_pending(zmq::socket_t &s, int timeout_ms)
		{
			zmq::pollitem_t items[] = {{ s, 0, ZMQ_POLLIN, 0 }};
			try {
				zmq::poll(&items[0], 1, timeout_ms);
			} catch (const zmq::error_t& e) {
				std::cout << "ZMQ error when looking for pending data, what: " << e.what() << std::endl;
			}
			return (items[0].revents & ZMQ_POLLIN);
		}

		/** Generic receive method. Tries to receive a value of T from the given socket.
		  * T must have locatable extraction semantics. This method will block until
		  * at least one byte is readable from the socket or an error occurs.
		  *
		  * \param[in] s socket to receive from
		  * \param[in,out] v value to receive
		  * \param[in] flags ZMQ flags
		  * \throws ib_error on error.
		  */
		template<class T>
		inline bool recv(zmq::socket_t &s, T &v, int flags)
		{
		  zmq::message_t msg;

		  s.recv(&msg, flags); // TODO Add error check
		  std::istream is(static_cast<char*>(msg.data()), msg.size());

		  is >> v;

		  if (!is.fail())
		  {
			  return true;
		  }
		  else
		  {
			  return false; // TODO Send error message
		  }
		}

		/** Receive a string. */
		template<>
		inline bool recv(zmq::socket_t &s, std::string &v, int flags)
		{
		  zmq::message_t msg;
		  bool r = s.recv(&msg, flags); // TODO Add error check

		  v.assign(
			static_cast<char*>(msg.data()),
			static_cast<char*>(msg.data()) + msg.size());

		  return r;
		}

		/** Receive a OpenCV Mat. */
		template<>
		inline bool recv(zmq::socket_t &s, cv::Mat &v, int flags)
		{
			zmq::message_t msg;

			s.recv(&msg, flags); // TODO Add error check

			std::string str;
			str.assign(
			        static_cast<char*>(msg.data()),
			        static_cast<char*>(msg.data()) + msg.size());
			std::istringstream is(str.c_str());
			int cols, rows, channels, type;
			is  >> cols
				>> rows
				>> channels
				>> type;

			v = cv::Mat(rows,cols,type);

			s.recv(v.data,(v.rows * v.cols * v.elemSize()),0);

			return true;

		}

		/** Generic send method. T must have insertion operator semantics.
		  *
		  * \param[in] s socket to send data to
		  * \param[in] v data to send
		  * \param[in] flags ZMQ send flags.
		  * \throws ib_error on error.
		  */
		template<class T>
		inline bool send(zmq::socket_t &s, const T &v, int flags)
		{
		  std::ostringstream ostr;
		  ostr << v;

		  if (ostr.good())
		  {
			  const std::string &str = ostr.str();
			  zmq::message_t msg(str.size());
			  if (!str.empty()) {
				memcpy(msg.data(), str.c_str(), str.size());
			  }

			  s.send(msg, flags); // TODO Add error check
			  return true;
		  }
		  else
		  {
			  return false;
		  }


		}

		/** Send OpenCV Mat. */
		template<>
		inline bool send(zmq::socket_t &s, const cv::Mat &v, int flags)
		{
			std::ostringstream ostr;
			ostr << v.cols << " "
			   << v.rows << " "
			   << v.channels() << " "
			   << v.type();

			if (ostr.good())
			{
				const std::string &str = ostr.str();

				zmq::message_t msgHeader(str.size());
				memcpy(msgHeader.data(), str.c_str(), str.size());
				s.send(msgHeader,flags | ZMQ_SNDMORE);

				zmq::message_t msgData(v.rows * v.cols * v.elemSize());
				memcpy(msgData.data(),v.data,(v.rows * v.cols * v.elemSize()));

				s.send(msgData,flags);
				return true;
			}
			else
			{
				return false;
			}

		}

		/** Send string. */
		template<>
		inline bool send(zmq::socket_t &s, const std::string &v, int flags)
		{
		  zmq::message_t msg(v.size());
		  if (!v.empty()) {
			memcpy(msg.data(), v.c_str(), v.size());
		  }

		  s.send(msg, flags); // TODO add error check
		  return true;
		}
	} // End of namespace io


} // End of namespace vrm_protocol

#endif // __VRM_PROTOCOL_HPP__
