#ifndef __VRM_PROTOCOL_REQREP_HPP__
#define __VRM_PROTOCOL_REQREP_HPP__

#include "vrm_protocol.hpp"

#define SHUTDOWN_MESSAGE "shutdown"
#define DATA_MESSAGE "data"
#define CONNECT_MESSAGE "connect"
#define CONNECTION_ESTABLISHED "connect_ok"

namespace vrm_protocol {

	template<typename T>
	class reqrep_server : public server<T> {
	public:

	  /** Default constructor. */
		reqrep_server()
		: server<T>(context_ptr(new zmq::context_t(1))),
		  client_address_(""),
		  connected_(false)
	  {}

	  /** Destructor. */
	  virtual ~reqrep_server()
	  {}

	  virtual void startup(const std::string &addr = "tcp://127.0.0.1:6000")
	  {
		if (!network_entity::_s) {
			network_entity::_s = socket_ptr(new zmq::socket_t(*network_entity::_ctx_ptr, ZMQ_ROUTER));
		}
		network_entity::_s->bind(addr.c_str());
	  }

	  bool isClientConnected()
	  {
		  return connected_;
	  }

	  bool waitForClient(int timeout_ms = -1)
	  {
		  T msg;

		  if (!connected_)
			  receive(msg,timeout_ms);

		  return connected_;
	  }

	  virtual bool publish(const T &t, int timeout_ms = 0)
	  {

			if (network_entity::_s) {
				if (connected_)
				{
					io::send(*network_entity::_s, client_address_, 0 | ZMQ_SNDMORE);
					io::send(*network_entity::_s, t, 0);
					return true;
				}
				else
				{
					return false;
				}
			}
			else
			{
				return false;
			}
	  }

	    bool isConnected()
	    {
	    	return connected_;
	    }

	    void disconnect()
	    {
	    	if (isConnected())
	    	{
	    		connected_ = false;
				if (network_entity::_s) {
					io::send(*network_entity::_s, client_address_, 0 | ZMQ_SNDMORE);
					io::send(*network_entity::_s, SHUTDOWN_MESSAGE, 0);
				}
	    	}
	    }

		virtual void shutdown()
		{
			disconnect();
			network_entity::shutdown();
		}

		bool receive(T &t, int timeout_ms = -1)
		{
			if (network_entity::_s)
			{
				if (timeout_ms != -1)
				{
					if (!io::is_data_pending(*network_entity::_s, timeout_ms))
					{
					  return false;
					}
					else
					{
					  if (receive_message(t, ZMQ_DONTWAIT))
						  return true;
					  else
						  return false;
					}
				}
				else
				{
					if (receive_message(t, ZMQ_DONTWAIT))
					  return true;
					else
					  return false;
				}
			}
			else
			{
			  return false;
			}
		}

	private:

		std::string client_address_;
		bool connected_;

		virtual bool publish(const std::string &t, int timeout_ms = 0)
		{
			if (network_entity::_s) {
				if (connected_)
				{
					io::send(*network_entity::_s, client_address_, 0 | ZMQ_SNDMORE);
					io::send(*network_entity::_s, DATA_MESSAGE, 0 | ZMQ_SNDMORE);
					io::send(*network_entity::_s, t, 0);
					return true;
				}
				else
				{
					return false;
				}
			}
			else
			{
				return false;
			}
		}

		/** Receive complete message once */
		bool receive_message(T &t, int flags)
		{
			// Receive address and message type
			std::string address;
			bool r = io::recv(*network_entity::_s, address, flags);
			std::string type;
			r = io::recv(*network_entity::_s, type, flags);

			if (client_address_ != address)
			{
				if(!connected_ && type == CONNECT_MESSAGE)
				{
					//std::cout << "Client connected! address: " << address << std::endl;
					client_address_ = address;
					connected_ = true;
					if(!publish(CONNECTION_ESTABLISHED, 100))
					{
						std::cout << "Unable to send CONNECTION_ESTABLISHED" << std::endl;
					}
				}
				else
				{
					// Recieve message to delete it from queue
					if (type == DATA_MESSAGE)
					{
						T tmp;
						io::recv(*network_entity::_s, tmp, flags);
					}
				}
				r = false;
			}
			else if (client_address_ == address)
			{
				if (type == SHUTDOWN_MESSAGE)
				{
					connected_ = false;
					client_address_ = "";
					r = false;
				}
				else if (type == DATA_MESSAGE)
				{
					bool r = io::recv(*network_entity::_s, t, flags);
				}
				else if (type == CONNECT_MESSAGE)
				{
					r = false;
				}
			}
			return r;
		}

	};

	  template<typename T>
	  class reqrep_client : public client<T> {
	  public:

	    /** Default constructor */
		  reqrep_client()
	      : client<T>(context_ptr(new zmq::context_t(1))),
	        connected_(false),
			connect_msg_sent_(false)
	    {}

	    virtual ~reqrep_client()
	    {}

	    virtual void startup(const std::string &addr = "tcp://127.0.0.1:6000")
	    {
			if (!network_entity::_s) {
				network_entity::_s = socket_ptr(new zmq::socket_t(*network_entity::_ctx_ptr, ZMQ_DEALER));
			}
			try {
				network_entity::_s->connect(addr.c_str());
				//network_entity::_s->setsockopt(ZMQ_SUBSCRIBE, 0, 0);
			} catch (const zmq::error_t& e) {
				std::cout << "ZMQ error when trying to connect to server, what: " << e.what() << std::endl;
			}
	    }

	    bool isConnected()
	    {
	    	return connected_;
	    }

	    void disconnect()
	    {
	    	if (isConnected())
	    	{
	    		connected_ = false;
				if (network_entity::_s) {
					io::send(*network_entity::_s, SHUTDOWN_MESSAGE, 0);
				}
	    	}
	    }

	    bool connect(int timeout_ms = 10)
	    {
	    	if(!network_entity::_s)
	    	{
	    		std::cout << "Connection not initialized - Call function \"startup\"" << std::endl;
	    		return false;
	    	}
	    	// Send connect message
	    	if (!connect_msg_sent_)
	    	{
				io::send(*network_entity::_s, CONNECT_MESSAGE, 0);
				connect_msg_sent_ = true;
	    	}

	    	// Recieve answer from server
	    	std::string reply;

			if (!io::is_data_pending(*network_entity::_s, timeout_ms))
			{
				return false;
			}
			else
			{
				if (!io::recv(*network_entity::_s, reply, ZMQ_DONTWAIT))
					return false;
			}

	    	if (reply != CONNECTION_ESTABLISHED)
	    	{
	    		return false;
	    	}
	    	else
	    	{
	    		connected_ = true;
	    		return true;
	    	}
	    }

	    virtual void shutdown()
	    {
	    	disconnect();
			network_entity::shutdown();
	    }

	    virtual bool receive(T &t, int timeout_ms = -1)
	    {
	      if (network_entity::_s)
	      {
	    	  if (timeout_ms != -1)
	    	  {
		    	  if (!io::is_data_pending(*network_entity::_s, timeout_ms))
		          {
		        	  return false;
		          }
		          else
		          {
			    	  if (receive_message(t, ZMQ_DONTWAIT))
			    	  	  return true;
					  else
						  return false;
		          }
	    	  }
	    	  else
	    	  {
		    	  if (receive_message(t, ZMQ_DONTWAIT))
		    	  	  return true;
				  else
					  return false;
	    	  }
	      }
	      else
	      {
	    	  return false;
	      }
	    }

		  bool publish(const T &t, int timeout_ms = 0)
		  {
			if (network_entity::_s) {
				io::send(*network_entity::_s, DATA_MESSAGE, 0 | ZMQ_SNDMORE);
				io::send(*network_entity::_s, t, 0);
				return true;
			}
			else
				return false;
		  }

	  private:

		  bool connected_;
		  bool connect_msg_sent_;

	    /** Receive complete message once */
	    bool receive_message(T &t, int flags)
	    {
	    	std::string type;
	    	bool r = io::recv(*network_entity::_s, type, flags);
	    	if (type == SHUTDOWN_MESSAGE)
			{
				connected_ = false;
				connect_msg_sent_ = false;
				r = false;
			}
			else if (type == DATA_MESSAGE)
			{
				r = io::recv(*network_entity::_s, t, flags);
			}

	      return r;
	    }

	  };


}

#endif //__VRM_PROTOCOL_REQREP_HPP__
