#ifndef __VRM_PROTOCOL_INPROC_HPP__
#define __VRM_PROTOCOL_INPROC_HPP__

#include "vrm_protocol.hpp"

namespace vrm_protocol {

	template<typename T>
	class inproc_server : public server<T> {
	public:

	  /** Default constructor. */
		inproc_server(zmq::context_t* c)
		: server<T>(context_ptr(c))
	  {}

	  /** Destructor. */
	  virtual ~inproc_server()
	  {}

	  virtual void startup(const std::string &addr = "tcp://127.0.0.1:6000")
	  {
		if (!network_entity::_s) {
			network_entity::_s = socket_ptr(new zmq::socket_t(*network_entity::_ctx_ptr, ZMQ_PAIR));
		}
		network_entity::_s->bind(addr.c_str());
	  }

	  /** Publish data to clients.
		*
		* \param[in] t data to be published.
		* \param[in] timeout_ms unused. Method returns always immediately.
		* \returns true if data was published successfully.
		* \returns false never.
		**/
	  virtual bool publish(const T &t, int timeout_ms = 0)
	  {
		if (network_entity::_s) {
			io::send(*network_entity::_s, t, 0);
			return true;
		}
		else
			return false;
	  }
	};

	   /** Fast client implementation. */
	  template<typename T>
	  class inproc_client : public client<T> {
	  public:

	    /** Default constructor */
	    inproc_client(zmq::context_t* c)
	      : client<T>(context_ptr(c))
	    {}

	    virtual ~inproc_client()
	    {}

	    /** Start a new connection to the given endpoint. If called multiple times, the client
	      * will connect to more endpoints.
	      *
	      * \param [in] addr endpoint address to connect to
	      * \throws ib_error on error
	      */
	    virtual void startup(const std::string &addr = "tcp://127.0.0.1:6000")
	    {
			if (!network_entity::_s) {
				network_entity::_s = socket_ptr(new zmq::socket_t(*network_entity::_ctx_ptr, ZMQ_PAIR));
			}
			try {
				network_entity::_s->connect(addr.c_str());
			} catch (const zmq::error_t& e) {
				std::cout << "ZMQ error when trying to connect to server, what: " << e.what() << std::endl;
			}
	    }

	    /** Receive data.
	      *
	      * \warning You should not rely on receiving a single specific data element
	      *          with the fast_client and fast_server implementation. Expect data to get lost.
	      *
	      * \param [in,out] t data to be received
	      * \param [in] timeout_ms Maximum wait time in milliseconds to receive data.
	      *             Timeout is set to 1 second by default.
	      * \returns true if data was received successfully.
	      * \returns false when receive timeout occurred.
	      * \throws ib_error on error
	      */
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

	  private:

	    /** Receive complete message once */
	    bool receive_message(T &t, int flags)
	    {
	      bool r = io::recv(*network_entity::_s, t, flags);

	      return r;
	    }

	  };


}

#endif //__VRM_PROTOCOL_INPROC_HPP__
