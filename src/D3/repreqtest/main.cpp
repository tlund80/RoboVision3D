/********************************************************************************************************************
 *
 * \file                main.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-14
 * \version             0.1
 * \brief               Main program for the D3 board
 *
*********************************************************************************************************************/

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include <vrm_protocol/reqrep.hpp>
#include <vrm_protocol/vrm_cmd_msg.hpp>

using namespace std;

int main (int argc, char** argv) {

	// parse command line
	bool server = false;
	if (argc>1)
	{
		server = true;
	}

	if (server)
	{

		std::string rr_server_address = "tcp://127.0.0.1:6010";
	
		std::cout << rr_server_address << std::endl;

		vrm_protocol::reqrep_server<vrm_protocol::vrm_cmd> rr_server;
		rr_server.startup(rr_server_address);

		while (!rr_server.waitForClient(1000))
		{
			std::cout << "Waiting for client!" << std::endl;
		}
		std::cout << "Client connected!" << std::endl;

		// Super loop
		while (1)
		{
			vrm_protocol::vrm_cmd msg;

			if (rr_server.receive(msg, 1000))
			{
				std::cout << "received: msg: " << msg.timestamp << std::endl;
				sleep(2);
				rr_server.publish(msg,100);
				std::cout << "Send: " << msg.timestamp << std::endl;
			}
			std::cout << "No new messages" << std::endl;
		}
	}
	else
	{
		std::string rr_client_address = "tcp://127.0.0.1:6010";
	
		std::cout << rr_client_address << std::endl;

		vrm_protocol::reqrep_client<vrm_protocol::vrm_cmd> rr_client;
		rr_client.startup(rr_client_address);

		while(!rr_client.connect())
		{
			std::cout << "Trying to connect to server" << std::endl;
		}

		std::cout << "Connected to server" << std::endl;

		int msgs = 0;
		// Super loop
		while (msgs < 5)
		{
			msgs++;
			sleep(1);
			vrm_protocol::vrm_cmd msg;

			msg.timestamp = 22.3 * (double)msgs + 11.97;

			rr_client.publish(msg);
			
			std::cout << "Send: " << msg.timestamp << std::endl;
			vrm_protocol::vrm_cmd test;
			rr_client.receive(test,10000);
			std::cout << "Received: " <<  test.timestamp << std::endl;

		}
		rr_client.shutdown();

		std::cout << "Exit" << std::endl;
	}

	return 0;
}
