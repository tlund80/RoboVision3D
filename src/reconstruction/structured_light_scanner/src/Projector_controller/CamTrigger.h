/********************************************************************************************************************
 *
 * \file                CamTrigger.h
 * \author              Kent Hansen (kenthansen1@gmail.com)
 * \date                2013-04-15
 * \version             1.0
 * \brief               Class for camera triggering using Arduino board and sync signal
 *
*********************************************************************************************************************/

#ifndef CAMTRIGGER_H_
#define CAMTRIGGER_H_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>


namespace perception {

class CamTrigger
{
	public:
		/// Constructor
		CamTrigger();

        /*!
		* Trigger cameras on next sync signal
		*
		* \return True for success
		*/
		bool triggerOneshot();

        /*!
		* Trigger cameras continuously on sync signal. Triggering stops on new command.
		*
		* \return True for success
		*/
		bool triggerContinuous();

        /*!
		* Check if camera triggering is done
		*
		* \return Number of bytes received. Zero indicates no data received. Negative numbers indicate errors
		*/
		int triggerDone();

	private:
		int sockfd;
		struct sockaddr_in servaddr;
};

} /* namespace perception */

#endif /* CAMTRIGGER_H_ */
