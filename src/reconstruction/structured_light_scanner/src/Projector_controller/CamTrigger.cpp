/********************************************************************************************************************
 *
 * \file                CamTrigger.cpp
 * \author              Kent Hansen (kenthansen1@gmail.com)
 * \date                2013-04-15
 * \version             1.0
 * \brief               Class for camera triggering using Arduino board and sync signal
 *
*********************************************************************************************************************/

#include "CamTrigger.h"

namespace perception {

CamTrigger::CamTrigger()
{
	sockfd=socket(AF_INET,SOCK_DGRAM,0);

	bzero(&servaddr,sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr=inet_addr("169.254.94.90");
	servaddr.sin_port=htons(5000);
}

bool CamTrigger::triggerOneshot()
{
	int ret = sendto(sockfd,"a",1,0,(struct sockaddr *)&servaddr,sizeof(servaddr));
	return (ret > 0) ? true : false;
}

bool CamTrigger::triggerContinuous()
{
	int ret = sendto(sockfd,"b",1,0,(struct sockaddr *)&servaddr,sizeof(servaddr));
	return (ret > 0) ? true : false;
}

int CamTrigger::triggerDone()
{
	char buffer[1000];
	int ret = recvfrom(sockfd,buffer,1000,0,NULL,NULL);
	if( ret > 0)
	{
		std::string value(buffer);
		std::string temp = value.substr(0,11);
		std::cout << temp << std::endl;

		if( temp.compare("TriggerDone") == 0 )
		{
			std::cout << "Complete" << std::endl;
			return 1;
		}
	}
	return 0;
}

} /* namespace perception */
