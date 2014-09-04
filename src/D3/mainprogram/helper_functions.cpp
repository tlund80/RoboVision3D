/********************************************************************************************************************
 *
 * \file                helper_functions.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-14
 * \version             0.1
 * \brief               Various helper functions
 *
*********************************************************************************************************************/

#include "helper_functions.h"

namespace vrm3dvision {

	//  Sleep for a number of milliseconds
	void sleep_ms (int msecs)
	{
		struct timespec t;
		t.tv_sec = msecs / 1000;
		t.tv_nsec = (msecs % 1000) * 1000000;
		nanosleep (&t, NULL);
	}

}
