/********************************************************************************************************************
 *
 * \file                main.cpp
 * \author              Jeppe Pedersen (jepp@teknologisk.dk)
 * \date                2013-10-14
 * \version             0.1
 * \brief               Main program for the D3 board
 *
*********************************************************************************************************************/

//#include <iostream>
//#include <cstdlib>
//#include <cstring>
//#include <sstream>
//#include <iomanip>
//#include <signal.h>
//
//#include <vrmusbcam2.h>
//#include <opencv2/opencv.hpp>
//
//#include <vrm_protocol/pubsub.hpp>
//#include <vrm_protocol/vrm_cmd_msg.hpp>
//#include <vrm_protocol/image_group_msg.hpp>
//#include "cam_handler.hpp"
//#include "helper_functions.h"
//#include "logger.hpp"
//#include "ProjectorCtrl/ProjectorCtrl.hpp"
//#include "timer.hpp"

#include "d3_ctrl.hpp"

using namespace std;
using namespace vrm3dvision;


int main (int argc, char** argv)
{
	D3Ctrl ctrl;
	ctrl.mainLoop();

    return 0;
}
