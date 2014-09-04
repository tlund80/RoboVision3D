/*
 * IActionServer.h
 *
 *  Created on: Aug 7, 2013
 *      Author: thomas
 */

#ifndef IACTIONSERVER_H_
#define IACTIONSERVER_H_

#include <iostream>

using namespace std;

namespace structured_light_scanner {

// Base class
class IActionServer
{
public:

   // pure virtual function providing interface framework.
	virtual void startScan(bool _scan) = 0;

private:



};
}



#endif /* IACTIONSERVER_H_ */
