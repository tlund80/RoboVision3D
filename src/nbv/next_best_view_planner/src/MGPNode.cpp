#include "MGPNode.h"

using namespace std;

namespace next_best_view_planner
{

MGPNode::MGPNode()
{
    init();
    this->_nodeID = generateID();
    this->printNode();
}

MGPNode::MGPNode(rw::math::Transform3D<>  pose) : _pose(pose)
{
	this->_nodeID = generateID();
	init();
	this->printNode();
}

MGPNode::MGPNode(rw::math::Transform3D<> pose, string nodeID, int numberOfTimesInState) :
		 _pose(pose),
		 _nodeID(nodeID),
		 _numberOfTimesInState(numberOfTimesInState)
{
	init();
	this->printNode();
}

void MGPNode::init()
{
	this->_numberOfTimesInState = 0;
	this->incrementVisitCount();
	this->probabilityToNode = 0.0;
	this->setIsInVisualizationGraph(false);
	this->setIsUpdated(true);
}

string MGPNode::getNodeID() const
{ 
	return this->_nodeID;
}

void MGPNode::setNodeID(string nodeID)
{
	this->_nodeID = nodeID;
}

int MGPNode::incrementVisitCount()
{
	this->setIsUpdated(true);
	return ++_numberOfTimesInState;
}

int MGPNode::getVisitCount()
{
	return _numberOfTimesInState;
}

void MGPNode::setVisitCount(int numberOfTimesInState)
{
	this->_numberOfTimesInState = numberOfTimesInState;
}

rw::math::Transform3D<> MGPNode::getPose()
{
	return this->_pose;
}

bool MGPNode::isInVisualizationGraph()
{
	return this->inVisualizationGraph;
}

bool MGPNode::isUpdated()
{
	return this->isUpdatedStatus;
}


void MGPNode::setIsInVisualizationGraph(bool input)
{
	this->inVisualizationGraph = input;
}

void MGPNode::setIsUpdated(bool input)
{
	this->isUpdatedStatus = input;
}

string MGPNode::generateID()
{
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    return boost::lexical_cast<std::string>(uuid);
}

void MGPNode::printNode()
{
	cout << "----------------------------" << endl;
	cout << "Node id = " << this->_nodeID << " @ " << this << endl;
	cout << "NumTimesInState   = " << this->_numberOfTimesInState << endl;
	cout << "Pose: " << _pose << endl;
	cout << "----------------------------" << endl;

}
} /* namespace dti_co_worker */
