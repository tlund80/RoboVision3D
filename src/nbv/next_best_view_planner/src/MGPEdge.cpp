#include "MGPEdge.h"

namespace next_best_view_planner
{

MGPEdge::MGPEdge(int weight, double euclidean_cost) : _weight(weight), _euclideanCost(euclidean_cost)
{
	this->numberOfTraverses = 0;
	this->setIsInVisualizationGraph(false);
	this->setIsUpdated(true);

	boost::uuids::uuid uuid = boost::uuids::random_generator()();
	edge_id = boost::lexical_cast<std::string>(uuid);
	
	this->printEdge();
}

int MGPEdge::incrementTraverseCount()
{
	this->setIsUpdated(true);
	return ++numberOfTraverses;
}

/*void MGPEdge::setBoostEdgeRef(edge_t ref)
{
  this->_boost_ref = ref;
}
*/
void MGPEdge::setEuclideanCost(double cost)
{
	this->_euclideanCost = cost;
	this->setIsUpdated(true);
}

void MGPEdge::setNumberOfTraverses(int numberOfTraverses)
{
	this->setIsUpdated(true);
	this->numberOfTraverses = numberOfTraverses;
}

int MGPEdge::getNumberOfTraverses()
{
	return this->numberOfTraverses;
}

double MGPEdge::getEuclideanCost()
{
	return this->_euclideanCost;
}

bool MGPEdge::isInVisualizationGraph()
{
	return this->inVisualizationGraph;
}

bool MGPEdge::isUpdated()
{
	return this->isUpdatedStatus;
}

void MGPEdge::setIsInVisualizationGraph(bool input)
{
	this->inVisualizationGraph = input;
}

void MGPEdge::setIsUpdated(bool input)
{
	this->isUpdatedStatus = input;
}

string MGPEdge::getEdgeId()
{
	return this->edge_id;
}

void MGPEdge::setEdgeId(string edgeId)
{
	this->edge_id = edgeId;
}

void MGPEdge::printEdge()
{
	cout << "----------------------------" << endl;
	cout << "Edge id = " << this->edge_id << " @ " << this << endl;
	cout << "numberOfTraverses   = " << this->numberOfTraverses << endl;
	cout << "Euclidean cost: " << _euclideanCost << endl;
	cout << "Weight: " << _weight << endl;
	cout << "----------------------------" << endl;

}

} /* namespace next_best_view_planner */
