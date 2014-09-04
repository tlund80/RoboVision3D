#ifndef MGPEDGE_H_
#define MGPEDGE_H_

#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <boost/lexical_cast.hpp>

#include <string>
using namespace std;

namespace next_best_view_planner
{

class MGPEdge
{
  
private:
	bool inVisualizationGraph;
	bool isUpdatedStatus;
	int numberOfTraverses;
	double _euclideanCost;
	int _weight;
	string edge_id;
	//edge_t _boost_ref;
	
	void printEdge();

public:

	MGPEdge();
	MGPEdge(int weight, double euclidean_cost);

	//void setBoostEdgeRef(edge_t ref);
	string getEdgeId();
	int incrementTraverseCount();
	int getNumberOfTraverses();
	void setEuclideanCost(double);
	double getEuclideanCost();

	bool isInVisualizationGraph();
	bool isUpdated();

	void setIsInVisualizationGraph(bool);
	void setIsUpdated(bool);

	void setEdgeId(string edgeId);
	void setNumberOfTraverses(int numberOfTraverses);

};


} /* namespace next_best_view_planner */
#endif /* MGPEDGE_H_ */
