#ifndef MGPNODE_H_
#define MGPNODE_H_

#include <vector>
#include <string>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <boost/lexical_cast.hpp>

#include <rw/math/Transform3D.hpp>

using namespace std;

namespace next_best_view_planner
{
class MGPNode
{

	private:
		bool inVisualizationGraph;
		bool isRare;
		bool isUpdatedStatus;
		bool _isAnomaly;
		rw::math::Transform3D<>  _pose;
		int _numberOfTimesInState;
		double probabilityToNode;
		string _nodeID;

	public:


		/**
		 * Constructor created the node with the pose. The nodeID is automatically created as a combined hash from the data. The constructor also calls incrementVisitCount().
	 	 * @param [vector<double>] data
		 * @returns void
		 */
		MGPNode();
		MGPNode(rw::math::Transform3D<>  pose);
		MGPNode(rw::math::Transform3D<>  pose, string nodeID, int numberOfTimesInState);
		void init();

		/**
		 * Function that increments the number of times this node has been visited
	 	 * @param void
		 * @returns [int] - number of visits
		 */
		int incrementVisitCount();

		/**
		 * Function that returns the number of times this state has been visited
	 	 * @param void
		 * @returns [int] - number of visits
		 */
		int getVisitCount();
		
		/**
		 * Function that sets the number of times this state has been visited
	 	 * @param [int] - number of visits
		 * @returns void
		 */
		void setVisitCount(int numberOfTimesInState);

		string getNodeID() const;
		void setNodeID(string nodeID);

		void printNode();

		rw::math::Transform3D<>  getPose();

		bool isInVisualizationGraph();
		bool isUpdated();

		void setIsInVisualizationGraph(bool);
		void setIsUpdated(bool);

		string generateID();

		int generateHash();

};
} /* namespace next_best_view_planner */
#endif /* MGPNODE_H_ */
