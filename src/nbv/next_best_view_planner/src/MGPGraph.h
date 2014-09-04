/*
 * EMMGraph.h
 *
 *  Created on: Aug 29, 2012
 *      Author: adz
 */

#ifndef EMMGRAPH_H_
#define EMMGRAPH_H_

#include "MGPConf.h"
//#include "EMMCluster.h"
//#include "EMMQueue.h"
//#include "EMMMap.h"
#include <boost/signal.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>

#include <boost/graph/graphviz.hpp>
#include <boost/graph/iteration_macros.hpp>

#include <rw/math/Transform3D.hpp>

namespace next_best_view_planner
{

class MGPGraph
{

	private:

		set<vertex_t> setteledNodes;

		string counterID;
		int getNewID();
		vertex_t currentState;

		string graphID;
		boost::signal<void (bool) > _graphChanged;
		bool _graphChangedCondition;
		bool _firstRun;
		
		int _num_nodes;

	public:

		next_best_view_planner::Graph graph;

		MGPGraph();
		virtual ~MGPGraph();

		void computeJohnson();
		
		void saveGraph(std::string file_name);
		void printEdgeWeights();

		void setCurrentState(vertex_t currentState);
		vertex_t getCurrentState();

		bool isStatePoolEmpty();

		int getNumberOfNodes();

		MGPNode* getNodeAt(int index);

		void incrementEdge(MGPNode* currentState, MGPNode* newState);

		void test(void);

		Graph* getGraphPtr();
		
		string getGraphId();
		void setGraphId(string graphID);

		bool addEdge(vertex_t from, vertex_t to, int weight, double euclidean_cost);
		vertex_t addVertex(rw::math::Transform3D<> pose);
		void removeVertex(vertex_t Vertex);

		boost::signal<void (bool)>& getGraphChangedSignal(void);
		void signalGraphChanged();

		void reset();
		void setCurrentNodeToFirstNode();
		bool compareVertices(vertex_t vertex1, vertex_t vertex2);
		
		int& num_vertices(Graph graph);



};

} /* namespace dti_co_worker */
#endif /* EMMGRAPH_H_ */
