/*
 * EMMGraph.cpp
 *
 *  Created on: Aug 29, 2012
 *      Author: adz
 */

#include "MGPGraph.h"

#include <sstream>

namespace next_best_view_planner
{

MGPGraph::MGPGraph()
{
 _num_nodes = 0;
}

MGPGraph::~MGPGraph()
{
	std::cout << "Closed MGPGraph with ID = " << this->getGraphId() << std::endl;

}

void MGPGraph::computeJohnson()
{   static const int V = 60;
    
    // std::vector < int >d(V, (std::numeric_limits < int >::max)());
//  int D[V][V];
  //int D[_num_nodes][_num_nodes]; 

//  boost::johnson_all_pairs_shortest_paths(graph, D,boost::distance_map(&d[0]));
  
  
}

void MGPGraph::setCurrentState(vertex_t currentState)
{
	this->currentState = currentState;
}

vertex_t MGPGraph::getCurrentState()
{
	return this->currentState;
}

bool MGPGraph::isStatePoolEmpty()
{
	//return num_vertices(graph) == 0;
}

Graph* MGPGraph::getGraphPtr()
{
	return &graph;
}

string MGPGraph::getGraphId()
{
	return this->graphID;
}

void MGPGraph::setGraphId(string graphID)
{
	this->graphID = graphID;
}

void MGPGraph::printEdgeWeights() {
 
  std::pair<edge_iterator, edge_iterator> edges = boost::edges(graph);
 
/*		
  typedef boost::property_map<Graph, boost::edge_weight_t>::const_type WeightMap;
  // The following line will not compile:
  WeightMap weights = boost::get(boost::edge_weight_t(), graph);
*/

//boost::get(boost::edge_property, graph);
// std:pair<> = boost::get(MGPEdge(), graph);
 
  edge_iterator edge;
  for (edge = edges.first; edge != edges.second; ++edge) {
    std::cout << *edges.first << std::endl;
  }
  
}


bool MGPGraph::addEdge(vertex_t from, vertex_t to, int weight, double euclidean_cost)
{
  
	bool added;
	edge_t edge;
	boost::tie(edge, added)= boost::add_edge(from, to, MGPEdge(weight,euclidean_cost), graph);
  
       // next_best_view_planner::MGPEdge e = boost::get(boost::edge_property_tag,graph);
	
	graph[edge].incrementTraverseCount();
	
	
	return added;
}

vertex_t MGPGraph::addVertex(rw::math::Transform3D<> pose)
{
	return boost::add_vertex(MGPNode(pose),graph);
}


void MGPGraph::removeVertex(vertex_t Vertex)
{
    boost:: clear_vertex(Vertex, graph);
    boost::remove_vertex(Vertex,graph);
}
boost::signal<void (bool)>& MGPGraph::getGraphChangedSignal(void)
{
	return this->_graphChanged;
}

void MGPGraph::signalGraphChanged()
{
	this->_graphChanged(true); // We are passing along TRUE but it is not used.
}

void MGPGraph::setCurrentNodeToFirstNode()
{
	if(!isStatePoolEmpty())
	{
	    vertex_iterator i, end;
	    boost::tie(i, end) = boost::vertices(graph);
	    vertex_t startNode = *i;
		if(startNode != boost::graph_traits<Graph>::null_vertex())
		{
			std::cout << "MGPGraph: Resetting graph " << this->getGraphId() << " to state " << graph[startNode].getNodeID() << std::endl;
			setCurrentState(startNode);
			signalGraphChanged();
		}
	}
}
void MGPGraph::reset()
{
	_firstRun = true;
	vertex_iterator i, end;
    for (boost::tie(i, end) = boost::vertices(graph); i != end; ++i)
	{
		graph[*i].setIsInVisualizationGraph(false);
	}
	edge_iterator iEdge, endEdge;
    for (boost::tie(iEdge, endEdge) = boost::edges(graph); iEdge != endEdge; ++iEdge)
	{
    	graph[*iEdge].setIsInVisualizationGraph(false);
	}
    setCurrentNodeToFirstNode();
}

bool MGPGraph::compareVertices(vertex_t vertex1, vertex_t vertex2)
{
	return graph[vertex1].getNodeID().compare(graph[vertex2].getNodeID()) != 0;
}


void MGPGraph::saveGraph(std::string file_name)
{
/*  using namespace boost;
  std::ofstream in(file_name.c_str());

  
  std::vector<std::string> NameVec; // for dot file names
  // Write out the graph
  boost::write_graphviz(in, graph, make_label_writer(&NameVec[0]));
  */
}


} /* namespace next_best_view_planner */
