#ifndef MGPCONF_H_
#define MGPCONF_H_

#include "MGPNode.h"// Needed for the typedefs below!
#include "MGPEdge.h"// Needed for the typedefs below!

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_iterator.hpp>
#include <boost/property_map/property_map.hpp>

#include <boost/scoped_ptr.hpp>

#include <boost/format.hpp>
#include <boost/functional/hash.hpp>

#include <curl/curl.h>

#include <map>
#include <vector>

using namespace std;

namespace next_best_view_planner
{

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, next_best_view_planner::MGPNode, next_best_view_planner::MGPEdge> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t; // A vertex is am opaque handle
typedef boost::graph_traits<Graph>::edge_descriptor edge_t;
typedef boost::graph_traits<Graph>::vertex_iterator vertex_iterator;
typedef boost::graph_traits<Graph>::edge_iterator edge_iterator;
typedef boost::property< Graph, next_best_view_planner::MGPEdge> edge_info_prop_t;
//typedef boost::property_map< Graph, next_best_view_planner::MGPEdge>::type edge_map;

}

#endif /* MGPCONF_H_ */
