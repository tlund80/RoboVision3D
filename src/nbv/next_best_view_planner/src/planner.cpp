/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2014  <copyright holder> <email>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#include "planner.h"
#include <../../../../RobWork/src/rw/math/Q.hpp>

namespace next_best_view_planner
{
  
Planner::Planner(boost::shared_ptr<workcell> workcell, boost::shared_ptr<PlannerConfiguration> Config) : _workcell(workcell), _config(Config)
{
  _samplerType = Planner::GAUSS;
  
   PlannerConfiguration *configuration = _config.get();
   
  if(_config->getPlannertype() == PlannerConfiguration::PRM)
  {
   
    PRMConfig* c = (PRMConfig*)configuration;
    _plannerType = Planner::PRM;
    
    std::cout << "Planner type is PRM!!" << std::endl;
    _motionPlanning_PRM.init(this, c);
  }else if (_config->getPlannertype() == PlannerConfiguration::RRT)
  {
     std::cout << "Planner type is RRT!!" << std::endl;
     RRTConfig* c = (RRTConfig*)configuration;
    _plannerType = Planner::RRT;
    _motionPlanning_RRT.init(this, c);
    
  }

}

Planner::~Planner()
{

}

void Planner::computeShortestPath(const std::vector<rw::math::Transform3D<> > &pose_list)
{
  boost::shared_ptr<MGPGraph> graph;
  graph.reset(new MGPGraph());
  
  std::vector<vertex_t> nodes;
  std::map<vertex_t, rw::math::Transform3D<> > node_map;
  
  graph->setGraphId("Multi_Path_Planning_Graph");
  std::cout << "Adding nodes to the graph!" << std::endl; 
  for(size_t i=0; i<pose_list.size();i++){
      vertex_t node_ref = graph->addVertex(pose_list[i]);
      std::pair<vertex_t,rw::math::Transform3D<> > pair(node_ref, pose_list[i]);
      node_map.insert(pair);
      nodes.push_back(node_ref);
  }
  
  std::cout << "Added " << pose_list.size() << " nodes to the graph" << std::endl; 
    
  for(size_t j = 0; j<node_map.size(); j++){
    for(size_t k = 0; k<node_map.size(); k++){
       if(j != k){
	 //node_map.find()
	 graph->addEdge(nodes[j],nodes[k], 1, 0.98765);
       }
    }
  }
   
 //  graph->computeJohnson();
 //graph->printEdgeWeights();
}
	
void Planner::sample_sphere(const rw::math::Transform3D<> &orgin, std::vector<rw::math::Transform3D<> > &pose_list)
{
  using namespace rw::math;
  //Samples the workspace around an object as an half sphere 
  double delta_angle_hor = Pi/8; 
  double delta_angle_ver = Pi/8;
  double radius =  0.30; 
  
  Q home(6, 3.141, -1.571, -1.571, -1.571, 1.571,1.571);
  std::vector<Q> solution;
  
  double x, y, z, roll, pitch, yaw;
  
  for(double i = 0; i<= 2*Pi; i = delta_angle_hor + i ){
      for(double j = Pi/2; j<=Pi  ; j = delta_angle_ver + j ){
	
	x = radius * cos(i)* sin(j);
	y = radius * sin(i)* sin(j);
	z = radius * cos(j);
	roll = i;
	pitch = j+Pi/2;
	yaw =0;
	
	Vector3D<> v(x,y,z);
	RPY<> _rpy(roll,pitch,yaw);
	Transform3D<> p(v, _rpy.toRotation3D());

	Transform3D<> result = orgin * p;
	
	//std::cout << result << std::endl;
	
	// Check configuration
	if(_workcell->solve_invkin(result,false,home,solution, true, true)){
	  if(j < Pi){
	      // it is not the top point in the sphere -> add point
	      pose_list.push_back(result); 
	  }else if(j == Pi && i == 0){
	     // 
	     pose_list.push_back(result); 
	  }
	}
     }
  } 
}

void Planner::sample_sphere_random(const rw::math::Transform3D<> &orgin, std::vector<rw::math::Transform3D<> > &pose_list)
{
  using namespace rw::math;
   
  int num_points = 50;
  float radius =  0.30; 
  float x, y, z, roll, pitch, yaw;
  
   boost::mt19937 rand_gen;
   rand_gen.seed(static_cast<unsigned int>(std::time(0)));
   
   // Create the distribution object.
   boost::uniform_on_sphere<float> unif_sphere(3);
    
   // This is what will actually supply drawn values.
   boost::variate_generator<boost::mt19937&, boost::uniform_on_sphere<float> >    random_on_sphere(rand_gen, unif_sphere); 
    
   Q home(6, 3.141, -1.571, -1.571, -1.571, 1.571,1.571);
   std::vector<Q> solution;
    int i = 0;
   while(i != num_points){
    std::vector<float> random_sphere_point = random_on_sphere();
	x = random_sphere_point.at(0)*radius;
	y = random_sphere_point.at(1)*radius;
	z = random_sphere_point.at(2)*radius;
	
	pitch = acos(z/radius)+Pi/2;
	roll = atan2(y,x);
	yaw =0;
	
	//std::cout << "r: " << roll << " " << "p: " << pitch << " " << "x: " << x << " " << "y: " << y << " " << "z: " << z << std::endl;
	
	if(z < 0 ){
	  Vector3D<> v(x,y,z);
	  RPY<> _rpy(roll,pitch,yaw);
	  Transform3D<> p(v, _rpy.toRotation3D());
	  Transform3D<> result = orgin * p;
	  
	  if(_workcell->solve_invkin(result,false,home,solution, true, true)){
	      pose_list.push_back(result);
	      i++;
	  }
	}
    } 
}

bool Planner::plan(const rw::math::Q &qstart, const rw::math::Q &qgoal, rw::trajectory::Path<Q> &path)
{
	std::string colFrameStr;

	if(_workcell->inCollision(qstart, colFrameStr)) {
		RW_THROW("In collision!!");
	}

	if(_workcell->inCollision(qgoal, colFrameStr)) {
		RW_THROW("In collision!!");
	}

	// BUG: planner type cannot be switched dynamically
	switch(_plannerType)
	{
	  case Planner::PRM:
		  return _motionPlanning_PRM.plan(qstart, qgoal, path);
	  case Planner::RRT:
		  return _motionPlanning_RRT.plan(qstart, qgoal, path);
	  default:
		  RW_THROW("Unknown planner type!");
		  break;
	}
	return false;
}
void Planner::MotionPlanning_PRM::init(Planner *motionPlanning,PRMConfig* config)
{
	_motionPlanning = motionPlanning;
	if(config->getCollisionCheckingStrategy() == PlannerConfiguration::LAZY)_collisionCheckingStrategy = rwlibs::pathplanners::PRMPlanner::LAZY; 
	else if(config->getCollisionCheckingStrategy() == PlannerConfiguration::NODECHECK)_collisionCheckingStrategy = rwlibs::pathplanners::PRMPlanner::NODECHECK; 
	else if(config->getCollisionCheckingStrategy() == PlannerConfiguration::FULL)_collisionCheckingStrategy = rwlibs::pathplanners::PRMPlanner::FULL; 
	else std::cout << "Unknown Collision Cheking Strategy requested in Planner::MotionPlanning_PRM::init()" << std::endl;

	if(config->getNeighborSearchStrategy() == PlannerConfiguration::BRUTE_FORCE) _neighborSearchStrategy = rwlibs::pathplanners::PRMPlanner::BRUTE_FORCE;
	else if(config->getNeighborSearchStrategy() == PlannerConfiguration::PARTIAL_INDEX_TABLE) _neighborSearchStrategy = rwlibs::pathplanners::PRMPlanner::PARTIAL_INDEX_TABLE;
	else std::cout << "Unknown Neighbor search Strategy requested in Planner::MotionPlanning_PRM::init()" << std::endl;
 
	if(config->getShortestPathSearchStrategy() == PlannerConfiguration::A_STAR) _shortestPathSearchStrategy = rwlibs::pathplanners::PRMPlanner::A_STAR;
	else if(config->getShortestPathSearchStrategy() == PlannerConfiguration::DIJKSTRA) _shortestPathSearchStrategy = rwlibs::pathplanners::PRMPlanner::DIJKSTRA;
	else std::cout << "Unknown Shortest Path search Strategy requested in Planner::MotionPlanning_PRM::init()" << std::endl;
 
	_roadmapNodecount = config->getRoadmapNodecount();
	_maxtime = config->getMaxTime();
	_resolution = config->getResolution();
}
bool Planner::MotionPlanning_PRM::plan(const rw::math::Q &qstart, const rw::math::Q &qgoal, rw::trajectory::Path<Q> &path)
{
	using namespace rwlibs::pathplanners;
	using namespace rw::trajectory;
	using namespace rw::pathplanning;

	std::cout << "PRM Planner called!" << std::endl;
	workcell* workcell = _motionPlanning->_workcell.get();

	QConstraint::Ptr qConstraintPtr = QConstraint::make( workcell->getCollisionDetector(), workcell->getRobot(), workcell->getState());
	QSampler::Ptr qSamplerPtr = _motionPlanning->getSampler(qstart, qgoal);
	PRMPlanner planner(qConstraintPtr, qSamplerPtr, _resolution, *(workcell->getRobot()), workcell->getState());

	planner.setCollisionCheckingStrategy(_collisionCheckingStrategy);
	planner.setNeighSearchStrategy(_neighborSearchStrategy);
	planner.setShortestPathSearchStrategy(_shortestPathSearchStrategy);
	planner.buildRoadmap(_roadmapNodecount);

	return planner.query(qstart, qgoal, path, _maxtime);
	
}

void Planner::MotionPlanning_RRT::init(Planner *motionPlanning,RRTConfig* config)
{
  _motionPlanning = motionPlanning;
  _extend = config->getExtend();
  _resolution = config->getResolution();
  std::cout << "_extend: " << _extend << std::endl;
  std::cout << "_resolution: " << _resolution << std::endl;
}

bool Planner::MotionPlanning_RRT::plan(const rw::math::Q &qstart, const rw::math::Q &qgoal, rw::trajectory::Path<rw::math::Q> &path)
{
        using namespace rwlibs::pathplanners;
	using namespace rw::trajectory;
	using namespace rw::pathplanning;
	using namespace rw::math;
	
	std::cout << "RRT Planner called!" << std::endl;
	workcell *workcell = _motionPlanning->_workcell.get();
	EuclideanMetric<Q>::Ptr metricPtr(new EuclideanMetric<Q>());
	QConstraint::Ptr qConstraintPtr = QConstraint::make(workcell->getCollisionDetector(), workcell->getRobot(), workcell->getState());
	QEdgeConstraint::Ptr qEdgeConstraintPtr = QEdgeConstraint::make(qConstraintPtr, metricPtr, _resolution);
	PlannerConstraint plannerConstraint(qConstraintPtr, qEdgeConstraintPtr);
	QSampler::Ptr qSamplerPtr = _motionPlanning->getSampler(qstart, qgoal);
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(plannerConstraint, qSamplerPtr, metricPtr, _extend, RRTPlanner::RRTConnect);

	return planner->query(qstart, qgoal, path);
}

rw::pathplanning::QSampler::Ptr Planner::getSampler(const Q &qstart, const Q &qgoal) const
{
	switch(_samplerType)
	{
	case Planner::GAUSS:
		return rw::common::ownedPtr(new Planner::GaussSampler(qstart, qgoal, 1.0));
	case Planner::UNIFORM:
		return rw::pathplanning::QSampler::makeUniform(_workcell->getRobot());
	}

	return rw::pathplanning::QSampler::makeUniform(_workcell->getRobot());
}


} /* namespace next_best_view_planner */
