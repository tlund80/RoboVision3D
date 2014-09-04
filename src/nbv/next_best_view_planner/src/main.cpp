/********************************************************************************************************************
 *
 * Copyright (c) 2014
 *
 * Danish Technological Institute (DTI)
 *
 *-------------------------------------------------------------------------------------------------------------------
 *
 * @file       main.cpp
 * @Author     Thomas Sølund (thso@dti.dk)
 * @brief
 * @details
 * @addtogroup
 *
 * $Revision: 4575 $
 * $Date: 2014-05-13 15:37:03 +0100 (Thu, 16 Jan 2014) $
 * $Author: thso $
 * $Id: main.cpp 4575 2014-05-13 14:37:03Z mmo $
 *
 *
 *-------------------------------------------------------------------------------------------------------------------
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Danish Technolocial Institute (DTI) nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * This program is copyrighted software: you can redistribute it and/or modify
 * it under the terms and protection of the Consortium Agreements and/or NDAs
 * assosiated to collaborative projects with DTI. Redistribution to and use by
 * parties outside of such an agreement is prohibited.
 *
 * This program is distributed in the hope that modifications of this software
 * and results achieved thereof will be reported back to DTI and the collaborative
 * projects. When modifying and evaluating this software, ensure that the author
 * have been notified.
 *
 *
 ********************************************************************************************************************/
#include "ros/ros.h"
#include "workcell.h"

#include <boost/shared_ptr.hpp>
#include "planner.h"
#include "ros_interface.h"
#include "path_optimization.h"

using namespace next_best_view_planner;



int main(int argc, char** argv)
{
	ros::init(argc, argv, "next_best_view_planner");
	ros::NodeHandle n("~");
	
	std::string _workCellFileName;
	std::string _deviceName;
	std::string _planning_frame;
	std::string _planner_type;
	std::string _collisionCheckingStrategy;
	std::string _neighborSearchStrategy;
	std::string _shortestPathSearchStrategy;
	std::string _optimization;
	int _roadmapNodecount;
	double _maxtime;
	double _extend;
	double _resolution;
	
	double _clearance_step_size = 0.1;
	int _clearance_max_count = 0;
	double _clearance_max_time = 20.0;
	double _pathlength_resolution = 0.01;
	
	int _collisionChk = 0;
	int _neighborSearch = 0;
	int _shortestPath = 0;
	int _plannertype = 0;
	int _optimizationType = 0;
	

	
	if(!n.getParam("workcell_file_name",_workCellFileName)){
	    ROS_ERROR("Next_best_view_planner: Could not get 'workcell_file_name' parameter from the parameter server");
	    return 0;}    
	if(!n.getParam("device_name",_deviceName)){
	    ROS_ERROR("Next_best_view_planner: Could not get 'device_name' parameter from the parameter server");
	    return 0;}
	if(!n.getParam("planning_frame",_planning_frame)){
	    ROS_ERROR("Next_best_view_planner: Could not get 'device_name' parameter from the parameter server");
	    return 0;}
	if(n.getParam("planner_type",_planner_type)){
	 
	  if( _planner_type.compare("PRM") == 0 || _planner_type.compare("prm") == 0)
	  {
	    _plannertype = PlannerConfiguration::PRM;
	    if(!n.getParam("collisionCheckingStrategy",_collisionCheckingStrategy)){
	    ROS_ERROR("Next_best_view_planner: Could not get 'collisionCheckingStrategy' parameter from the parameter server");
	    return 0;}
	    
	    if(!n.getParam("neighborSearchStrategy",_neighborSearchStrategy)){
	    ROS_ERROR("Next_best_view_planner: Could not get 'neighborSearchStrategy' parameter from the parameter server");
	    return 0;}
	    
	    if(!n.getParam("shortestPathSearchStrategy",_shortestPathSearchStrategy)){
	    ROS_ERROR("Next_best_view_planner: Could not get 'shortestPathSearchStrategy' parameter from the parameter server");
	    return 0;}
	    
	    if(!n.getParam("roadmapNodecount",_roadmapNodecount)){
	    ROS_ERROR("Next_best_view_planner: Could not get 'roadmapNodecount' parameter from the parameter server");
	    return 0;}
	    
	    if(!n.getParam("maxtime",_maxtime)){
	    ROS_ERROR("Next_best_view_planner: Could not get 'maxtime' parameter from the parameter server");
	    return 0;}
	    
	  }else if(_planner_type.compare("RRT") == 0 || _planner_type.compare("rrt") == 0){
	    _plannertype = PlannerConfiguration::RRT;
	    if(!n.getParam("extend",_extend)){
	    ROS_ERROR("Next_best_view_planner: Could not get 'extend' parameter from the parameter server");
	    return 0;} 
	  }
	  
	}else{
	  ROS_ERROR("Next_best_view_planner: Could not get 'planner_type' parameter from the parameter server");
	    return 0;
       }    
       if(!n.getParam("resolution",_resolution)){
	    ROS_ERROR("Next_best_view_planner: Could not get 'resolution' parameter from the parameter server");
	    return 0;}
       
       if(n.getParam("optimization",_optimization))
       {
	  if( _optimization.compare("CLERANCE") == 0 || _optimization.compare("clerance") == 0)
	  {
	    _optimizationType = OptimizationConfiguration::Clearance;
	     if(!n.getParam("clearance_step_size",_clearance_step_size)){
	      ROS_ERROR("Next_best_view_planner: Could not get 'clearance_step_size' parameter from the parameter server");
	      return 0;}
	      
	      if(!n.getParam("clearance_max_count",_clearance_max_count)){
	      ROS_ERROR("Next_best_view_planner: Could not get 'clearance_max_count' parameter from the parameter server");
	      return 0;}
	      
	      if(!n.getParam("clearance_max_time",_clearance_max_time)){
	      ROS_ERROR("Next_best_view_planner: Could not get 'clearance_max_time' parameter from the parameter server");
	      return 0;}
	      
	      
	  }else if( _optimization.compare("PATHLENGTH") == 0 || _optimization.compare("pathlength") == 0){
	      _optimizationType = OptimizationConfiguration::PathLength;
	      if(!n.getParam("pathlength_resolution",_pathlength_resolution)){
	      ROS_ERROR("Next_best_view_planner: Could not get 'pathlength_resolution' parameter from the parameter server");
	      return 0;}
		

         }else if( _optimization.compare("ALL") == 0 || _optimization.compare("all") == 0){
	      _optimizationType = OptimizationConfiguration::All;
	      
	      if(!n.getParam("pathlength_resolution",_pathlength_resolution)){
	      ROS_ERROR("Next_best_view_planner: Could not get 'pathlength_resolution' parameter from the parameter server");
	      return 0;}
	      
	       if(!n.getParam("clearance_step_size",_clearance_step_size)){
	      ROS_ERROR("Next_best_view_planner: Could not get 'clearance_step_size' parameter from the parameter server");
	      return 0;}
	      
	      if(!n.getParam("clearance_max_count",_clearance_max_count)){
	      ROS_ERROR("Next_best_view_planner: Could not get 'clearance_max_count' parameter from the parameter server");
	      return 0;}
	      
	      if(!n.getParam("clearance_max_time",_clearance_max_time)){
	      ROS_ERROR("Next_best_view_planner: Could not get 'clearance_max_time' parameter from the parameter server");
	      return 0;}
	      
       }else{
	    ROS_ERROR("Next_best_view_planner: Could not get 'optimization' parameter from the parameter server");
	    return 0;}
	  }
      
	//Parse values
	if(_collisionCheckingStrategy.compare("LAZY")== 0 || 
	   _collisionCheckingStrategy.compare("lazy")== 0) _collisionChk = PlannerConfiguration::LAZY; 
	else if(_collisionCheckingStrategy.compare("NODECHECK")== 0 || 
		_collisionCheckingStrategy.compare("nodecheck")== 0) _collisionChk = PlannerConfiguration::NODECHECK;
	else if(_collisionCheckingStrategy.compare("FULL")== 0 || 
		_collisionCheckingStrategy.compare("full")== 0) _collisionChk = PlannerConfiguration::FULL;
	
	if(_neighborSearchStrategy.compare("BRUTE_FORCE")== 0 || 
	   _neighborSearchStrategy.compare("brute_force")== 0) _neighborSearch = PlannerConfiguration::BRUTE_FORCE; 
	else if(_neighborSearchStrategy.compare("PARTIAL_INDEX_TABLE")== 0 || 
	        _neighborSearchStrategy.compare("partial_index_table")== 0) _neighborSearch = PlannerConfiguration::PARTIAL_INDEX_TABLE; 
	
       if(_shortestPathSearchStrategy.compare("A_STAR")== 0 || 
	   _shortestPathSearchStrategy.compare("a_star")== 0) _shortestPath = PlannerConfiguration::A_STAR; 
       else if(_shortestPathSearchStrategy.compare("DIJKSTRA")== 0 || 
	       _shortestPathSearchStrategy.compare("dijkstra")== 0) _shortestPath = PlannerConfiguration::DIJKSTRA; 
	
       
	boost::shared_ptr<workcell> _workcell;
	_workcell.reset(new workcell());
	_workcell->init(_workCellFileName,_deviceName, _planning_frame);
       
       boost::shared_ptr<PlannerConfiguration> p;
	
       if(_plannertype == PlannerConfiguration::PRM){
	    boost::shared_ptr<PRMConfig> _config;
	    _config.reset(new PRMConfig());
	
	    _config->setCollisionCheckingStrategy(_collisionChk);
	    _config->setMaxTime(_maxtime);
	    _config->setNeighborSearchStrategy(_neighborSearch);
	    _config->setResolution(_resolution);
	    _config->setRoadmapNodecount(_roadmapNodecount);
	    _config->setShortestPathSearchStrategy(_shortestPath);
      
	    p = _config; 
       }else if(_plannertype == PlannerConfiguration::RRT){
	  boost::shared_ptr<RRTConfig> _config;
	    _config.reset(new RRTConfig());
	    
	 _config->setEsxtend(_extend);
	 _config->setResolution(_resolution);
	 
	   p = _config; 
       }
       
       boost::shared_ptr<OptimizationConfiguration> _o;
       
       if(_optimizationType == OptimizationConfiguration::Clearance)
       {
	 boost::shared_ptr<ClearanceConfig> _cleareance_config;
	    _cleareance_config.reset(new ClearanceConfig());
	    
	    _cleareance_config->setMaxCount(_clearance_max_count);
	    _cleareance_config->setMaxTime(_clearance_max_time);
	    _cleareance_config->setStepsize(_clearance_step_size);
	    _o = _cleareance_config;
       }else if(_optimizationType == OptimizationConfiguration::PathLength)
       {
       
        boost::shared_ptr<PathLengthConfig> _path_length_config;
	    _path_length_config.reset(new PathLengthConfig());
	    
	    _path_length_config->setResolution(_pathlength_resolution);
	    _o = _path_length_config;
       }    
	
       boost::shared_ptr<Path_optimization> _path_optimization;
	_path_optimization.reset(new Path_optimization(_workcell,_o));
       
	boost::shared_ptr<Planner> _planner;
	_planner.reset(new Planner(_workcell,p));
	
	boost::shared_ptr<ros_interface> _ros_interface;
	_ros_interface.reset(new ros_interface(_planner,_workcell,_path_optimization, n));
	if(!_ros_interface->init()){
	 return 0; 
	}
	
	ROS_INFO("Next_best_view_planner is running!!");
	ros::spin();

	return 0;
}
